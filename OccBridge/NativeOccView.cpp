#include <windows.h>
#include <cstdint>
#include <vector>
#include "NativeOccView.h"
#include "../Kinematics/RobotKinematics.h"
#include "../Kinematics/RobotPartDef.h"
#include "../Kinematics/TcpPoseSolver.h"
#include "../Scene/StepLoader.h"
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Geom_Axis2Placement.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <gp_Trsf.hxx>
#include <Quantity_Color.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

enum class MouseButton : int {
	Left   = 1048576,
	Middle = 4194304
};

struct NativeOccView::Impl
{
	HWND hwnd = nullptr;
	int lastX = 0;
	int lastY = 0;
	bool isRotating = false;
	bool isPanning  = false;

	Handle( Aspect_DisplayConnection ) displayConnection;
	Handle( OpenGl_GraphicDriver ) graphicDriver;
	Handle( V3d_Viewer ) viewer;
	Handle( V3d_View ) view;
	Handle( AIS_InteractiveContext ) context;
	std::vector<Handle( AIS_Shape )> shapes;

	std::vector<TopoDS_Shape> originalShapes;
	OccBridge::RobotKinematics kin;

	Handle( AIS_Trihedron ) tcpTrihedron;
	gp_Trsf tcpTrsf;
	bool hasTcp = false;
};

static constexpr double kZoomFactor = 1.15;

NativeOccView::NativeOccView( void )
	: m_impl( new Impl() )
// Allocates the PIMPL object; all OCCT members are default-initialized here
{
}

NativeOccView::~NativeOccView( void )
// Releases the PIMPL object; OCCT Handles decrement their ref-count in Impl's destructor
{
	delete m_impl;
}

void NativeOccView::initialize( HWND hwnd )
// Creates the OCCT driver, Viewer, Context, and View, then binds them to the Win32 window
{
	m_impl->hwnd = hwnd;

	// OCCT rendering stack (bottom to top):
	//   DisplayConnection -> GraphicDriver -> Viewer -> View / InteractiveContext
	// The Viewer owns lights and shared state; the Context manages AIS objects.
	m_impl->displayConnection = new Aspect_DisplayConnection();
	m_impl->graphicDriver = new OpenGl_GraphicDriver( m_impl->displayConnection );
	m_impl->viewer = new V3d_Viewer( m_impl->graphicDriver );
	m_impl->viewer->SetDefaultLights();
	m_impl->viewer->SetLightOn();
	m_impl->context = new AIS_InteractiveContext( m_impl->viewer );
	m_impl->view = m_impl->viewer->CreateView();

	// Bind the OCCT view to the host HWND. WNT_Window must be mapped before the
	// first redraw, otherwise OpenGL has no surface to draw onto.
	Handle( WNT_Window ) window = new WNT_Window( reinterpret_cast<Aspect_Handle>( m_impl->hwnd ) );
	m_impl->view->SetWindow( window );
	if( !window->IsMapped() ) {
		window->Map();
	}

	m_impl->view->SetBackgroundColor( Quantity_NOC_GRAY30 );
	// Corner trihedron: per-axis colors (X=blue, Y=green, Z=red) must be configured
	// via ZBufferTriedronSetup before calling TriedronDisplay.
	m_impl->view->ZBufferTriedronSetup( Quantity_NOC_BLUE, Quantity_NOC_GREEN, Quantity_NOC_RED );
	m_impl->view->TriedronDisplay( Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08, V3d_ZBUFFER );
	m_impl->view->MustBeResized();
	m_impl->view->SetProj( V3d_XposYnegZpos );
	m_impl->view->Redraw();
}

void NativeOccView::resize( int /*width*/, int /*height*/ )
// Notifies the OCCT View that the window size changed and triggers a redraw
{
	if( !m_impl->view.IsNull() ) {
		m_impl->view->MustBeResized();
		m_impl->view->Redraw();
	}
}

void NativeOccView::redraw( void )
// Requests an immediate OCCT View redraw without recalculating scene structure
{
	if( !m_impl->view.IsNull() ) {
		m_impl->view->Redraw();
	}
}

bool NativeOccView::loadStep( const wchar_t* filePath, bool append )
// Reads a STEP file via Scene::StepLoader and displays the merged root shape.
{
	if( m_impl->context.IsNull() ) {
		return false;
	}

	if( !append ) {
		clearScene();
	}

	Scene::StepLoader loader;
	auto shape = loader.read( filePath );
	if( !shape ) {
		return false;
	}

	Handle( AIS_Shape ) aisShape = new AIS_Shape( *shape );
	m_impl->context->Display( aisShape, Standard_False );
	m_impl->context->SetDisplayMode( aisShape, 1, Standard_False );
	m_impl->shapes.push_back( aisShape );
	fitAll();
	return true;
}

bool NativeOccView::beginRobotArm( const RobotPartDef* parts, int partCount,
								   const int* axisToPartMap, int mapCount )
// Clears the scene and configures the kinematics solver with the part definitions and
// axis-to-part mapping. RobotKinematics owns the part list and joint angles from here on.
{
	if( m_impl->context.IsNull() ) {
		return false;
	}

	clearScene();

	std::vector<RobotPartDef> partVec( parts, parts + partCount );
	std::vector<std::pair<int, int>> axisMap;
	axisMap.reserve( mapCount / 2 );
	for( int i = 0; i < mapCount; i += 2 ) {
		axisMap.emplace_back( axisToPartMap[ i ], axisToPartMap[ i + 1 ] );
	}
	m_impl->kin.configure( std::move( partVec ), std::move( axisMap ) );
	return true;
}

bool NativeOccView::loadRobotPart( int index )
// Reads one STEP file via Scene::StepLoader, creates its AIS_Shape with color,
// and adds it to the context.
{
	const auto& parts = m_impl->kin.parts();
	if( index < 0 || index >= static_cast<int>( parts.size() ) ) {
		return false;
	}

	const auto& part = parts[ index ];

	// On read failure, push empty placeholders so that vector indices remain in
	// lock-step with kin.parts(). updateRobotTransforms() then skips null entries.
	Scene::StepLoader loader;
	auto shape = loader.read( part.filePath );
	if( !shape ) {
		m_impl->originalShapes.emplace_back();
		m_impl->shapes.push_back( Handle( AIS_Shape )() );
		return false;
	}

	m_impl->originalShapes.push_back( *shape );

	// Display as shaded (mode 1). All Display/SetColor calls pass Standard_False
	// so the viewer is not redrawn after each part; endRobotArm() does one final
	// UpdateCurrentViewer for the whole batch.
	Handle( AIS_Shape ) aisShape = new AIS_Shape( *shape );
	const Quantity_Color qColor(
		part.colorR / 255.0,
		part.colorG / 255.0,
		part.colorB / 255.0,
		Quantity_TOC_sRGB );
	m_impl->context->Display( aisShape, 1, -1, Standard_False );
	m_impl->context->SetColor( aisShape, qColor, Standard_False );
	m_impl->shapes.push_back( aisShape );
	return true;
}

void NativeOccView::endRobotArm( void )
// Computes cumulative DH transforms, creates the TCP trihedron, and fits the camera
{
	// Build a TCP (Tool Center Point) trihedron that will be re-positioned by
	// updateRobotTransforms() to follow the last DH frame in real time.
	if( !m_impl->context.IsNull() && !m_impl->kin.parts().empty() ) {
		Handle( Geom_Axis2Placement ) axis = new Geom_Axis2Placement(
			gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ), gp_Dir( 1, 0, 0 ) );
		m_impl->tcpTrihedron = new AIS_Trihedron( axis );
		m_impl->tcpTrihedron->SetSize( 80.0 );
		m_impl->tcpTrihedron->SetDatumPartColor( Prs3d_DatumParts_XAxis, Quantity_Color( Quantity_NOC_BLUE ) );
		m_impl->tcpTrihedron->SetDatumPartColor( Prs3d_DatumParts_YAxis, Quantity_Color( Quantity_NOC_GREEN ) );
		m_impl->tcpTrihedron->SetDatumPartColor( Prs3d_DatumParts_ZAxis, Quantity_Color( Quantity_NOC_RED ) );
		m_impl->context->Display( m_impl->tcpTrihedron, Standard_False );
	}
	updateRobotTransforms();
	fitAll();
}

void NativeOccView::setJointAngle( int axisIndex, double angleDeg )
// Forwards the joint update to the kinematics solver, then re-applies transforms.
{
	m_impl->kin.setJointAngle( axisIndex, angleDeg );
	updateRobotTransforms();
}

void NativeOccView::updateRobotTransforms( void )
// Asks RobotKinematics for the cumulative DH chain and applies each part's final
// transform (DH * offset) to its AIS_Shape via SetLocalTransformation. Also re-positions
// the TCP trihedron and caches the TCP frame for getTcpPose().
{
	if( m_impl->context.IsNull() || m_impl->kin.parts().empty() ) {
		return;
	}

	const auto& cumulative = m_impl->kin.computeCumulative();
	const int n = static_cast<int>( cumulative.size() );

	for( int i = 0; i < n; ++i ) {
		if( i >= static_cast<int>( m_impl->shapes.size() ) || m_impl->shapes[ i ].IsNull() ) {
			continue;
		}
		// SetLocalTransformation updates in-place without re-tessellating geometry,
		// so it is cheap enough to call on every slider change.
		m_impl->shapes[ i ]->SetLocalTransformation( m_impl->kin.computeFinal( i ) );
	}

	if( !m_impl->tcpTrihedron.IsNull() ) {
		if( auto tcp = m_impl->kin.tcpFrame() ) {
			m_impl->tcpTrihedron->SetLocalTransformation( *tcp );
			m_impl->tcpTrsf = *tcp;
			m_impl->hasTcp = true;
		}
	}

	m_impl->context->UpdateCurrentViewer();
}

void NativeOccView::clearScene( void )
// Removes all AIS objects from the Context and clears all internal state containers
{
	if( m_impl->context.IsNull() ) {
		return;
	}

	for( const auto& shape : m_impl->shapes ) {
		if( !shape.IsNull() ) {
			m_impl->context->Remove( shape, Standard_False );
		}
	}
	m_impl->shapes.clear();
	m_impl->originalShapes.clear();
	m_impl->kin.configure( {}, {} );
	if( !m_impl->tcpTrihedron.IsNull() ) {
		m_impl->context->Remove( m_impl->tcpTrihedron, Standard_False );
		m_impl->tcpTrihedron.Nullify();
	}
	m_impl->hasTcp = false;
	m_impl->context->UpdateCurrentViewer();
	redraw();
}

void NativeOccView::fitAll( void )
// Calls FitAll and ZFitAll to ensure the scene is fully visible in both axes
{
	if( !m_impl->view.IsNull() ) {
		m_impl->view->FitAll();
		m_impl->view->ZFitAll();
		m_impl->view->Redraw();
	}
}

bool NativeOccView::getTcpPose( double out[6] ) const
// Returns the cached TCP pose as [x, y, z, rx, ry, rz] in mm and degrees.
// Delegates the matrix decomposition to OccBridge::solveTcpPose so the Euler
// convention stays in one place. Returns false if no robot is loaded.
{
	if( out == nullptr || !m_impl->hasTcp ) {
		return false;
	}

	const auto pose = OccBridge::solveTcpPose( m_impl->tcpTrsf );
	for( int i = 0; i < 6; ++i ) {
		out[ i ] = pose[ i ];
	}
	return true;
}

void NativeOccView::setViewIso( void )
// Sets projection to X+Y-Z+ for an isometric look, then fits all
{
	if( !m_impl->view.IsNull() ) {
		m_impl->view->SetProj( V3d_XposYnegZpos );
		fitAll();
	}
}

void NativeOccView::setViewTop( void )
// Sets projection to Z+ (top-down) then fits all
{
	if( !m_impl->view.IsNull() ) {
		m_impl->view->SetProj( V3d_Zpos );
		fitAll();
	}
}

void NativeOccView::onMouseDown( int x, int y, int button )
// Records the start position; left button begins OCCT rotation, middle button begins pan
{
	m_impl->lastX = x;
	m_impl->lastY = y;

	if( button == static_cast<int>( MouseButton::Left ) ) {
		m_impl->isRotating = true;
		if( !m_impl->view.IsNull() ) {
			m_impl->view->StartRotation( x, y );
		}
	} else if( button == static_cast<int>( MouseButton::Middle ) ) {
		m_impl->isPanning = true;
	}
}

void NativeOccView::onMouseMove( int x, int y, int /*buttonMask*/ )
// Executes rotation or pan based on active flags; otherwise updates cursor highlight
{
	if( m_impl->view.IsNull() || m_impl->context.IsNull() ) {
		return;
	}

	if( m_impl->isRotating ) {
		m_impl->view->Rotation( x, y );
	} else if( m_impl->isPanning ) {
		m_impl->view->Pan( x - m_impl->lastX, m_impl->lastY - y );
		m_impl->lastX = x;
		m_impl->lastY = y;
	} else {
		m_impl->context->MoveTo( x, y, m_impl->view, Standard_True );
	}
}

void NativeOccView::onMouseUp( void )
// Clears rotation and pan flags, ending the interactive operation
{
	m_impl->isRotating = false;
	m_impl->isPanning = false;
}

void NativeOccView::onMouseWheel( int delta )
// Zooms the scene by a fixed factor based on wheel direction
{
	if( m_impl->view.IsNull() ) {
		return;
	}

	const double factor = (delta > 0) ? kZoomFactor : (1.0 / kZoomFactor);
	m_impl->view->SetZoom( factor );
	m_impl->view->Redraw();
}
