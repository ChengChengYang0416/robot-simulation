#include <windows.h>
#include <cstdint>
#include <vector>
#include "NativeOccView.h"
#include "../Interaction/MouseInteractor.h"
#include "../Kinematics/RobotKinematics.h"
#include "../Kinematics/RobotPartDef.h"
#include "../Kinematics/TcpPoseSolver.h"
#include "../Scene/SceneRepository.h"
#include "../Scene/StepLoader.h"
#include <AIS_InteractiveContext.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <gp_Trsf.hxx>
#include <Quantity_Color.hxx>

struct NativeOccView::Impl
{
	HWND hwnd = nullptr;

	Handle( Aspect_DisplayConnection ) displayConnection;
	Handle( OpenGl_GraphicDriver ) graphicDriver;
	Handle( V3d_Viewer ) viewer;
	Handle( V3d_View ) view;
	Handle( AIS_InteractiveContext ) context;

	Scene::SceneRepository repo;
	Interaction::MouseInteractor mouse;
	OccBridge::RobotKinematics kin;

	// Maps part index -> SceneRepository slot id; -1 when the STEP file failed to
	// load. Replaces the legacy convention of pushing null AIS_Shape placeholders
	// to keep shapes vector in lock-step with part definitions.
	std::vector<int> partToSlot;
};

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
	m_impl->repo.attach( m_impl->context );
	m_impl->mouse.attach( m_impl->view, m_impl->context );

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

	const auto slot = m_impl->repo.addShape( *shape );
	if( !slot.isValid ) {
		return false;
	}
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
	m_impl->partToSlot.assign( partCount, -1 );
	return true;
}

bool NativeOccView::loadRobotPart( int index )
// Reads one STEP file via Scene::StepLoader and registers the resulting shape with
// SceneRepository. On failure, partToSlot[index] stays at -1 and updateRobotTransforms()
// simply skips that part (no AIS placeholder needed).
{
	const auto& parts = m_impl->kin.parts();
	if( index < 0 || index >= static_cast<int>( parts.size() ) ) {
		return false;
	}

	const auto& part = parts[ index ];

	Scene::StepLoader loader;
	auto shape = loader.read( part.filePath );
	if( !shape ) {
		return false;
	}

	// Display as shaded (mode 1). SceneRepository batches the call with Standard_False
	// so the viewer is not redrawn after each part; endRobotArm() does one final
	// updateViewer() for the whole batch.
	const Quantity_Color qColor(
		part.colorR / 255.0,
		part.colorG / 255.0,
		part.colorB / 255.0,
		Quantity_TOC_sRGB );
	const auto slot = m_impl->repo.addColoredShape( *shape, qColor );
	if( !slot.isValid ) {
		return false;
	}
	m_impl->partToSlot[ index ] = slot.slotId;
	return true;
}

void NativeOccView::endRobotArm( void )
// Asks SceneRepository to create the TCP trihedron, then runs the first transform pass
// and fits the camera. The trihedron's pose is set by updateRobotTransforms().
{
	if( !m_impl->kin.parts().empty() ) {
		m_impl->repo.ensureTcpTrihedron();
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
// Asks RobotKinematics for the cumulative DH chain, then pushes each part's final
// transform (DH * offset) into SceneRepository via the partToSlot map. The TCP
// trihedron pose is updated through the repository as well.
{
	if( m_impl->kin.parts().empty() ) {
		return;
	}

	const auto& cumulative = m_impl->kin.computeCumulative();
	const int n = static_cast<int>( cumulative.size() );

	for( int i = 0; i < n; ++i ) {
		const int slot = ( i < static_cast<int>( m_impl->partToSlot.size() ) )
			? m_impl->partToSlot[ i ] : -1;
		if( slot < 0 ) {
			continue;
		}
		m_impl->repo.setTransform( slot, m_impl->kin.computeFinal( i ) );
	}

	if( auto tcp = m_impl->kin.tcpFrame() ) {
		m_impl->repo.setTcpTransform( *tcp );
	}

	m_impl->repo.updateViewer();
}

void NativeOccView::clearScene( void )
// Asks the repository to remove all displayed objects, then resets kinematics and
// the part-to-slot map. The repository keeps its context attachment for reuse.
{
	m_impl->repo.clear();
	m_impl->kin.configure( {}, {} );
	m_impl->partToSlot.clear();
	m_impl->repo.updateViewer();
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
	if( out == nullptr ) {
		return false;
	}

	auto tcp = m_impl->repo.tcpFrame();
	if( !tcp ) {
		return false;
	}

	const auto pose = OccBridge::solveTcpPose( *tcp );
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
// Forwards to MouseInteractor, which owns the rotate / pan state machine.
{
	m_impl->mouse.onMouseDown( x, y, button );
}

void NativeOccView::onMouseMove( int x, int y, int buttonMask )
// Forwards to MouseInteractor; with no active drag it also drives hover highlight.
{
	m_impl->mouse.onMouseMove( x, y, buttonMask );
}

void NativeOccView::onMouseUp( void )
// Clears the active rotate / pan flags in MouseInteractor.
{
	m_impl->mouse.onMouseUp();
}

void NativeOccView::onMouseWheel( int delta )
// Forwards to MouseInteractor::onMouseWheel for zoom-in / zoom-out.
{
	m_impl->mouse.onMouseWheel( delta );
}
