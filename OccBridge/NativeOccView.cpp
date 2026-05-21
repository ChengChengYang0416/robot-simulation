#include <windows.h>
#include <cstdint>
#include <string>
#include <vector>
#include <cmath>
#include "NativeOccView.h"
#include "RobotPartDef.h"
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Geom_Axis2Placement.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <gp_Trsf.hxx>
#include <Quantity_Color.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

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

	std::vector<RobotPartDef> partDefs;
	std::vector<TopoDS_Shape> originalShapes;
	std::vector<std::pair<int, int>> axisToPartMap;
	std::vector<double> jointAngles;

	Handle( AIS_Trihedron ) tcpTrihedron;
};

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kDegToRad = kPi / 180.0;
static constexpr double kEpsilon = 1e-10;
static constexpr double kZoomFactor = 1.15;

static gp_Trsf makeDhTransform( double a, double alphaDeg, double d, double thetaDeg )
{
	const double alphaRad = alphaDeg * kDegToRad;
	const double thetaRad = thetaDeg * kDegToRad;

	// Standard DH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
	gp_Trsf rz;
	if( std::abs( thetaRad ) > kEpsilon ) {
		rz.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ) ), thetaRad );
	}

	gp_Trsf tz;
	if( std::abs( d ) > kEpsilon ) {
		tz.SetTranslation( gp_Vec( 0, 0, d ) );
	}

	gp_Trsf tx;
	if( std::abs( a ) > kEpsilon ) {
		tx.SetTranslation( gp_Vec( a, 0, 0 ) );
	}

	gp_Trsf rx;
	if( std::abs( alphaRad ) > kEpsilon ) {
		rx.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 1, 0, 0 ) ), alphaRad );
	}

	gp_Trsf result = rz;
	result.Multiply( tz );
	result.Multiply( tx );
	result.Multiply( rx );
	return result;
}

static gp_Trsf makeOffsetTransform( double tx, double ty, double tz,
								   double rxDeg, double ryDeg, double rzDeg )
{
	gp_Trsf rotX;
	if( std::abs( rxDeg ) > kEpsilon ) {
		rotX.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 1, 0, 0 ) ), rxDeg * kDegToRad );
	}

	gp_Trsf rotY;
	if( std::abs( ryDeg ) > kEpsilon ) {
		rotY.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 1, 0 ) ), ryDeg * kDegToRad );
	}

	gp_Trsf rotZ;
	if( std::abs( rzDeg ) > kEpsilon ) {
		rotZ.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ) ), rzDeg * kDegToRad );
	}

	gp_Trsf trans;
	if( std::abs( tx ) > kEpsilon || std::abs( ty ) > kEpsilon || std::abs( tz ) > kEpsilon ) {
		trans.SetTranslation( gp_Vec( tx, ty, tz ) );
	}

	// T * Rz * Ry * Rx: cancels DH rotation, placing CAD in original orientation
	gp_Trsf result = trans;
	result.Multiply( rotZ );
	result.Multiply( rotY );
	result.Multiply( rotX );
	return result;
}

static std::string wideToUtf8( const wchar_t* wide )
{
	const int len = WideCharToMultiByte( CP_UTF8, 0, wide, -1, nullptr, 0, nullptr, nullptr );
	std::string utf8( len - 1, '\0' );
	WideCharToMultiByte( CP_UTF8, 0, wide, -1, &utf8[ 0 ], len, nullptr, nullptr );
	return utf8;
}

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

	m_impl->displayConnection = new Aspect_DisplayConnection();
	m_impl->graphicDriver = new OpenGl_GraphicDriver( m_impl->displayConnection );
	m_impl->viewer = new V3d_Viewer( m_impl->graphicDriver );
	m_impl->viewer->SetDefaultLights();
	m_impl->viewer->SetLightOn();
	m_impl->context = new AIS_InteractiveContext( m_impl->viewer );
	m_impl->view = m_impl->viewer->CreateView();

	Handle( WNT_Window ) window = new WNT_Window( reinterpret_cast<Aspect_Handle>( m_impl->hwnd ) );
	m_impl->view->SetWindow( window );
	if( !window->IsMapped() ) {
		window->Map();
	}

	m_impl->view->SetBackgroundColor( Quantity_NOC_GRAY30 );
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
// Converts the path to UTF-8 and parses the geometry using STEPControl_Reader
{
	if( m_impl->context.IsNull() ) {
		return false;
	}

	if( !append ) {
		clearScene();
	}

	const std::string utf8Path = wideToUtf8( filePath );

	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile( utf8Path.c_str() );
	if( status != IFSelect_RetDone ) {
		return false;
	}

	const Standard_Integer roots = reader.TransferRoots();
	if( roots <= 0 ) {
		return false;
	}

	TopoDS_Shape shape = reader.OneShape();
	if( shape.IsNull() ) {
		return false;
	}

	Handle( AIS_Shape ) aisShape = new AIS_Shape( shape );
	m_impl->context->Display( aisShape, Standard_False );
	m_impl->context->SetDisplayMode( aisShape, 1, Standard_False );
	m_impl->shapes.push_back( aisShape );
	fitAll();
	return true;
}

bool NativeOccView::beginRobotArm( const RobotPartDef* parts, int partCount,
								   const int* axisToPartMap, int mapCount )
// Clears the scene and stores part definitions, axis mapping, and joint angles
{
	if( m_impl->context.IsNull() ) {
		return false;
	}

	clearScene();

	m_impl->partDefs.assign( parts, parts + partCount );
	m_impl->axisToPartMap.clear();
	for( int i = 0; i < mapCount; i += 2 ) {
		m_impl->axisToPartMap.emplace_back( axisToPartMap[ i ], axisToPartMap[ i + 1 ] );
	}
	m_impl->jointAngles.assign( 6, 0.0 );
	return true;
}

bool NativeOccView::loadRobotPart( int index )
// Reads one STEP file, creates its AIS_Shape with color, and adds it to the context
{
	if( index < 0 || index >= static_cast<int>( m_impl->partDefs.size() ) ) {
		return false;
	}

	const auto& part = m_impl->partDefs[ index ];
	const std::string utf8Path = wideToUtf8( part.filePath.c_str() );

	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile( utf8Path.c_str() );
	if( status != IFSelect_RetDone ) {
		m_impl->originalShapes.emplace_back();
		m_impl->shapes.push_back( Handle( AIS_Shape )() );
		return false;
	}

	const Standard_Integer roots = reader.TransferRoots();
	if( roots <= 0 ) {
		m_impl->originalShapes.emplace_back();
		m_impl->shapes.push_back( Handle( AIS_Shape )() );
		return false;
	}

	TopoDS_Shape shape = reader.OneShape();
	m_impl->originalShapes.push_back( shape );

	Handle( AIS_Shape ) aisShape = new AIS_Shape( shape );
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
	if( !m_impl->context.IsNull() && !m_impl->partDefs.empty() ) {
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
// Updates the given axis angle and triggers a full DH cumulative transform recalculation
{
	if( axisIndex < 0 || axisIndex >= 6 || m_impl->jointAngles.size() != 6 ) {
		return;
	}

	m_impl->jointAngles[ axisIndex ] = angleDeg;
	updateRobotTransforms();
}

void NativeOccView::updateRobotTransforms( void )
// Applies the cumulative DH transform multiplied by the offset transform to each part
// via SetLocalTransformation, updating positions in-place
{
	if( m_impl->context.IsNull() || m_impl->partDefs.empty() ) {
		return;
	}

	const int n = static_cast<int>( m_impl->partDefs.size() );

	// Build joint angle delta per part
	std::vector<double> partJointDelta( n, 0.0 );
	for( const auto& mapping : m_impl->axisToPartMap ) {
		const int axisIdx = mapping.first;
		const int partIdx = mapping.second;
		if( axisIdx >= 1 && axisIdx <= 6 && partIdx >= 0 && partIdx < n ) {
			partJointDelta[ partIdx ] = m_impl->jointAngles[ axisIdx - 1 ];
		}
	}

	// Compute cumulative DH transforms
	std::vector<gp_Trsf> dhCumulative( n );
	for( int i = 0; i < n; ++i ) {
		const double theta = m_impl->partDefs[ i ].dhTheta + partJointDelta[ i ];
		gp_Trsf dhLocal = makeDhTransform(
			m_impl->partDefs[ i ].dhA, m_impl->partDefs[ i ].dhAlpha,
			m_impl->partDefs[ i ].dhD, theta );

		const int parent = m_impl->partDefs[ i ].parentIdx;
		if( parent >= 0 && parent < n ) {
			dhCumulative[ i ] = dhCumulative[ parent ];
			dhCumulative[ i ].Multiply( dhLocal );
		} else {
			dhCumulative[ i ] = dhLocal;
		}
	}

	for( int i = 0; i < n; ++i ) {
		if( m_impl->shapes[ i ].IsNull() ) {
			continue;
		}

		gp_Trsf offsetTrsf = makeOffsetTransform(
			m_impl->partDefs[ i ].offset[ 0 ], m_impl->partDefs[ i ].offset[ 1 ], m_impl->partDefs[ i ].offset[ 2 ],
			m_impl->partDefs[ i ].offset[ 3 ], m_impl->partDefs[ i ].offset[ 4 ], m_impl->partDefs[ i ].offset[ 5 ] );

		gp_Trsf finalTrsf = dhCumulative[ i ];
		finalTrsf.Multiply( offsetTrsf );

		m_impl->shapes[ i ]->SetLocalTransformation( finalTrsf );
	}

	// Update TCP trihedron to the last DH frame
	if( !m_impl->tcpTrihedron.IsNull() ) {
		const int lastIdx = n - 1;
		m_impl->tcpTrihedron->SetLocalTransformation( dhCumulative[ lastIdx ] );
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
	m_impl->partDefs.clear();
	m_impl->axisToPartMap.clear();
	m_impl->jointAngles.clear();
	if( !m_impl->tcpTrihedron.IsNull() ) {
		m_impl->context->Remove( m_impl->tcpTrihedron, Standard_False );
		m_impl->tcpTrihedron.Nullify();
	}
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
