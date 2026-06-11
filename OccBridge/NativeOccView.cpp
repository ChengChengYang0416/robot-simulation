#include <windows.h>
#include <cstdint>
#include <vector>
#include "NativeOccView.h"
#include "../Interaction/CameraController.h"
#include "../Interaction/MouseInteractor.h"
#include "../Kinematics/RobotKinematics.h"
#include "../Kinematics/RobotPartDef.h"
#include "../Kinematics/TcpPoseSolver.h"
#include "../Scene/SceneRepository.h"
#include "../Scene/StepLoader.h"
#include "../Viewer/ViewportContext.h"
#include <Quantity_Color.hxx>

struct NativeOccView::Impl
{
	Viewer::ViewportContext viewport;

	Scene::SceneRepository repo;
	Interaction::MouseInteractor mouse;
	Interaction::CameraController camera;
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
// Boots the OCCT rendering stack via ViewportContext, then wires every helper
// (scene repo, mouse, camera) to the resulting view / context handles.
{
	m_impl->viewport.initialize( hwnd );
	m_impl->repo.attach( m_impl->viewport.context() );
	m_impl->mouse.attach( m_impl->viewport.view(), m_impl->viewport.context() );
	m_impl->camera.attach( m_impl->viewport.view() );
}

void NativeOccView::resize( int width, int height )
// Forwards to ViewportContext; OCCT re-queries WNT_Window for the actual size.
{
	m_impl->viewport.resize( width, height );
}

void NativeOccView::redraw( void )
// Forwards to ViewportContext for a fast scene-only redraw.
{
	m_impl->viewport.redraw();
}

bool NativeOccView::loadStep( const wchar_t* filePath, bool append )
// Reads a STEP file via Scene::StepLoader and displays the merged root shape.
{
	if( m_impl->viewport.context().IsNull() ) {
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
	if( m_impl->viewport.context().IsNull() ) {
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
// Delegates to CameraController::fitAll (FitAll + ZFitAll + Redraw).
{
	m_impl->camera.fitAll();
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
// Delegates to CameraController::setViewIso (isometric projection + fitAll).
{
	m_impl->camera.setViewIso();
}

void NativeOccView::setViewTop( void )
// Delegates to CameraController::setViewTop (top-down projection + fitAll).
{
	m_impl->camera.setViewTop();
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
