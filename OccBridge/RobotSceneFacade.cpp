#include <windows.h>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "RobotSceneFacade.h"
#include "../Interaction/CameraController.h"
#include "../Interaction/JointDragController.h"
#include "../Interaction/ManipulatorController.h"
#include "../Interaction/MouseInteractor.h"
#include "../Kinematics/IkSolver.h"
#include "../Kinematics/AnalyticalIkSolver.h"
#include "../Kinematics/RobotKinematics.h"
#include "../Kinematics/RobotPartDef.h"
#include "../Kinematics/SingularityMonitor.h"
#include "../Kinematics/TcpPoseSolver.h"
#include "../Kinematics/TransformBuilder.h"
#include "../Scene/SceneRepository.h"
#include "../Scene/StepLoader.h"
#include "../Scene/TcpTrail.h"
#include "../Viewer/ViewportContext.h"
#include <AIS_Circle.hxx>
#include <Geom_Circle.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_Drawer.hxx>
#include <Quantity_Color.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>

namespace OccBridge {

struct RobotSceneFacade::Impl
{
	Viewer::ViewportContext viewport;

	Scene::SceneRepository repo;
	Scene::TcpTrail        trail;
	Interaction::MouseInteractor mouse;
	Interaction::ManipulatorController dragGizmo;
	Interaction::JointDragController   jointDrag;
	Interaction::CameraController camera;
	OccBridge::RobotKinematics kin;

	// Maps part index -> SceneRepository slot id; -1 when the STEP file failed to
	// load. Replaces the legacy convention of pushing null AIS_Shape placeholders
	// to keep shapes vector in lock-step with part definitions.
	std::vector<int> partToSlot;

	// Per-axis joint limits cached for the drag-mode IK. Defaults to a very loose
	// +/-360 deg window so that pre-load drag (no robot yet) does not constrain
	// the solver; HMI overrides these via setJointLimits() after each robot load.
	double jointMinDeg[ 6 ] = { -360.0, -360.0, -360.0, -360.0, -360.0, -360.0 };
	double jointMaxDeg[ 6 ] = {  360.0,  360.0,  360.0,  360.0,  360.0,  360.0 };
	bool   hasJointLimits   = false;

	// Drag state cached at the facade level so setGizmoMode() and setDragEnabled()
	// can independently update it and re-derive which controller (dragGizmo for TCP
	// Translate/Rotate, jointDrag for per-link Joint) is currently active. Without
	// this, the two setters would need to know each other's arguments to stay in
	// sync — a classic coupling smell.
	bool dragEnabled = false;
	int  gizmoMode   = static_cast<int>( IRobotScene::GizmoMode::Translate );

	// Cached AIS_Shape -> axis lookup built once per robot load in endRobotArm().
	// The joint-drag picker consumes the union of this and the ring lookup so the
	// operator can click either the link body or the visual affordance ring.
	std::unordered_map<AIS_InteractiveObject*, int> shapeAxisLookup;

	// One rotation ring per driven axis (index 0..5 → axes 1..6). The rings are the
	// visible affordance for Joint drag mode: without them the operator has no
	// on-screen cue that clicking a link will rotate it. Each ring is constructed
	// once at the world origin along +Z, then re-posed via SetLocalTransformation
	// on every FK update so it tracks its parent frame as the arm moves.
	std::vector<Handle( AIS_Circle )> jointRings;
	bool jointRingsVisible = false;

	// Base radius (mm) used for the outermost joint ring. Later rings taper linearly
	// so wrist rings do not visually swamp the fine geometry. Derived from the arm's
	// DH reach in rebuildJointRings; a sensible fallback is used pre-load.
	double jointRingBaseRadiusMm = 100.0;
};

RobotSceneFacade::RobotSceneFacade()
	: m_impl( new Impl() )
// Allocates the PIMPL object; all OCCT members are default-initialized here
{
}

RobotSceneFacade::~RobotSceneFacade()
// Releases the PIMPL object; OCCT Handles decrement their ref-count in Impl's destructor
{
	delete m_impl;
}

void RobotSceneFacade::initialize( HWND hwnd )
// Boots the OCCT rendering stack via ViewportContext, then wires every helper
// (scene repo, mouse, camera) to the resulting view / context handles.
{
	m_impl->viewport.initialize( hwnd );
	m_impl->repo.attach( m_impl->viewport.context() );
	m_impl->trail.attach( m_impl->viewport.context() );
	m_impl->mouse.attach( m_impl->viewport.view(), m_impl->viewport.context() );
	m_impl->dragGizmo.attach( m_impl->viewport.view(), m_impl->viewport.context() );
	m_impl->dragGizmo.setTargetPoseHandler(
		[ this ]( const gp_Trsf& target ) { this->applyDragTarget( target ); } );
	m_impl->jointDrag.attach( m_impl->viewport.view(), m_impl->viewport.context() );
	m_impl->jointDrag.setJointDragHandler(
		[ this ]( int axisIdx, double deltaDeg ) { this->applyJointDragDelta( axisIdx, deltaDeg ); } );
	// Joint-frame provider: delegate to axisFrameForIndex() so the ring-placement
	// pass reuses the exact same DH walk. Wrapping it in a lambda keeps the
	// controller decoupled from the facade (DIP): JointDragController only sees a
	// std::function returning (origin, axis), never touches OccBridge internals.
	m_impl->jointDrag.setAxisFrameProvider(
		[ this ]( int axisOneBased, gp_Pnt& origin, gp_Dir& dir ) -> bool {
			return this->axisFrameForIndex( axisOneBased, origin, dir );
		} );
	m_impl->camera.attach( m_impl->viewport.view() );
}

void RobotSceneFacade::resize( int width, int height )
// Forwards to ViewportContext; OCCT re-queries WNT_Window for the actual size.
{
	m_impl->viewport.resize( width, height );
}

void RobotSceneFacade::redraw()
// Forwards to ViewportContext for a fast scene-only redraw.
{
	m_impl->viewport.redraw();
}

bool RobotSceneFacade::loadStep( const wchar_t* filePath, bool append )
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

bool RobotSceneFacade::beginRobotArm( const RobotPartDef* parts, int partCount,
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

bool RobotSceneFacade::loadRobotPart( int index )
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

void RobotSceneFacade::endRobotArm()
// Asks SceneRepository to create the TCP trihedron, then runs the first transform pass
// and fits the camera. The trihedron's pose is set by updateRobotTransforms().
{
	if( !m_impl->kin.parts().empty() ) {
		m_impl->repo.ensureTcpTrihedron();
	}
	updateRobotTransforms();
	// Wire the drag gizmo to the freshly-created trihedron so toggling drag mode after
	// a load anchors the manipulator without an extra HMI call. The gizmo stays hidden
	// until setDragEnabled(true) is called.
	m_impl->dragGizmo.setAnchorObject( m_impl->repo.tcpTrihedron() );

	// Build the AIS_Shape -> axis-index (1..6) lookup consumed by the joint-drag
	// picker. Walk the axisToPartMap once, resolve each driven part to its slot's
	// AIS_Shape handle, and stash the raw pointer as the map key. Raw pointers are
	// safe here because SceneRepository owns the Handles for the lifetime of the
	// scene; clearScene() explicitly resets the lookup below before those handles
	// drop. Parts that failed to load (slot -1) or landed on a null AIS_Shape are
	// silently skipped so the picker just treats them as non-draggable.
	m_impl->shapeAxisLookup.clear();
	for( const auto& mapping : m_impl->kin.axisToPartMap() ) {
		const int axisIdx = mapping.first;    // 1..6
		const int partIdx = mapping.second;
		if( axisIdx < 1 || axisIdx > 6 ) {
			continue;
		}
		if( partIdx < 0 || partIdx >= static_cast<int>( m_impl->partToSlot.size() ) ) {
			continue;
		}
		const int slotId = m_impl->partToSlot[ partIdx ];
		Handle( AIS_Shape ) shape = m_impl->repo.slotShape( slotId );
		if( shape.IsNull() ) {
			continue;
		}
		m_impl->shapeAxisLookup.emplace( shape.get(), axisIdx );
	}

	// Build the visual affordance rings for Joint drag mode. They stay hidden
	// until syncDragControllers turns Joint mode on, but must exist by the time
	// we hand the merged lookup to the picker so ring pointers are already keys.
	rebuildJointRings();
	pushAxisLookupToJointDrag();

	fitAll();
}

void RobotSceneFacade::setJointAngle( int axisIndex, double angleDeg )
// Forwards the joint update to the kinematics solver, then re-applies transforms.
{
	m_impl->kin.setJointAngle( axisIndex, angleDeg );
	updateRobotTransforms();
}

void RobotSceneFacade::setJointAngles( const double anglesDeg[ 6 ] )
// Batch joint update: writes all six axes into the kinematics solver and then runs
// a single updateRobotTransforms() pass. The naive loop of setJointAngle() would
// trigger six full scene redraws per call, which is the dominant jitter source for
// MoveL / MoveJ tick handlers running at tens to hundreds of Hz.
{
	if( anglesDeg == nullptr ) {
		return;
	}
	for( int i = 0; i < 6; ++i ) {
		m_impl->kin.setJointAngle( i, anglesDeg[ i ] );
	}
	updateRobotTransforms();
}

void RobotSceneFacade::updateRobotTransforms()
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
		m_impl->trail.pushPose( *tcp );
		// Re-pose the drag gizmo whenever joints move through a non-drag path
		// (MoveJ / MoveL / setJointAngle / Reset-to-Home). AIS_Manipulator's
		// SetAdjustPosition only aligns on Attach() — it does NOT track later
		// SetLocalTransformation calls on the anchor trihedron. Without this
		// resync the gizmo remains at the last drag mouse-up position while
		// the arm animates elsewhere, and the operator sees the "end marker"
		// stranded away from the robot after Home.
		if( m_impl->dragGizmo.isEnabled() ) {
			m_impl->dragGizmo.syncToPose( *tcp );
		}
	}

	// Ring positions follow the joints they sit on (rings on child links move as
	// their parent axes rotate). Only re-pose while the rings are actually visible
	// — hidden rings do not need the trsf work.
	if( m_impl->jointRingsVisible ) {
		refreshJointRingPoses();
	}

	m_impl->repo.updateViewer();
}

void RobotSceneFacade::clearScene()
// Asks the repository to remove all displayed objects, then resets kinematics and
// the part-to-slot map. The repository keeps its context attachment for reuse.
{
	// Drop both drag controllers first so they do not retain stale handles to the
	// trihedron / link shapes the repository is about to release. setEnabled(false)
	// also detaches and erases the manipulator from the context; setAnchorObject
	// (nullptr) and setAxisLookup({}) clear the cached anchors so the next load can
	// rebind cleanly.
	m_impl->dragGizmo.setEnabled( false );
	m_impl->dragGizmo.setAnchorObject( Handle( AIS_InteractiveObject )() );
	m_impl->jointDrag.setEnabled( false );
	m_impl->jointDrag.setAxisLookup( {} );
	setJointRingsVisible( false );
	m_impl->jointRings.clear();
	m_impl->shapeAxisLookup.clear();
	m_impl->dragEnabled = false;
	m_impl->repo.clear();
	m_impl->trail.clear();
	m_impl->kin.configure( {}, {} );
	m_impl->partToSlot.clear();
	m_impl->repo.updateViewer();
	redraw();
}

void RobotSceneFacade::setTcpTrailMode( int mode )
// Translates the int wire value to Scene::TcpTrail::Mode. Unknown values are
// silently coerced to Off rather than asserted so a future managed enum tweak
// can't crash the player. The trail's setMode does its own no-op short-circuit
// when the value is unchanged, so the call is cheap to re-invoke from the menu.
{
	auto value = static_cast<Scene::TcpTrail::Mode>( mode );
	if( value != Scene::TcpTrail::Mode::Off
	 	&& value != Scene::TcpTrail::Mode::Polyline
	 	&& value != Scene::TcpTrail::Mode::PolylineWithFrames ) {
		value = Scene::TcpTrail::Mode::Off;
	}
	m_impl->trail.setMode( value );
}

void RobotSceneFacade::clearTcpTrail()
// Operator-driven "erase what's on screen, keep recording from now on". Distinct
// from setTcpTrailMode(Off) which also stops sampling.
{
	m_impl->trail.clear();
}

void RobotSceneFacade::fitAll()
// Delegates to CameraController::fitAll (FitAll + ZFitAll + Redraw).
{
	m_impl->camera.fitAll();
}

bool RobotSceneFacade::getTcpPose( double out[ 6 ] ) const
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

bool RobotSceneFacade::getJointAngles( double outAnglesDeg[ 6 ] ) const
// Copies the kinematics core's current joint vector out. Needed by the HMI to
// reconcile its local cache after paths that mutate joints without going through
// setJointAngle(s) — currently the drag-mode gizmo, which commits IK results
// directly from applyDragTarget() and leaves the HMI copy stale.
{
	if( outAnglesDeg == nullptr ) {
		return false;
	}
	if( m_impl->kin.parts().empty() ) {
		return false;
	}
	const auto& joints = m_impl->kin.jointAnglesDeg();
	for( int i = 0; i < 6; ++i ) {
		outAnglesDeg[ i ] = joints[ i ];
	}
	return true;
}

int RobotSceneFacade::solveTcpIk( const double targetXyzRpy[ 6 ],
								  const double jointMinDeg[ 6 ],
								  const double jointMaxDeg[ 6 ],
								  double       outAnglesDeg[ 6 ] )
// Runs DLS IK using the kinematics solver's current joint angles as seed and writes
// the final angles into outAnglesDeg. The scene's joint state is restored before
// returning so the caller can decide whether to commit the solution.
{
	if( targetXyzRpy == nullptr || outAnglesDeg == nullptr ) {
		return static_cast<int>( IkSolveStatus::InvalidConfig );
	}
	if( m_impl->kin.parts().empty() || m_impl->kin.axisToPartMap().empty() ) {
		return static_cast<int>( IkSolveStatus::NoRobot );
	}

	// Snapshot the current joint angles so the scene can be reverted regardless of
	// whether the IK iteration converges. solveIkDls mutates kin in place.
	const auto seedDeg = m_impl->kin.jointAnglesDeg();

	// Build the target gp_Trsf from XYZ + ZYX intrinsic Euler (same convention as
	// solveTcpPose / makeOffset).
	const gp_Trsf target = Transform::makeOffset(
		targetXyzRpy[ 0 ], targetXyzRpy[ 1 ], targetXyzRpy[ 2 ],
		targetXyzRpy[ 3 ], targetXyzRpy[ 4 ], targetXyzRpy[ 5 ] );

	IkOptions opts;
	opts.useJointLimits = ( jointMinDeg != nullptr && jointMaxDeg != nullptr );
	if( opts.useJointLimits ) {
		for( int i = 0; i < 6; ++i ) {
			opts.jointMinDeg[ i ] = jointMinDeg[ i ];
			opts.jointMaxDeg[ i ] = jointMaxDeg[ i ];
		}
	}

	// Try the closed-form Pieper solver first (O(1), exact). It rejects geometries
	// without a spherical wrist by returning InvalidConfiguration; in that case we
	// fall back to the iterative DLS solver so the facade keeps working for any
	// 6-DOF arm that might be loaded later. solveIkAnalytical does not mutate kin,
	// so no snapshot/restore is needed for the fast path.
	IkResult res = solveIkAnalytical( m_impl->kin, target, seedDeg, opts );
	if( res.status != IkStatus::Converged ) {
		res = solveIkDls( m_impl->kin, target, seedDeg, opts );
	}

	for( int i = 0; i < 6; ++i ) {
		outAnglesDeg[ i ] = res.jointAnglesDeg[ i ];
	}

	// Restore the scene's joint state; caller commits via setJointAngle() on success.
	for( int i = 0; i < 6; ++i ) {
		m_impl->kin.setJointAngle( i, seedDeg[ i ] );
	}
	updateRobotTransforms();

	switch( res.status ) {
	case IkStatus::Converged:
		return static_cast<int>( IkSolveStatus::Converged );
	case IkStatus::MaxIterations:
		return static_cast<int>( IkSolveStatus::NotConverged );
	case IkStatus::InvalidConfiguration:
	default:
		return static_cast<int>( IkSolveStatus::InvalidConfig );
	}
}

bool RobotSceneFacade::getManipulability( double outMetrics[ 5 ],
										  int*   outKind,
										  int*   outLevel ) const
// Evaluates the singularity monitor at the current joint state. SingularityMonitor::
// evaluate() calls computeCumulative() (idempotent for the same q) so the scene's
// joint angles are not perturbed and there is no need to snapshot/restore.
{
	if( outMetrics == nullptr ) {
		return false;
	}
	if( m_impl->kin.parts().empty() || m_impl->kin.axisToPartMap().empty() ) {
		return false;
	}

	const SingularityReport rpt = OccBridge::evaluate( m_impl->kin );
	if( !rpt.valid ) {
		return false;
	}

	outMetrics[ 0 ] = rpt.manipulability;
	outMetrics[ 1 ] = rpt.wristManipulability;
	outMetrics[ 2 ] = rpt.armManipulability;
	outMetrics[ 3 ] = rpt.wristRatio;
	outMetrics[ 4 ] = rpt.armRatio;
	if( outKind != nullptr ) {
		*outKind = static_cast<int>( rpt.kind );
	}
	if( outLevel != nullptr ) {
		*outLevel = static_cast<int>( rpt.level );
	}
	return true;
}

void RobotSceneFacade::setViewIso()
// Delegates to CameraController::setViewIso (isometric projection + fitAll).
{
	m_impl->camera.setViewIso();
}

void RobotSceneFacade::setViewTop()
// Delegates to CameraController::setViewTop (top-down projection + fitAll).
{
	m_impl->camera.setViewTop();
}

void RobotSceneFacade::onMouseDown( int x, int y, int button )
// Drag controllers get first refusal in priority order: joint picker first (when
// Joint mode is armed it needs to consume any left-click over a driven link),
// then the TCP gizmo (consumes only when hovering a manipulator handle), then the
// camera. Only one controller can be enabled at a time (see syncDragControllers),
// so priority ordering here is defensive — but keeping it explicit avoids relying
// on the invariant if a future mode expands the drag surface.
{
	if( m_impl->jointDrag.onMouseDown( x, y, button ) ) {
		return;
	}
	if( m_impl->dragGizmo.onMouseDown( x, y, button ) ) {
		return;
	}
	m_impl->mouse.onMouseDown( x, y, button );
}

void RobotSceneFacade::onMouseMove( int x, int y, int buttonMask )
// During an active drag the corresponding controller callback runs IK (TCP gizmo)
// or applies the delta joint angle (joint picker) itself, so the mouse interactor
// is skipped to avoid double-handling. With no active drag both controllers return
// false and hover-driven detection still happens inside the mouse interactor's
// MoveTo() call.
{
	if( m_impl->jointDrag.onMouseMove( x, y, buttonMask ) ) {
		return;
	}
	if( m_impl->dragGizmo.onMouseMove( x, y, buttonMask ) ) {
		return;
	}
	m_impl->mouse.onMouseMove( x, y, buttonMask );
}

void RobotSceneFacade::onMouseUp()
// End-of-drag tear-down: re-pin the trihedron to the current FK_TCP and snap the
// gizmo to the same pose. Re-pinning is defensive: AIS_Manipulator::StopTransform
// has historically been observed to mutate the anchor's LocalTransformation as part
// of its tear-down, so we force-restore the trihedron to FK truth before the next
// StartTransform snapshots it as the new drag reference. Without this, a stale
// LocalTrsf gets captured at the next mouse-down and the IK target on the first
// drag-tick computes against the wrong base, yanking the TCP back to the previous
// drag's starting point. Joint-drag mouse-up needs no such fix-up: it commits joint
// angles through the FK path (setJointAngle + updateRobotTransforms) which is the
// same code the trihedron pose reads from, so there is no dual-write to reconcile.
{
	if( m_impl->jointDrag.onMouseUp() ) {
		// Joint drag terminated cleanly; nothing further to do because every mouse-
		// move already committed the intermediate joint state.
	} else if( m_impl->dragGizmo.onMouseUp() ) {
		if( auto tcp = m_impl->repo.tcpFrame() ) {
			m_impl->repo.setTcpTransform( *tcp );
			m_impl->dragGizmo.syncToPose( *tcp );
			m_impl->repo.updateViewer();
		}
	}
	m_impl->mouse.onMouseUp();
}

void RobotSceneFacade::onMouseWheel( int delta )
// Forwards to MouseInteractor::onMouseWheel for zoom-in / zoom-out.
{
	m_impl->mouse.onMouseWheel( delta );
}

bool RobotSceneFacade::saveScreenshot( const wchar_t* filePath )
// Forwards to ViewportContext, which owns the V3d_View and the framebuffer.
{
	return m_impl->viewport.saveScreenshot( filePath );
}

void RobotSceneFacade::setJointLimits( const double jointMinDeg[ 6 ],
									   const double jointMaxDeg[ 6 ] )
// Caches per-axis limits for the drag-mode IK. nullptr on either side clears the
// cache so the drag path falls back to unbounded IK. HMI calls this once per robot
// load; downstream solvers read the cached arrays inside applyDragTarget().
{
	if( jointMinDeg == nullptr || jointMaxDeg == nullptr ) {
		m_impl->hasJointLimits = false;
		return;
	}
	for( int i = 0; i < 6; ++i ) {
		m_impl->jointMinDeg[ i ] = jointMinDeg[ i ];
		m_impl->jointMaxDeg[ i ] = jointMaxDeg[ i ];
	}
	m_impl->hasJointLimits = true;
}

void RobotSceneFacade::setDragEnabled( bool enabled )
// Umbrella toggle for both drag paths. Which controller actually turns on is decided
// by the currently-selected gizmo mode: Translate/Rotate → AIS_Manipulator (TCP
// drag), Joint → per-link picker. Delegates to syncDragControllers() so the enable/
// disable + mode dispatch logic lives in exactly one place.
{
	m_impl->dragEnabled = enabled;
	syncDragControllers();
}

void RobotSceneFacade::setGizmoMode( int mode )
// Stores the new mode and re-drives the controllers. Also pushes the AIS_Manipulator
// handle-visibility mode when applicable so a mid-session Translate ↔ Rotate switch
// keeps the visible part in sync. Joint mode is a facade-level dispatch and never
// reaches the manipulator's SetPart; passing it through would put the manipulator
// into an inconsistent Translate-with-Rotation-hidden state on the next enable.
{
	m_impl->gizmoMode = mode;
	if( mode == static_cast<int>( GizmoMode::Rotate ) ) {
		m_impl->dragGizmo.setGizmoMode( Interaction::ManipulatorController::GizmoMode::Rotate );
	} else if( mode == static_cast<int>( GizmoMode::Translate ) ) {
		m_impl->dragGizmo.setGizmoMode( Interaction::ManipulatorController::GizmoMode::Translate );
	}
	syncDragControllers();
	m_impl->repo.updateViewer();
}

void RobotSceneFacade::syncDragControllers()
// Single source of truth for (dragEnabled, gizmoMode) → controller enable state.
// Any state transition that could change which controller should be active — the
// operator flipping the umbrella toggle, or switching Drag Target radios mid-
// session — routes through here so we never end up with both controllers enabled
// (and fighting for mouse events) or both disabled when the operator expected drag
// to be on.
{
	const bool jointMode = ( m_impl->gizmoMode == static_cast<int>( GizmoMode::Joint ) );
	const bool wantGizmo = m_impl->dragEnabled && !jointMode;
	const bool wantJoint = m_impl->dragEnabled &&  jointMode;

	m_impl->dragGizmo.setEnabled( wantGizmo );
	m_impl->jointDrag.setEnabled( wantJoint );
	setJointRingsVisible( wantJoint );

	if( wantJoint ) {
		// Rings must reflect the current joint state the first time they appear,
		// not the pose captured when they were last built.
		refreshJointRingPoses();
		m_impl->repo.updateViewer();
	}

	if( wantGizmo ) {
		// Snap the manipulator to the current TCP so the first interaction starts
		// from the real pose rather than the manipulator's default origin. Mirrors
		// the historical setDragEnabled(true) behaviour.
		if( auto tcp = m_impl->repo.tcpFrame() ) {
			m_impl->dragGizmo.syncToPose( *tcp );
			m_impl->repo.updateViewer();
		}
	}
}

void RobotSceneFacade::applyDragTarget( const gp_Trsf& targetWorld )
// Drag callback: convert the manipulator's proposed pose to (XYZ, ZYX-RPY), run the
// analytical IK first and fall back to DLS, commit joints on convergence. Failure
// paths are silent here — the gizmo stays where the user dragged it so the operator
// sees that the target was unreachable, and the next drag tick will try again.
//
// Per-axis limits are pulled from the cached arrays so the per-tick callback does
// not need to take them as arguments (and so HMI does not have to push them every
// frame). The seed is always the current joint vector, so the analytical solver's
// 8-branch selector picks the configuration closest to the operator's current pose.
{
	if( m_impl->kin.parts().empty() || m_impl->kin.axisToPartMap().empty() ) {
		return;
	}

	const auto seedDeg = m_impl->kin.jointAnglesDeg();

	IkOptions opts;
	opts.useJointLimits = m_impl->hasJointLimits;
	if( opts.useJointLimits ) {
		for( int i = 0; i < 6; ++i ) {
			opts.jointMinDeg[ i ] = m_impl->jointMinDeg[ i ];
			opts.jointMaxDeg[ i ] = m_impl->jointMaxDeg[ i ];
		}
	}

	IkResult res = solveIkAnalytical( m_impl->kin, targetWorld, seedDeg, opts );
	if( res.status != IkStatus::Converged ) {
		res = solveIkDls( m_impl->kin, targetWorld, seedDeg, opts );
	}
	if( res.status != IkStatus::Converged ) {
		// Restore the seed so a failed DLS run does not leave the kinematics in a
		// half-stepped state (DLS mutates kin in place). The trihedron is repainted
		// from kin via updateRobotTransforms on the next successful step.
		for( int i = 0; i < 6; ++i ) {
			m_impl->kin.setJointAngle( i, seedDeg[ i ] );
		}
		updateRobotTransforms();
		return;
	}

	for( int i = 0; i < 6; ++i ) {
		m_impl->kin.setJointAngle( i, res.jointAnglesDeg[ i ] );
	}
	updateRobotTransforms();
}

void RobotSceneFacade::applyJointDragDelta( int axisIdxZeroBased, double deltaDeg )
// Joint-drag callback (Phase 3.3): apply an incremental angle to a single axis and
// refresh the scene. The controller already projected the mouse motion onto the
// axis's screen tangent, so deltaDeg is signed and correctly scaled — this method
// only handles clamping to the cached joint limits and committing through the
// standard FK path (kin.setJointAngle + updateRobotTransforms). Reusing that path
// means the trihedron / TCP-trail / dashboard consumers pick up the change through
// the same channels they see for MoveJ / MoveL, no separate observer needed.
{
	if( axisIdxZeroBased < 0 || axisIdxZeroBased >= 6 ) {
		return;
	}
	if( m_impl->kin.parts().empty() ) {
		return;
	}

	const auto& current = m_impl->kin.jointAnglesDeg();
	double newAngle = current[ axisIdxZeroBased ] + deltaDeg;
	if( m_impl->hasJointLimits ) {
		if( newAngle < m_impl->jointMinDeg[ axisIdxZeroBased ] ) {
			newAngle = m_impl->jointMinDeg[ axisIdxZeroBased ];
		} else if( newAngle > m_impl->jointMaxDeg[ axisIdxZeroBased ] ) {
			newAngle = m_impl->jointMaxDeg[ axisIdxZeroBased ];
		}
	}
	m_impl->kin.setJointAngle( axisIdxZeroBased, newAngle );
	updateRobotTransforms();
}

bool RobotSceneFacade::axisFrameForIndex( int axisOneBased, gp_Pnt& origin, gp_Dir& dir ) const
// Per standard DH the joint that drives part i rotates around the +Z axis of
// part i's parent frame, with the pivot at the parent frame's origin. Parent -1
// (root joint) collapses to the world frame so J1 drags around world +Z.
// Extracted so both the joint-drag picker's frame callback AND rebuildJointRings
// share one authoritative walk — otherwise the two would drift apart the moment
// the DH convention changed.
{
	if( axisOneBased < 1 || axisOneBased > 6 ) {
		return false;
	}
	int partIdx = -1;
	for( const auto& mapping : m_impl->kin.axisToPartMap() ) {
		if( mapping.first == axisOneBased ) {
			partIdx = mapping.second;
			break;
		}
	}
	if( partIdx < 0 ) {
		return false;
	}
	const auto& parts = m_impl->kin.parts();
	if( partIdx >= static_cast<int>( parts.size() ) ) {
		return false;
	}
	const int parent = parts[ partIdx ].parentIdx;
	const auto& cumulative = m_impl->kin.computeCumulative();
	gp_Trsf parentFrame;
	if( parent >= 0 && parent < static_cast<int>( cumulative.size() ) ) {
		parentFrame = cumulative[ parent ];
	}
	origin = gp_Pnt( parentFrame.TranslationPart() );
	// Extract Z-axis by column access — HVectorialPart returns a temporary gp_Mat
	// by value so non-const methods like Multiply() cannot be called on it.
	const gp_Mat rot = parentFrame.HVectorialPart();
	dir = gp_Dir( rot.Column( 3 ) );
	return true;
}

void RobotSceneFacade::rebuildJointRings()
// Constructs one AIS_Circle per driven axis at the world origin along +Z. The
// per-frame refresh in refreshJointRingPoses() moves each ring onto its joint via
// SetLocalTransformation, which is far cheaper than reconstructing a Geom_Circle
// every tick. Radii taper from the base out to the wrist so proximal rings do not
// visually swallow the distal geometry. All rings are erased from the context
// first because the operator may reload a robot without clearing scene state.
{
	auto ctx = m_impl->viewport.context();
	if( ctx.IsNull() ) {
		return;
	}

	// Tear down any previous rings so a robot reload does not double-populate the
	// context (which would also leave stale raw pointers in axisLookup).
	for( auto& ring : m_impl->jointRings ) {
		if( !ring.IsNull() ) {
			ctx->Remove( ring, Standard_False );
		}
	}
	m_impl->jointRings.clear();
	m_impl->jointRingsVisible = false;

	const int axisCount = static_cast<int>( m_impl->kin.axisToPartMap().size() );
	if( axisCount == 0 ) {
		return;
	}

	// Derive a base radius from the arm's DH reach so LA906-class arms get larger
	// rings than LA580-class ones. Falls back to a sensible default when the DH
	// numbers are missing.
	double reach = 0.0;
	for( const auto& p : m_impl->kin.parts() ) {
		reach += std::abs( p.dhA ) + std::abs( p.dhD );
	}
	m_impl->jointRingBaseRadiusMm = ( reach > 0.0 ) ? ( reach * 0.10 ) : 100.0;

	// Bright cyan is chosen to contrast against both the dark gray background and
	// the typical STEP-file color palette (grays / oranges).
	const Quantity_Color ringColor( 0.15, 0.90, 1.00, Quantity_TOC_RGB );

	m_impl->jointRings.reserve( 6 );
	for( int axis = 1; axis <= 6; ++axis ) {
		// Radius tapers linearly from base (axis 1) down to ~40% at the wrist so
		// rings on smaller distal links do not dwarf the geometry there.
		const double taper  = 1.0 - 0.6 * ( static_cast<double>( axis - 1 ) / 5.0 );
		const double radius = m_impl->jointRingBaseRadiusMm * taper;

		Handle( Geom_Circle ) geomCircle = new Geom_Circle(
			gp_Ax2( gp::Origin(), gp::DZ() ), radius );
		Handle( AIS_Circle ) ring = new AIS_Circle( geomCircle );
		ring->SetColor( ringColor );
		ring->SetWidth( 3.0 );

		m_impl->jointRings.push_back( ring );
	}
}

void RobotSceneFacade::refreshJointRingPoses()
// Re-computes each ring's world placement from the current FK state and applies
// it via SetLocalTransformation. Rings on distal links move whenever any upstream
// joint rotates, so this needs to run on every FK tick while joint-mode is on.
// gp_Trsf::SetDisplacement(fromSystem, toSystem) rigidly moves a shape whose
// coordinates are expressed in fromSystem so that after the transform they have
// the same coordinates in toSystem — for a Geom_Circle built at (origin, +Z),
// that maps it to (jointOrigin, jointAxis).
{
	auto ctx = m_impl->viewport.context();
	if( ctx.IsNull() ) {
		return;
	}
	if( m_impl->jointRings.empty() ) {
		return;
	}

	const gp_Ax3 srcFrame( gp::Origin(), gp::DZ() );
	for( int axis = 1; axis <= 6; ++axis ) {
		const size_t idx = static_cast<size_t>( axis - 1 );
		if( idx >= m_impl->jointRings.size() ) {
			break;
		}
		Handle( AIS_Circle ) ring = m_impl->jointRings[ idx ];
		if( ring.IsNull() ) {
			continue;
		}

		gp_Pnt origin;
		gp_Dir dir;
		if( !axisFrameForIndex( axis, origin, dir ) ) {
			continue;
		}
		const gp_Ax3 dstFrame( origin, dir );
		gp_Trsf trsf;
		trsf.SetDisplacement( srcFrame, dstFrame );
		ring->SetLocalTransformation( trsf );

		if( m_impl->jointRingsVisible ) {
			ctx->Redisplay( ring, Standard_False );
		}
	}
}

void RobotSceneFacade::setJointRingsVisible( bool visible )
// Batched Display / Erase so the operator toggling Joint mode gets one redraw,
// not six. Also drives whether refreshJointRingPoses runs during FK updates,
// avoiding pointless transform work while the rings are hidden.
{
	auto ctx = m_impl->viewport.context();
	if( ctx.IsNull() ) {
		return;
	}
	if( m_impl->jointRingsVisible == visible ) {
		return;
	}
	m_impl->jointRingsVisible = visible;
	for( auto& ring : m_impl->jointRings ) {
		if( ring.IsNull() ) {
			continue;
		}
		if( visible ) {
			ctx->Display( ring, Standard_False );
			// AIS_Circle is a datum object; whole-object selection (mode 0) is
			// what the joint-drag picker needs to receive DetectedInteractive
			// results. Auto-activation is normally on but explicit is safer for
			// datum types where the default mode set can vary between OCCT
			// releases.
			ctx->Activate( ring, 0, Standard_False );
		} else {
			ctx->Erase( ring, Standard_False );
		}
	}
}

void RobotSceneFacade::pushAxisLookupToJointDrag()
// Union of the two axis-lookup halves (link shapes + rings) handed to the joint-
// drag picker as one flat map. Keeping the halves separate on the facade side lets
// each be rebuilt independently: shapes change only on robot load, rings change
// on load AND could in future be rebuilt on radius / color tuning without
// touching the shape half.
{
	std::unordered_map<AIS_InteractiveObject*, int> merged = m_impl->shapeAxisLookup;
	for( size_t i = 0; i < m_impl->jointRings.size(); ++i ) {
		if( m_impl->jointRings[ i ].IsNull() ) {
			continue;
		}
		merged.emplace( m_impl->jointRings[ i ].get(), static_cast<int>( i + 1 ) );
	}
	m_impl->jointDrag.setAxisLookup( std::move( merged ) );
}

IRobotScene* createRobotScene()
// Factory entry point declared in IRobotScene.h. Clients (OccViewerControl,
// future test harnesses) never name RobotSceneFacade directly, which lets us
// swap the implementation without touching the managed wrapper.
{
	return new RobotSceneFacade();
}

}  // namespace OccBridge
