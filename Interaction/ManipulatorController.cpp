#include "ManipulatorController.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Manipulator.hxx>
#include <AIS_ManipulatorMode.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_XYZ.hxx>

namespace Interaction {

namespace {

inline constexpr Standard_ShortReal kGizmoSizeMm = 120.0f;
// Default gizmo size in mm. Chosen visually larger than the 80 mm TCP trihedron so
// the manipulator handles do not visually clash with the axis labels.

gp_Ax2 toAx2( const gp_Trsf& trsf )
// Decomposes a gp_Trsf into the gp_Ax2 the manipulator's SetPosition expects.
// gp_Ax2(origin, n, vx) takes the origin point, the local Z direction (n), and the
// local X direction (vx). gp_Mat::Column uses 1-based indices (col 1 = X axis,
// col 3 = Z axis) which matches OCCT's column-major convention for the rotation
// part of gp_Trsf.
{
	const gp_XYZ origin = trsf.TranslationPart();
	const gp_Mat rot    = trsf.HVectorialPart();
	const gp_Dir xDir( rot.Column( 1 ) );
	const gp_Dir zDir( rot.Column( 3 ) );
	return gp_Ax2( gp_Pnt( origin ), zDir, xDir );
}

}  // namespace

void ManipulatorController::buildManipulator()
// Creates the AIS_Manipulator once and configures parts based on the current gizmo
// mode. Scaling is disabled unconditionally (irrelevant for an articulated arm).
// Activation on detection is enabled so the operator does not need an extra click
// to switch between the visible handles — hovering over one is enough.
{
	m_manipulator = new AIS_Manipulator();
	m_manipulator->SetPart( AIS_MM_Scaling, Standard_False );
	m_manipulator->SetModeActivationOnDetection( Standard_True );
	m_manipulator->SetSize( kGizmoSizeMm );
	applyGizmoModeParts();
}

void ManipulatorController::applyGizmoModeParts()
// Shows exactly one handle set (arrows OR rings). We intentionally never render both
// together: with SetModeActivationOnDetection enabled, having both parts visible lets
// a stray hover flip the active mode between ticks, which was observed to swap a
// user's translation drag mid-motion when the cursor crossed a rotation ring.
{
	if( m_manipulator.IsNull() ) {
		return;
	}
	const bool isTranslate = ( m_gizmoMode == GizmoMode::Translate );
	m_manipulator->SetPart( AIS_MM_Translation, isTranslate ? Standard_True : Standard_False );
	m_manipulator->SetPart( AIS_MM_Rotation,    isTranslate ? Standard_False : Standard_True );
}

void ManipulatorController::attach( const Handle( V3d_View ) & view,
									const Handle( AIS_InteractiveContext ) & context )
{
	m_view    = view;
	m_context = context;
	if( m_manipulator.IsNull() ) {
		buildManipulator();
	}
}

void ManipulatorController::setAnchorObject( const Handle( AIS_InteractiveObject ) & object )
// Switching anchors mid-flight: detach from the old owner if needed before storing the
// new one. If the controller is currently enabled and the new anchor is non-null, re-
// attach immediately so the gizmo stays usable across robot reloads.
{
	if( !m_manipulator.IsNull() && m_manipulator->IsAttached() ) {
		m_manipulator->Detach();
	}
	m_anchor = object;
	if( m_enabled && !m_anchor.IsNull() && !m_manipulator.IsNull() && !m_context.IsNull() ) {
		AIS_Manipulator::OptionsForAttach opts;
		opts.SetAdjustPosition( Standard_True );   // required: leaving this false makes
		opts.SetAdjustSize( Standard_False );      // Attach() desync the anchor's rendered
		opts.SetEnableModes( Standard_True );      // pose from its LocalTransformation.
		m_manipulator->Attach( m_anchor, opts );
	}
}

void ManipulatorController::setTargetPoseHandler( TargetPoseHandler handler )
{
	m_handler = std::move( handler );
}

void ManipulatorController::setEnabled( bool enabled )
// Enable: attach to the cached anchor (no-op if anchor is null), display the gizmo,
// and activate translation + rotation selection modes so hover-detection picks them.
// Disable: detach, erase, and reset the drag flag. Erasing rather than only deactivating
// modes keeps the viewer clean when drag mode is toggled off.
{
	if( m_manipulator.IsNull() || m_context.IsNull() ) {
		m_enabled = enabled;
		return;
	}

	if( enabled ) {
		if( m_anchor.IsNull() ) {
			m_enabled = true;
			return;
		}
		if( !m_manipulator->IsAttached() ) {
			AIS_Manipulator::OptionsForAttach opts;
			opts.SetAdjustPosition( Standard_True );   // required: leaving this false makes
			opts.SetAdjustSize( Standard_False );      // Attach() desync the anchor's rendered
			opts.SetEnableModes( Standard_True );      // pose from its LocalTransformation.
			m_manipulator->Attach( m_anchor, opts );
		}
		applyGizmoModeParts();
		if( m_gizmoMode == GizmoMode::Translate ) {
			m_manipulator->EnableMode( AIS_MM_Translation );
		} else {
			m_manipulator->EnableMode( AIS_MM_Rotation );
		}
		m_context->Display( m_manipulator, Standard_False );
		m_context->UpdateCurrentViewer();
	} else {
		m_isDragging = false;
		if( m_manipulator->HasActiveTransformation() ) {
			m_manipulator->StopTransform( Standard_False );
		}
		if( m_manipulator->HasActiveMode() ) {
			m_manipulator->DeactivateCurrentMode();
		}
		if( m_manipulator->IsAttached() ) {
			m_manipulator->Detach();
		}
		m_context->Remove( m_manipulator, Standard_False );
		m_context->UpdateCurrentViewer();
	}
	m_enabled = enabled;
}

void ManipulatorController::setGizmoMode( GizmoMode mode )
// Applies the requested mode. Mid-drag calls are dropped rather than queued: the
// operator's finger is on the mouse button, so the state we would race is the
// manipulator's active-mode selection. If not currently attached / displayed, we
// only remember the choice — setEnabled() will pick it up on the next enable.
{
	if( m_isDragging ) {
		return;
	}
	if( m_gizmoMode == mode ) {
		return;
	}
	m_gizmoMode = mode;
	if( m_manipulator.IsNull() ) {
		return;
	}
	// Clear any hover-selected mode before flipping visible parts, otherwise
	// currentMode may still point at a mode whose handles are about to be hidden.
	if( m_manipulator->HasActiveMode() ) {
		m_manipulator->DeactivateCurrentMode();
	}
	applyGizmoModeParts();
	if( !m_enabled || m_context.IsNull() || !m_manipulator->IsAttached() ) {
		return;
	}
	if( m_gizmoMode == GizmoMode::Translate ) {
		m_manipulator->EnableMode( AIS_MM_Translation );
	} else {
		m_manipulator->EnableMode( AIS_MM_Rotation );
	}
	m_context->Redisplay( m_manipulator, Standard_False );
	m_context->UpdateCurrentViewer();
}

void ManipulatorController::syncToPose( const gp_Trsf& tcpFrame )
// Snaps the gizmo to the supplied world-frame pose. Called by the facade after IK
// converges (or after StopTransform) so the gizmo follows the actual TCP rather than
// the operator's cursor when the requested pose was unreachable.
{
	if( m_manipulator.IsNull() || !m_manipulator->IsAttached() ) {
		return;
	}
	m_manipulator->SetPosition( toAx2( tcpFrame ) );
}

bool ManipulatorController::onMouseDown( int x, int y, int button )
// Mouse-down handling: only consume the event when drag is enabled AND the cursor
// is currently over one of the manipulator's handles. We rely on the hover state
// that MouseInteractor::onMouseMove already refreshed on the last cursor motion —
// re-running MoveTo inside the mouse-down handler re-enters the picking pipeline
// during a Windows mouse event and was observed to corrupt the selection state
// enough to crash StartRotation on the fallback camera-rotate path.
//
// HasActiveMode() alone is not enough: ModeActivationOnDetection sets currentMode on
// hover but does NOT reliably clear it when the cursor leaves the handle. Without
// the DetectedInteractive() check the camera interactor would be starved whenever
// the user previously hovered the gizmo, even if the actual click is in empty space.
{
	constexpr int kLeftButton = 1048576;   // matches Interaction::MouseButton::Left
	if( !m_enabled || m_manipulator.IsNull() || m_view.IsNull() || m_context.IsNull() ) {
		return false;
	}
	if( button != kLeftButton ) {
		return false;
	}
	if( !m_context->HasDetected() ) {
		return false;
	}
	const Handle( AIS_InteractiveObject ) detected = m_context->DetectedInteractive();
	if( detected.IsNull() || detected != m_manipulator ) {
		return false;
	}
	if( !m_manipulator->HasActiveMode() ) {
		return false;
	}
	m_manipulator->StartTransform( x, y, m_view );
	m_isDragging = true;
	return true;
}

bool ManipulatorController::onMouseMove( int x, int y, int /*buttonMask*/ )
// Drag step: compute the proposed target via ObjectTransformation (read-only — does
// NOT mutate the anchor's LocalTransformation), then hand the world-frame target to
// the IK callback. The gizmo's visual position is then explicitly snapped to the
// cursor's target so the operator sees the handle follow the mouse, while the
// trihedron is left untouched here and is driven exclusively by FK inside the
// handler (via updateRobotTransforms → repo.setTcpTransform).
//
// Why not Transform(x, y, view)? That call writes (delta * StartTrsf) into the
// anchor's LocalTransformation as a side-effect. When the handler subsequently runs
// IK and fails (e.g. unreachable target / joint limit), it restores the seed joints
// and FK overwrites the trihedron back to the seed pose — but the manipulator's
// internal myPosition has already moved to the cursor's target. The user sees the
// gizmo lead and the trihedron lag for every failed tick. Decoupling the two paths
// (gizmo := cursor, trihedron := FK) makes the failure mode self-evident (the gizmo
// rubber-bands away from the TCP) and removes the double-write hazard entirely.
{
	if( !m_isDragging || m_manipulator.IsNull() || m_view.IsNull() ) {
		return false;
	}
	if( !m_manipulator->HasActiveMode() ) {
		return false;
	}
	gp_Trsf delta;
	if( !m_manipulator->ObjectTransformation( x, y, m_view, delta ) ) {
		return true;   // mode active, mouse hasn't moved off start position yet
	}
	const gp_Trsf target = delta * m_manipulator->StartTransformation();
	m_manipulator->SetPosition( toAx2( target ) );
	if( m_handler ) {
		m_handler( target );
	}
	return true;
}

bool ManipulatorController::onMouseUp()
// Drag end: clear the manipulator's started-transformation flag without reverting the
// anchor. We never called Transform() during the drag (FK was the sole writer of the
// anchor's LocalTransformation via the facade), so there is nothing to "commit" — but
// StopTransform(Standard_False) would revert the anchor's LocalTrsf back to the
// myStartTrsfs snapshot captured at StartTransform, which is the FK pose at drag-START
// (not the FK pose at drag-END). Passing Standard_True keeps the anchor at its current
// LocalTransformation (= the last FK_TCP that updateRobotTransforms wrote), which is
// what the next StartTransform needs to snapshot as the new reference. Without this,
// the next drag would treat the stale start pose as truth and IK would yank the TCP
// back to the previous drag's starting point.
{
	if( !m_isDragging ) {
		return false;
	}
	if( !m_manipulator.IsNull() && m_manipulator->HasActiveTransformation() ) {
		m_manipulator->StopTransform( Standard_True );
	}
	m_isDragging = false;
	return true;
}

}  // namespace Interaction
