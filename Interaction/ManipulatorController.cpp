#include "ManipulatorController.h"

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
// Creates the AIS_Manipulator once and configures parts: translation + rotation on
// all three axes, scaling disabled (irrelevant for an articulated arm). Activation
// on detection is enabled so the operator does not need an extra click to switch
// between manipulation modes — hovering over a handle is enough.
{
	m_manipulator = new AIS_Manipulator();
	m_manipulator->SetPart( AIS_MM_Scaling, Standard_False );
	m_manipulator->SetPart( AIS_MM_Translation, Standard_True );
	m_manipulator->SetPart( AIS_MM_Rotation, Standard_True );
	m_manipulator->SetModeActivationOnDetection( Standard_True );
	m_manipulator->SetSize( kGizmoSizeMm );
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
		opts.SetAdjustPosition( Standard_True );
		opts.SetAdjustSize( Standard_False );
		opts.SetEnableModes( Standard_True );
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
			opts.SetAdjustPosition( Standard_True );
			opts.SetAdjustSize( Standard_False );
			opts.SetEnableModes( Standard_True );
			m_manipulator->Attach( m_anchor, opts );
		}
		m_manipulator->EnableMode( AIS_MM_Translation );
		m_manipulator->EnableMode( AIS_MM_Rotation );
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
// Mouse-down handling: only consume the event when drag is enabled AND the manipulator
// already reports an active mode (hover-driven detection set it). Otherwise the camera
// interactor takes over via the facade's fallback path.
{
	constexpr int kLeftButton = 1048576;  // matches Interaction::MouseButton::Left
	if( !m_enabled || m_manipulator.IsNull() || m_view.IsNull() ) {
		return false;
	}
	if( button != kLeftButton ) {
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
// Drag step: apply the manipulator's Transform (which also writes the new pose into
// the anchor's LocalTransformation), then read that pose back as the IK target.
// Reading the anchor's LocalTransformation rather than computing the manipulator's
// Position-derived gp_Trsf keeps the dragged pose consistent with what the user sees
// on screen, even for plane-translation modes that combine axes.
{
	if( !m_isDragging || m_manipulator.IsNull() || m_view.IsNull() ) {
		return false;
	}
	if( !m_manipulator->HasActiveMode() ) {
		return false;
	}
	m_manipulator->Transform( x, y, m_view );
	if( !m_anchor.IsNull() && m_handler ) {
		m_handler( m_anchor->LocalTransformation() );
	}
	return true;
}

bool ManipulatorController::onMouseUp()
// Drag end: cancel rather than apply, because the facade has already committed the
// motion through FK each tick. StopTransform(false) leaves the anchor's transform
// untouched, and the next FK pass (driven by the post-IK joint commit) will keep
// the trihedron at its final location regardless.
{
	if( !m_isDragging ) {
		return false;
	}
	if( !m_manipulator.IsNull() && m_manipulator->HasActiveTransformation() ) {
		m_manipulator->StopTransform( Standard_False );
	}
	m_isDragging = false;
	return true;
}

}  // namespace Interaction
