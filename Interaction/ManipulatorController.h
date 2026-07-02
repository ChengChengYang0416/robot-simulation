#pragma once

#include <functional>
#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Manipulator.hxx>
#include <V3d_View.hxx>
#include <gp_Trsf.hxx>

namespace Interaction {

class ManipulatorController
{
// Owns the AIS_Manipulator 6-DoF drag gizmo used by Phase 3.1 (end-effector drag mode).
// Stays narrow: lifecycle + mouse-event-to-target-pose translation only. The IK solve
// and joint commit live in RobotSceneFacade so this class never depends on the
// kinematics layer. Mouse handlers return bool so the facade can route either to this
// controller (when drag is consuming the event) or to the camera MouseInteractor.
public:
	enum class GizmoMode : int
	{
		Translate = 0,   // three arrow handles only — position drag
		Rotate    = 1,   // three ring handles only — orientation drag
	};
	// Exclusive mode selector. Both parts on the AIS_Manipulator are visually hidden
	// except the one matching the current mode so hover-detection cannot silently
	// pick up the wrong handle after the operator changes intent.

	using TargetPoseHandler = std::function<void( const gp_Trsf& )>;
	// Invoked on every drag move with the manipulator's proposed world-frame target
	// transform. The handler typically runs IK and commits joints.

	void attach( const Handle( V3d_View ) & view,
				 const Handle( AIS_InteractiveContext ) & context );
	// Caches the OCCT view / context and constructs the AIS_Manipulator with default
	// part configuration (translation + rotation enabled, scaling disabled, activation
	// on detection). The gizmo is NOT displayed until setEnabled(true) is called.

	void setAnchorObject( const Handle( AIS_InteractiveObject ) & object );
	// Stores the AIS object the manipulator should attach to (typically the TCP
	// trihedron). Pass a null handle to detach and clear the anchor. Re-attaches
	// in-place if the controller is currently enabled.

	void setTargetPoseHandler( TargetPoseHandler handler );
	// Registers the per-frame drag callback. May be set / replaced at any time.

	void setEnabled( bool enabled );
	// Toggles gizmo visibility and selection modes. Enabling without an anchor is a
	// no-op; the gizmo only appears once both an anchor and enabled=true are present.

	[[nodiscard]] bool isEnabled() const
	{
		return m_enabled;
	}

	void setGizmoMode( GizmoMode mode );
	// Switches between Translate-only and Rotate-only handle sets. Safe to call
	// before attach()/setEnabled(): the choice is cached and applied on the next
	// enable. Mid-drag calls are ignored so an accidental HMI click during a drag
	// tick cannot leave the manipulator in an inconsistent selection state.

	[[nodiscard]] GizmoMode gizmoMode() const
	{
		return m_gizmoMode;
	}

	void syncToPose( const gp_Trsf& tcpFrame );
	// Re-anchors the gizmo on the supplied world-frame TCP pose. Called after FK
	// updates so the gizmo follows the actual TCP after IK convergence.

	[[nodiscard]] bool onMouseDown( int x, int y, int button );
	// Returns true when the event was consumed (drag started). The facade then
	// skips forwarding to the camera interactor.

	[[nodiscard]] bool onMouseMove( int x, int y, int buttonMask );
	// Returns true when actively dragging; invokes the target-pose handler with
	// the gizmo's proposed pose. Returns false when not dragging so the camera
	// interactor can drive hover / rotate / pan as usual.

	[[nodiscard]] bool onMouseUp();
	// Returns true when an active drag was just terminated.

private:
	void buildManipulator();
	// Lazily creates the AIS_Manipulator with the desired part configuration.
	// Called from attach() once the context is known.

	void applyGizmoModeParts();
	// Applies the current m_gizmoMode to the manipulator by toggling SetPart for
	// Translation / Rotation. Requires the manipulator to be constructed; the
	// context / display refresh is left to the caller.

	Handle( V3d_View )               m_view;
	Handle( AIS_InteractiveContext ) m_context;
	Handle( AIS_Manipulator )        m_manipulator;
	Handle( AIS_InteractiveObject )  m_anchor;
	TargetPoseHandler                m_handler;
	GizmoMode                        m_gizmoMode  = GizmoMode::Translate;
	bool                             m_enabled    = false;
	bool                             m_isDragging = false;
};

}  // namespace Interaction
