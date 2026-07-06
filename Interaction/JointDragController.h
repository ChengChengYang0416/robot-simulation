#pragma once

#include <functional>
#include <unordered_map>
#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <V3d_View.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace Interaction {

class JointDragController
{
// Phase 3.3 — click-a-link joint drag. Mirrors ManipulatorController's role in the
// Interaction layer: pure input handler + AIS-picking, kinematics-agnostic. On
// mouse-down over a driven robot link, resolves it to an axis index and projects
// the axis's world direction into screen space to obtain a tangent vector. Every
// mouse-move projects the pixel delta onto that tangent and reports a signed
// degree delta to the facade, which decides how to commit the joint. The facade
// injects the axis-lookup and axis-frame provider once per robot load so this
// class never needs to touch RobotKinematics directly (DIP, same reasoning that
// keeps ManipulatorController free of IK dependencies).
public:
	using JointDragHandler = std::function<void( int axisIdxZeroBased, double deltaDeg )>;
	// Fires on every mouse-move during an active drag. Argument is the 0-based axis
	// index (0..5, i.e. J1..J6) so the handler matches setJointAngle's signature.

	using AxisFrameProvider = std::function<bool( int axisIdxOneBased, gp_Pnt& origin, gp_Dir& dir )>;
	// Facade-supplied query: for axis 1..6, fill origin+dir with the joint's world-frame
	// pivot point and its rotation axis unit vector (Z of the parent DH frame per
	// standard DH convention). Returns false when the axis is out of range. Called once
	// per mouse-down; the axis is constant for the duration of a single-joint drag
	// (parent frames are frozen so the axis in world space does not change).

	void attach( const Handle( V3d_View ) & view,
				 const Handle( AIS_InteractiveContext ) & context );
	// Caches the view / context. Called once by the facade during initialize().

	void setEnabled( bool enabled );
	// Gate for mouse handling. When false, all onMouseDown/Move/Up calls short-circuit
	// return false so the camera interactor gets the events unchanged.

	[[nodiscard]] bool isEnabled() const
	{
		return m_enabled;
	}

	void setAxisLookup( std::unordered_map<AIS_InteractiveObject*, int> lookup );
	// Registers the AIS_Shape → axis-index (1..6) map that mouse-down consults to decide
	// whether a picked object is a draggable link. Passed by value so callers can
	// std::move into place; keys are raw pointers so no OCCT hash specialisation is
	// needed and the pointer stays valid as long as the SceneRepository retains the
	// handle (cleared by clearScene → setAxisLookup({})).

	void setAxisFrameProvider( AxisFrameProvider provider );
	// Registers the joint-frame query. See AxisFrameProvider docs for semantics.

	void setJointDragHandler( JointDragHandler handler );
	// Registers the per-mouse-move callback.

	[[nodiscard]] bool onMouseDown( int x, int y, int button );
	// Returns true only when left-click hit a driven link AND the axis-frame provider
	// resolved. Consumed events skip the camera interactor. Non-consumed clicks (empty
	// space, base, TCP trihedron) fall through to the camera as usual.

	[[nodiscard]] bool onMouseMove( int x, int y, int buttonMask );
	// Returns true while a drag is active; invokes the joint-drag handler with the
	// signed degree delta since the previous move. Returns false when idle so the
	// mouse interactor can still drive hover detection.

	[[nodiscard]] bool onMouseUp();
	// Returns true when an active drag was just terminated.

private:
	Handle( V3d_View )               m_view;
	Handle( AIS_InteractiveContext ) m_context;

	std::unordered_map<AIS_InteractiveObject*, int> m_axisLookup;
	AxisFrameProvider                m_axisFrameProvider;
	JointDragHandler                 m_handler;

	bool m_enabled    = false;
	bool m_isDragging = false;
	int  m_dragAxis   = -1;   // 0-based; -1 when idle.
	int  m_lastX      = 0;
	int  m_lastY      = 0;

	// Screen-space unit tangent vector for the active drag. Computed at mouse-down
	// by projecting a world-space +dθ displacement onto the screen so the sign is
	// correct by construction (no per-axis view-direction test needed).
	double m_tangentX = 0.0;
	double m_tangentY = 0.0;
};

}  // namespace Interaction
