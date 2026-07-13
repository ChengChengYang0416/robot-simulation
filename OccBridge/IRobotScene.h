#pragma once

struct HWND__;
using HWND = HWND__*;
struct RobotPartDef;

namespace OccBridge {

class IRobotScene
{
// Abstract facade that decouples OccViewerControl (managed wrapper) from any concrete
// native implementation. Methods mirror the NativeOccView public surface as it stands
// after R0..R5; subclasses compose the small libraries (Viewer / Scene / Interaction /
// Kinematics) however they see fit. No exceptions are thrown; failures are reported
// via bool / nullptr return values to match the embedded no-exception policy.
public:
	virtual ~IRobotScene() = default;
	// Polymorphic deletion contract; concrete implementations release their owned
	// resources (OCCT handles, kinematics buffers, etc.) in their own destructors.

	// ---- Lifecycle / surface management ----------------------------------------

	virtual void initialize( HWND hwnd ) = 0;
	// Builds the underlying 3D rendering stack and binds it to the host Win32 window.

	virtual void resize( int width, int height ) = 0;
	// Notifies the implementation that the host surface size changed.

	virtual void redraw() = 0;
	// Forces an immediate scene redraw without changing scene structure.

	// ---- Geometry loading ------------------------------------------------------

	[[nodiscard]] virtual bool loadStep( const wchar_t* filePath, bool append ) = 0;
	// Loads a standalone STEP file; clears the scene first when append is false.

	[[nodiscard]] virtual bool beginRobotArm( const RobotPartDef* parts, int partCount,
											  const int* axisToPartMap, int mapCount ) = 0;
	// Clears the scene and configures the kinematics with the given part definitions
	// and axis-to-part mapping; subsequent loadRobotPart / endRobotArm calls populate
	// geometry and finalize the chain.

	[[nodiscard]] virtual bool loadRobotPart( int index ) = 0;
	// Loads geometry for the part at the given index registered by beginRobotArm.

	virtual void endRobotArm() = 0;
	// Finalizes the robot arm: applies the initial transform pass and fits the camera.

	virtual void clearScene() = 0;
	// Removes all displayed objects and resets kinematics state.

	// ---- Kinematics driver -----------------------------------------------------

	virtual void setJointAngle( int axisIndex, double angleDeg ) = 0;
	// Updates the given 0-based axis angle (degrees) and refreshes the scene.

	virtual void setJointAngles( const double anglesDeg[ 6 ] ) = 0;
	// Batch variant of setJointAngle: applies all six axes and redraws the scene
	// exactly once. Required by MoveL / MoveJ tick handlers to avoid the 6x full
	// scene redraw cost of looping setJointAngle() per axis at high frame rates.

	[[nodiscard]] virtual bool getJointAngles( double outAnglesDeg[ 6 ] ) const = 0;
	// Reads the current 6-axis joint vector (degrees) from the kinematics core.
	// Returns false when no robot is loaded or out is null. Callers can use this
	// to reconcile a cached HMI copy after paths that mutate joints outside the
	// setJointAngle(s) surface — currently the drag-mode gizmo (Phase 3.1/3.2)
	// which commits IK solutions directly from applyDragTarget().

	[[nodiscard]] virtual bool getTcpPose( double out[6] ) const = 0;
	// Fills out with [x, y, z, rx, ry, rz] in mm and degrees (ZYX intrinsic Euler);
	// returns false when no robot is loaded or out is null.

	enum class IkSolveStatus : int
	{
		Converged       = 0,
		NoRobot         = 1,
		NotConverged    = 2,
		InvalidConfig   = 3,
	};

	[[nodiscard]] virtual int solveTcpIk( const double targetXyzRpy[6],
										  const double jointMinDeg[6],
										  const double jointMaxDeg[6],
										  double       outAnglesDeg[6] ) = 0;
	// Runs DLS IK for the requested TCP pose using the current joint angles as seed.
	// Inputs:
	//   targetXyzRpy  - [x, y, z, rx, ry, rz] in mm and degrees (ZYX intrinsic Euler,
	//                   same convention as getTcpPose)
	//   jointMinDeg   - per-axis lower limits in degrees (size 6)
	//   jointMaxDeg   - per-axis upper limits in degrees (size 6)
	//   outAnglesDeg  - filled with the solver's final joint angles (size 6)
	// Returns IkSolveStatus cast to int. The scene's joint state is left unchanged
	// regardless of outcome; the caller is responsible for committing the solution
	// via setJointAngle() when it accepts the result.

	[[nodiscard]] virtual bool getManipulability( double outMetrics[ 5 ],
												  int*   outKind,
												  int*   outLevel ) const = 0;
	// Computes Jacobian-based singularity metrics for the current joint state.
	// outMetrics layout (mm^3 / rad^6 for the full det, dimensionless for ratios):
	//   [0] manipulability        = |det(J)|
	//   [1] wristManipulability   = |det(J[3:6, 3:6])|
	//   [2] armManipulability     = |det(J[0:3, 0:3])|
	//   [3] wristRatio            = wristManipulability                  (dimensionless)
	//   [4] armRatio              = armManipulability / L_ref^3          (dimensionless)
	// outKind  = SingularityKind  (0 None, 1 Wrist, 2 Elbow, 3 Shoulder, 4 Combined)
	// outLevel = SingularityLevel (0 Normal, 1 Warning, 2 Critical)
	// Returns false when no robot is loaded or outMetrics is null. The scene's joint
	// state is not modified.

	virtual void setJointLimits( const double jointMinDeg[ 6 ],
								 const double jointMaxDeg[ 6 ] ) = 0;
	// Caches per-axis joint limits used by internally-driven IK paths (currently the
	// drag-mode gizmo). Pass nullptr for either array to revert to unbounded IK. HMI
	// callers that pass limits per call via solveTcpIk() do not need to invoke this;
	// it exists so the facade can run IK from the mouse-move handler without taking
	// the joint-limit arrays as parameters on every drag tick.

	// ---- Drag mode (Phase 3.1) -------------------------------------------------

	virtual void setDragEnabled( bool enabled ) = 0;
	// Toggles the AIS_Manipulator 6-DoF drag gizmo. When enabled, the gizmo is
	// attached to the TCP trihedron and mouse drags on its handles run IK against
	// the requested pose; on convergence the joint angles are committed and the
	// gizmo re-syncs to the new TCP frame. Enabling without a loaded robot leaves
	// the gizmo hidden until endRobotArm() supplies an anchor.

	enum class GizmoMode : int
	{
		Translate = 0,   // three arrow handles → TCP position drag
		Rotate    = 1,   // three ring handles  → TCP orientation drag (Phase 3.2)
		Joint     = 2,   // click a link → drag rotates its driving axis (Phase 3.3)
	};

	virtual void setGizmoMode( int mode ) = 0;
	// Switches the drag interaction between three targets: Translate/Rotate operate
	// on the TCP via the AIS_Manipulator gizmo; Joint hides the gizmo and instead
	// picks a robot link on mouse-down, resolves it to its driving axis, and
	// rotates that axis by projecting the mouse motion onto the axis's screen-space
	// tangent. mode is a GizmoMode cast to int (kept as int across the interface
	// so C++/CLI callers don't have to translate the native enum). Safe to call
	// whether or not drag is currently enabled; mid-drag calls are ignored by the
	// controllers.

	// ---- Camera ----------------------------------------------------------------

	virtual void fitAll() = 0;
	virtual void setViewIso() = 0;
	virtual void setViewTop() = 0;

	// ---- TCP trail visualisation ----------------------------------------------

	enum class TcpTrailMode : int
	{
		Off                = 0,  // no display, no sampling
		Polyline           = 1,  // TCP point trail only
		PolylineWithFrames = 2,  // TCP trail + sub-sampled mini trihedrons
	};

	virtual void setTcpTrailMode( int mode ) = 0;
	// Switches the TCP travel-history visualisation. mode is a TcpTrailMode cast
	// to int (kept as int across the interface so C++/CLI callers don't have to
	// translate the native enum). Off both stops sampling on subsequent joint
	// changes and removes any previously drawn trail.

	virtual void clearTcpTrail() = 0;
	// Empties the TCP history buffer and removes the displayed polyline / frames
	// from the viewer. Mode is preserved so the next joint update repopulates.

	virtual void setTcpTrailMaxPoints( int maxPoints ) = 0;
	// Updates the ring-buffer cap for the TCP trail. The implementation clamps
	// to a sane range (min 2, project-defined upper bound) and rebuilds the
	// visible polyline / frames immediately when the new cap shrinks below the
	// current buffer size. Values that don't change the effective cap are no-ops.

	virtual void setTcpTrailFrameStride( int stride ) = 0;
	// Updates the sub-sampling stride used by the PolylineWithFrames trail mode.
	// Clamped by the implementation; only takes visual effect while mode is
	// PolylineWithFrames but is stored for the next mode switch either way.

	virtual void setTcpTrailColor( int r, int g, int b ) = 0;
	// Sets the trail polyline colour. Components are 0..255; out-of-range values
	// are clamped. The frame trihedrons keep their X/Y/Z axis colours (blue /
	// green / red) so orientation stays legible regardless of the trail tint.

	// ---- Mouse / interaction ---------------------------------------------------

	virtual void onMouseDown( int x, int y, int button ) = 0;
	virtual void onMouseMove( int x, int y, int buttonMask ) = 0;
	virtual void onMouseUp() = 0;
	virtual void onMouseWheel( int delta ) = 0;

	// ---- Capture ---------------------------------------------------------------

	[[nodiscard]] virtual bool saveScreenshot( const wchar_t* filePath ) = 0;
	// Saves the current view to an image file. Format is inferred from the file
	// extension (.png recommended). Returns false on null/empty path, null view,
	// or encoder failure.

protected:
	IRobotScene() = default;
	IRobotScene( const IRobotScene& ) = delete;
	IRobotScene& operator=( const IRobotScene& ) = delete;
	IRobotScene( IRobotScene&& ) = delete;
	IRobotScene& operator=( IRobotScene&& ) = delete;
	// Interface is not copyable / movable; concrete implementations own non-trivial
	// resources and are exposed via std::unique_ptr<IRobotScene>.
};

[[nodiscard]] IRobotScene* createRobotScene();
// Factory entry point that hides the concrete RobotSceneFacade from clients.
// Caller owns the returned pointer and must `delete` it (or wrap in unique_ptr).
// Returning a raw pointer avoids forcing <memory> into this header and keeps
// the signature usable from C++/CLI translation units, which cannot store
// std::unique_ptr as a managed-class member field.

}  // namespace OccBridge
