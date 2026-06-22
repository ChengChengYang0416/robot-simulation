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

	// ---- Camera ----------------------------------------------------------------

	virtual void fitAll() = 0;
	virtual void setViewIso() = 0;
	virtual void setViewTop() = 0;

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
