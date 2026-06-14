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

	[[nodiscard]] virtual bool getTcpPose( double out[6] ) const = 0;
	// Fills out with [x, y, z, rx, ry, rz] in mm and degrees (ZYX intrinsic Euler);
	// returns false when no robot is loaded or out is null.

	// ---- Camera ----------------------------------------------------------------

	virtual void fitAll() = 0;
	virtual void setViewIso() = 0;
	virtual void setViewTop() = 0;

	// ---- Mouse / interaction ---------------------------------------------------

	virtual void onMouseDown( int x, int y, int button ) = 0;
	virtual void onMouseMove( int x, int y, int buttonMask ) = 0;
	virtual void onMouseUp() = 0;
	virtual void onMouseWheel( int delta ) = 0;

protected:
	IRobotScene() = default;
	IRobotScene( const IRobotScene& ) = delete;
	IRobotScene& operator=( const IRobotScene& ) = delete;
	IRobotScene( IRobotScene&& ) = delete;
	IRobotScene& operator=( IRobotScene&& ) = delete;
	// Interface is not copyable / movable; concrete implementations own non-trivial
	// resources and are exposed via std::unique_ptr<IRobotScene>.
};

}  // namespace OccBridge
