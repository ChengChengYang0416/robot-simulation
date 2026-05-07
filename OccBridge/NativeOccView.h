#pragma once

struct HWND__;
using HWND = HWND__*;
struct RobotPartDef;

class NativeOccView
{
public:
	NativeOccView();
	// Constructor; allocates the PIMPL implementation object

	~NativeOccView();
	// Destructor; releases the PIMPL implementation object

	void initialize(HWND hwnd);
	// Initializes the OCCT 3D viewer and binds it to the given window handle

	void resize(int width, int height);
	// Notifies the viewer that the window size has changed and triggers a redraw

	void redraw();
	// Forces an immediate redraw of the 3D scene

	[[nodiscard]] bool loadStep(const wchar_t* filePath, bool append);
	// Loads a STEP file; clears the scene first when append is false; returns true on success

	[[nodiscard]] bool loadRobotArm(const RobotPartDef* parts, int partCount,
									const int* axisToPartMap, int mapCount);
	// Loads robot arm parts and builds the DH kinematic chain
	// axisToPartMap is an interleaved [axisIdx, partIdx] array of length mapCount

	void setJointAngle(int axisIndex, double angleDeg);
	// Sets the joint angle (degrees) for the given 0-based axis and updates the scene

	void clearScene();
	// Removes all objects from the scene and resets robot arm state

	void fitAll();
	// Auto-fits the camera to fully enclose all scene objects

	void setViewIso();
	// Switches to an isometric view projection

	void setViewTop();
	// Switches to a top-down view projection

	void onMouseDown(int x, int y, int button);
	// Handles mouse button press; begins rotation (left button) or pan (middle button)

	void onMouseMove(int x, int y, int buttonMask);
	// Handles mouse move; performs rotation, pan, or cursor highlight

	void onMouseUp();
	// Handles mouse button release; stops rotation and pan

	void onMouseWheel(int delta);
	// Handles mouse wheel; zooms in when delta > 0, zooms out when delta < 0

private:
	void updateRobotTransforms();
	// Recomputes cumulative DH transforms for all parts based on current joint angles

	struct Impl;
	// PIMPL implementation struct; defined in the .cpp file to hide OCCT headers and details

	Impl* m_impl;
	// Pointer to the PIMPL implementation; manages OCCT objects and state
};
