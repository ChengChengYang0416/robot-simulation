#pragma once

struct HWND__;
using HWND = HWND__*;
struct RobotPartDef;

class NativeOccView
{
public:
	NativeOccView( void );
	// Constructor; allocates the PIMPL implementation object

	~NativeOccView( void );
	// Destructor; releases the PIMPL implementation object

	void initialize( HWND hwnd );
	// Initializes the OCCT 3D viewer and binds it to the given window handle

	void resize( int width, int height );
	// Notifies the viewer that the window size has changed and triggers a redraw

	void redraw( void );
	// Forces an immediate redraw of the 3D scene

	[[nodiscard]] bool loadStep( const wchar_t* filePath, bool append );
	// Loads a STEP file; clears the scene first when append is false; returns true on success

	[[nodiscard]] bool beginRobotArm( const RobotPartDef* parts, int partCount,
									  const int* axisToPartMap, int mapCount );
	// Clears the scene and stores part definitions, axis mapping, and joint angles

	[[nodiscard]] bool loadRobotPart( int index );
	// Loads a single STEP file for the given part index and displays it with its color

	void endRobotArm( void );
	// Finalizes the robot arm by computing DH transforms and fitting the view

	void setJointAngle( int axisIndex, double angleDeg );
	// Sets the joint angle (degrees) for the given 0-based axis and updates the scene

	[[nodiscard]] bool getTcpPose( double out[6] ) const;
	// Fills out with TCP pose [x, y, z, rx, ry, rz] in mm and degrees (ZYX intrinsic Euler);
	// returns false if no robot is currently loaded

	void clearScene( void );
	// Removes all objects from the scene and resets robot arm state

	void fitAll( void );
	// Auto-fits the camera to fully enclose all scene objects

	void setViewIso( void );
	// Switches to an isometric view projection

	void setViewTop( void );
	// Switches to a top-down view projection

	void onMouseDown( int x, int y, int button );
	// Handles mouse button press; begins rotation (left button) or pan (middle button)

	void onMouseMove( int x, int y, int buttonMask );
	// Handles mouse move; performs rotation, pan, or cursor highlight

	void onMouseUp( void );
	// Handles mouse button release; stops rotation and pan

	void onMouseWheel( int delta );
	// Handles mouse wheel; zooms in when delta > 0, zooms out when delta < 0

private:
	void updateRobotTransforms( void );
	// Recomputes cumulative DH transforms for all parts based on current joint angles

	struct Impl;
	// PIMPL implementation struct; defined in the .cpp file to hide OCCT headers and details

	Impl* m_impl;
	// Pointer to the PIMPL implementation; manages OCCT objects and state
};
