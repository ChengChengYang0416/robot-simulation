#pragma once

#include <V3d_View.hxx>

namespace Interaction {

class CameraController
{
public:
	void attach( const Handle( V3d_View ) & view );
	// Stores the OCCT view this controller drives. Must be called once after the
	// view is constructed; null handle disables all camera operations.

	void setViewIso();
	// Switches the projection to X+Y-Z+ (isometric) and refits the scene.

	void setViewTop();
	// Switches the projection to Z+ (top-down) and refits the scene.

	void fitAll();
	// Runs FitAll + ZFitAll so the whole scene is visible in both screen and depth
	// axes, then redraws. No-op when no view is attached.

private:
	Handle( V3d_View ) m_view;
};

}  // namespace Interaction
