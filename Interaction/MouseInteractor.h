#pragma once

#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>

namespace Interaction {

enum class MouseButton : int
{
	Left   = 1048576,
	Middle = 4194304
};
// Matches the WinForms MouseButtons bitmask values that OccViewerControl forwards;
// kept here so callers do not need to hardcode the magic numbers at the boundary.

class MouseInteractor
{
public:
	void attach( const Handle( V3d_View ) & view,
				 const Handle( AIS_InteractiveContext ) & context );
	// Stores the OCCT view and AIS context the interactor drives. Must be called once
	// after they are constructed; re-attaching mid-interaction is not supported.

	void onMouseDown( int x, int y, int button );
	// Records the cursor anchor; MouseButton::Left starts a V3d_View rotation,
	// MouseButton::Middle starts a pan. Other buttons are ignored.

	void onMouseMove( int x, int y, int buttonMask );
	// Drives rotation or pan when the corresponding flag is active. With neither
	// flag, forwards (x, y) to AIS_InteractiveContext::MoveTo for hover highlight.

	void onMouseUp();
	// Clears the rotating / panning flags, ending the active interaction.

	void onMouseWheel( int delta );
	// Zooms the view by a fixed factor based on wheel sign (delta > 0 zooms in).

private:
	Handle( V3d_View ) m_view;
	Handle( AIS_InteractiveContext ) m_context;
	int m_lastX = 0;
	int m_lastY = 0;
	bool m_isRotating = false;
	bool m_isPanning  = false;
};

}  // namespace Interaction
