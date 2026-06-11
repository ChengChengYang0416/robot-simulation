#pragma once

#include <AIS_InteractiveContext.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>

struct HWND__;
using HWND = HWND__*;

namespace Viewer {

class ViewportContext
{
public:
	void initialize( HWND hwnd );
	// Builds the OCCT rendering stack (DisplayConnection -> GraphicDriver -> Viewer ->
	// View / InteractiveContext) and binds it to the host Win32 window. Also configures
	// default lights, gray background, and the lower-left corner trihedron. Calling
	// twice without a clear is unsupported and will leak the previous stack.

	void resize( int width, int height );
	// Notifies the OCCT View that the window size changed and triggers a redraw.
	// Width and height are accepted for API parity but OCCT queries the WNT_Window
	// directly, so they are unused.

	void redraw();
	// Forces an immediate View::Redraw without recalculating scene structure.

	[[nodiscard]] Handle( V3d_View ) view() const
	{
		return m_view;
	}
	// Returns the OCCT view so callers (camera, mouse, scene repo) can attach to it.

	[[nodiscard]] Handle( AIS_InteractiveContext ) context() const
	{
		return m_context;
	}
	// Returns the AIS interactive context that owns all displayed objects.

private:
	HWND m_hwnd = nullptr;
	Handle( Aspect_DisplayConnection ) m_displayConnection;
	Handle( OpenGl_GraphicDriver ) m_graphicDriver;
	Handle( V3d_Viewer ) m_viewer;
	Handle( V3d_View ) m_view;
	Handle( AIS_InteractiveContext ) m_context;
};

}  // namespace Viewer
