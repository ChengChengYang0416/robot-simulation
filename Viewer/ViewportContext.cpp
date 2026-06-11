#include "ViewportContext.h"
#include <Aspect_TypeOfTriedronPosition.hxx>
#include <Quantity_Color.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <V3d_TypeOfVisualization.hxx>
#include <WNT_Window.hxx>

namespace Viewer {

void ViewportContext::initialize( HWND hwnd )
// OCCT rendering stack (bottom to top):
//   DisplayConnection -> GraphicDriver -> Viewer -> View / InteractiveContext
// The Viewer owns lights and shared state; the Context manages AIS objects.
{
	m_hwnd = hwnd;

	m_displayConnection = new Aspect_DisplayConnection();
	m_graphicDriver = new OpenGl_GraphicDriver( m_displayConnection );
	m_viewer = new V3d_Viewer( m_graphicDriver );
	m_viewer->SetDefaultLights();
	m_viewer->SetLightOn();
	m_context = new AIS_InteractiveContext( m_viewer );
	m_view = m_viewer->CreateView();

	// Bind the OCCT view to the host HWND. WNT_Window must be mapped before the
	// first redraw, otherwise OpenGL has no surface to draw onto.
	Handle( WNT_Window ) window = new WNT_Window( reinterpret_cast<Aspect_Handle>( m_hwnd ) );
	m_view->SetWindow( window );
	if( !window->IsMapped() ) {
		window->Map();
	}

	m_view->SetBackgroundColor( Quantity_NOC_GRAY30 );
	// Corner trihedron: per-axis colors (X=blue, Y=green, Z=red) must be configured
	// via ZBufferTriedronSetup before calling TriedronDisplay.
	m_view->ZBufferTriedronSetup( Quantity_NOC_BLUE, Quantity_NOC_GREEN, Quantity_NOC_RED );
	m_view->TriedronDisplay( Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08, V3d_ZBUFFER );
	m_view->MustBeResized();
	m_view->SetProj( V3d_XposYnegZpos );
	m_view->Redraw();
}

void ViewportContext::resize( int /*width*/, int /*height*/ )
// OCCT queries the bound WNT_Window for the actual size, so the int parameters
// are advisory only. Kept in the signature for HMI parity.
{
	if( !m_view.IsNull() ) {
		m_view->MustBeResized();
		m_view->Redraw();
	}
}

void ViewportContext::redraw()
// Fast path that skips MustBeResized; use when the scene changed but the window did not.
{
	if( !m_view.IsNull() ) {
		m_view->Redraw();
	}
}

}  // namespace Viewer
