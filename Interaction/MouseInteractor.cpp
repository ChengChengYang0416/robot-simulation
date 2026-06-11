#include "MouseInteractor.h"

namespace Interaction {

namespace {
inline constexpr double kZoomFactor = 1.15;
}

void MouseInteractor::attach( const Handle( V3d_View ) & view,
							  const Handle( AIS_InteractiveContext ) & context )
// Caches both handles together; either being null disables the corresponding behaviour
// (e.g. no context -> no hover highlight, no view -> no rotation / pan / zoom).
{
	m_view = view;
	m_context = context;
}

void MouseInteractor::onMouseDown( int x, int y, int button )
// V3d_View::StartRotation must be called once at the anchor position; subsequent
// onMouseMove calls feed absolute coordinates to V3d_View::Rotation.
{
	m_lastX = x;
	m_lastY = y;

	if( button == static_cast<int>( MouseButton::Left ) ) {
		m_isRotating = true;
		if( !m_view.IsNull() ) {
			m_view->StartRotation( x, y );
		}
	} else if( button == static_cast<int>( MouseButton::Middle ) ) {
		m_isPanning = true;
	}
}

void MouseInteractor::onMouseMove( int x, int y, int /*buttonMask*/ )
// Pan needs delta values (and Y is flipped because OCCT uses bottom-up screen coords),
// so lastX / lastY are advanced each move. Rotation feeds absolute coords directly.
{
	if( m_view.IsNull() ) {
		return;
	}

	if( m_isRotating ) {
		m_view->Rotation( x, y );
	} else if( m_isPanning ) {
		m_view->Pan( x - m_lastX, m_lastY - y );
		m_lastX = x;
		m_lastY = y;
	} else if( !m_context.IsNull() ) {
		m_context->MoveTo( x, y, m_view, Standard_True );
	}
}

void MouseInteractor::onMouseUp()
// Idempotent; safe to call without a prior onMouseDown.
{
	m_isRotating = false;
	m_isPanning = false;
}

void MouseInteractor::onMouseWheel( int delta )
// SetZoom multiplies the current zoom; positive delta amplifies, negative shrinks
// (the reciprocal keeps zoom-in / zoom-out symmetric across wheel ticks).
{
	if( m_view.IsNull() ) {
		return;
	}

	const double factor = ( delta > 0 ) ? kZoomFactor : ( 1.0 / kZoomFactor );
	m_view->SetZoom( factor );
	m_view->Redraw();
}

}  // namespace Interaction
