#include "CameraController.h"

namespace Interaction {

void CameraController::attach( const Handle( V3d_View ) & view )
// Stores the view handle; re-attaching mid-session is allowed but typically
// only done once during initialization.
{
	m_view = view;
}

void CameraController::setViewIso()
// V3d_XposYnegZpos is the OCCT enum for the standard isometric look the HMI uses
// as its default; refit so the scene stays centred after the projection swap.
{
	if( m_view.IsNull() ) {
		return;
	}
	m_view->SetProj( V3d_XposYnegZpos );
	fitAll();
}

void CameraController::setViewTop()
// Top-down (looking along -Z toward +Z plane); refit afterwards for the same reason.
{
	if( m_view.IsNull() ) {
		return;
	}
	m_view->SetProj( V3d_Zpos );
	fitAll();
}

void CameraController::fitAll()
// FitAll centres the scene in the viewport; ZFitAll adjusts the near/far planes so
// nothing is clipped. Redraw is required because both calls only mark dirty state.
{
	if( m_view.IsNull() ) {
		return;
	}
	m_view->FitAll();
	m_view->ZFitAll();
	m_view->Redraw();
}

}  // namespace Interaction
