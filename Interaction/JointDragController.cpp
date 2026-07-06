#include "JointDragController.h"

#include <cmath>
#include <gp_XYZ.hxx>

namespace Interaction {

namespace {

// Sensitivity in degrees of joint rotation per pixel of tangential mouse motion.
// 0.5 gives ~50° for a 100 px drag, comparable to industrial teach-pendant jog
// wheels and matches the feel of the AIS_Manipulator's translation drag at the
// default 120 mm gizmo size. Tunable if operator feedback disagrees.
constexpr double kDegPerPixel = 0.5;

// Matches Interaction::MouseButton::Left (kept as a local constant to avoid a
// cross-header dependency; MouseInteractor already exposes the same value and
// ManipulatorController uses the same convention).
constexpr int kLeftButton = 1048576;

// Length in mm used to construct the reference offset point from the joint axis.
// Only the DIRECTION of the projected screen segment matters, not the length, so
// any positive value works; 100 mm keeps the two projected points comfortably
// separated in screen space even when the camera is zoomed out.
constexpr double kReferenceRadiusMm = 100.0;

// Threshold below which the projected axis is treated as viewed edge-on. When the
// axis is nearly parallel to the view direction the projected segment collapses
// and the tangent direction is ill-defined; we fall back to plain horizontal drag.
constexpr double kMinScreenSegmentPx = 1.0e-3;

gp_XYZ perpendicular( const gp_XYZ& v )
// Returns a unit vector perpendicular to v. Picks the world axis least parallel to v
// as the cross-product partner so the result never collapses regardless of v's
// direction. Assumes v is already normalised.
{
	gp_XYZ helper( 1.0, 0.0, 0.0 );
	if( std::abs( v.Dot( helper ) ) > 0.9 ) {
		helper = gp_XYZ( 0.0, 1.0, 0.0 );
	}
	gp_XYZ result = v.Crossed( helper );
	result.Normalize();
	return result;
}

}  // namespace

void JointDragController::attach( const Handle( V3d_View ) & view,
								  const Handle( AIS_InteractiveContext ) & context )
{
	m_view    = view;
	m_context = context;
}

void JointDragController::setEnabled( bool enabled )
{
	if( !enabled && m_isDragging ) {
		m_isDragging = false;
		m_dragAxis   = -1;
	}
	m_enabled = enabled;
}

void JointDragController::setAxisLookup( std::unordered_map<AIS_InteractiveObject*, int> lookup )
{
	m_axisLookup = std::move( lookup );
}

void JointDragController::setAxisFrameProvider( AxisFrameProvider provider )
{
	m_axisFrameProvider = std::move( provider );
}

void JointDragController::setJointDragHandler( JointDragHandler handler )
{
	m_handler = std::move( handler );
}

bool JointDragController::onMouseDown( int x, int y, int button )
// Left-click on a driven link → resolve axis → compute screen tangent → arm the drag.
// Everything else returns false so the camera interactor handles the click. The hover
// state consulted here (HasDetected / DetectedInteractive) is maintained by
// MouseInteractor's onMouseMove; re-running MoveTo() inside a Windows mouse event has
// caused crashes in the past (documented in the manipulator controller) so we trust
// the cached hover instead.
{
	if( !m_enabled || m_view.IsNull() || m_context.IsNull() ) {
		return false;
	}
	if( button != kLeftButton ) {
		return false;
	}
	if( !m_context->HasDetected() ) {
		return false;
	}
	Handle( AIS_InteractiveObject ) detected = m_context->DetectedInteractive();
	if( detected.IsNull() ) {
		return false;
	}
	auto it = m_axisLookup.find( detected.get() );
	if( it == m_axisLookup.end() ) {
		return false;
	}
	const int axisOneBased = it->second;
	if( axisOneBased < 1 || axisOneBased > 6 ) {
		return false;
	}
	if( !m_axisFrameProvider ) {
		return false;
	}

	gp_Pnt origin;
	gp_Dir dir;
	if( !m_axisFrameProvider( axisOneBased, origin, dir ) ) {
		return false;
	}

	// Compute radial from the ACTUAL click location on the ring, not from an
	// arbitrary world-axis helper. Tangent direction reverses on opposite sides
	// of a circle, so a fixed radial gives the correct sign only when the user
	// happens to click near it — for every other click the drag feels inverted.
	// Unproject the click into a world ray (view.ConvertWithProj), intersect the
	// plane containing the ring (normal = axis, through origin), and use the
	// hit point to build radial.
	const gp_XYZ  dirXYZ = dir.XYZ();
	Standard_Real px = 0.0, py = 0.0, pz = 0.0, vx = 0.0, vy = 0.0, vz = 0.0;
	m_view->ConvertWithProj( x, y, px, py, pz, vx, vy, vz );
	const gp_XYZ rayOrigin( px, py, pz );
	gp_XYZ       rayDir( vx, vy, vz );
	if( rayDir.SquareModulus() > 0.0 ) {
		rayDir.Normalize();
	}
	gp_XYZ       radial;
	const double denom = rayDir.Dot( dirXYZ );
	if( std::abs( denom ) > 1.0e-6 ) {
		// Solve rayOrigin + t·rayDir hits plane {P : (P - origin)·axis = 0}
		const double t          = ( origin.XYZ() - rayOrigin ).Dot( dirXYZ ) / denom;
		const gp_XYZ hit        = rayOrigin + rayDir * t;
		gp_XYZ       radialRaw  = hit - origin.XYZ();
		radialRaw               -= dirXYZ * radialRaw.Dot( dirXYZ );   // remove axial drift
		if( radialRaw.SquareModulus() > 1.0e-9 ) {
			radial = radialRaw;
			radial.Normalize();
		} else {
			radial = perpendicular( dirXYZ );   // click at axis centre, degenerate
		}
	} else {
		radial = perpendicular( dirXYZ );       // ray parallel to ring plane, degenerate
	}

	// Compute a world-space tangent for a small +dθ rotation, then project two points
	// (P0 and P0 + tangent) to screen coordinates. The screen delta between them IS
	// the screen tangent for +dθ, with the correct sign baked in by the right-hand
	// rule. This sidesteps every sign-flip corner case that a per-axis view-direction
	// heuristic would need to handle (axis pointing into vs. out of the screen, Y-up
	// vs. Y-down screen conventions, etc.).
	const gp_XYZ tangent = dirXYZ.Crossed( radial );                     // axis × r → +dθ tangent
	const gp_XYZ P0      = origin.XYZ() + radial  * kReferenceRadiusMm;
	const gp_XYZ P1      = P0            + tangent * kReferenceRadiusMm;

	Standard_Integer sx0 = 0, sy0 = 0, sx1 = 0, sy1 = 0;
	m_view->Convert( P0.X(), P0.Y(), P0.Z(), sx0, sy0 );
	m_view->Convert( P1.X(), P1.Y(), P1.Z(), sx1, sy1 );

	const double tdx = static_cast<double>( sx1 - sx0 );
	const double tdy = static_cast<double>( sy1 - sy0 );
	const double len = std::hypot( tdx, tdy );
	if( len < kMinScreenSegmentPx ) {
		// Axis viewed edge-on: fall back to horizontal drag with an arbitrary sign
		// (positive dx → positive rotation). Better than a NaN tangent.
		m_tangentX = 1.0;
		m_tangentY = 0.0;
	} else {
		m_tangentX = tdx / len;
		m_tangentY = tdy / len;
	}

	m_dragAxis   = axisOneBased - 1;   // convert to 0-based for the handler
	m_isDragging = true;
	m_lastX      = x;
	m_lastY      = y;
	return true;
}

bool JointDragController::onMouseMove( int x, int y, int buttonMask )
// Project the pixel delta since the previous move onto the pre-computed screen
// tangent. The dot product magnitude is the tangential pixel travel; multiplied by
// the sensitivity constant it becomes a signed degree delta. The facade closure
// handles clamping to joint limits and committing the new angle. buttonMask is
// unused — we started the drag on left-down and end it on any mouse-up, so the
// button state during the drag itself is irrelevant.
{
	(void)buttonMask;
	if( !m_isDragging ) {
		return false;
	}
	if( m_dragAxis < 0 || !m_handler ) {
		// Defensive: drag was cleared out from under us (setEnabled(false) race).
		return false;
	}

	const double dx = static_cast<double>( x - m_lastX );
	const double dy = static_cast<double>( y - m_lastY );
	const double tangentialPx = dx * m_tangentX + dy * m_tangentY;
	m_lastX = x;
	m_lastY = y;

	if( tangentialPx != 0.0 ) {
		m_handler( m_dragAxis, tangentialPx * kDegPerPixel );
	}
	return true;
}

bool JointDragController::onMouseUp()
{
	if( !m_isDragging ) {
		return false;
	}
	m_isDragging = false;
	m_dragAxis   = -1;
	return true;
}

}  // namespace Interaction
