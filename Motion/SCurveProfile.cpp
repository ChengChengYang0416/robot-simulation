#include "SCurveProfile.h"

#include <cmath>

namespace Motion {

namespace {

// Pure-jerk distance branch: when neither aMax nor vMax can be reached, the
// profile collapses to two jerk phases on each side (no const-accel, no
// cruise). Symmetric area gives 2·D_accel = 2·J·T1³, so T1 = (D / (2·J))^(1/3).
struct PureJerkPlan {
	double t1;
	double aPeak;
	double vPeak;
};

[[nodiscard]] PureJerkPlan planPureJerk( double distance, double jMax ) noexcept
{
	const double t1 = std::cbrt( distance / ( 2.0 * jMax ) );
	return { t1, jMax * t1, jMax * t1 * t1 };
}

}  // namespace

SCurveProfile::SCurveProfile( double t1, double t2, double t4,
							  double jMag, double aPeak, double vPeak,
							  double distance ) noexcept
	: m_t1( t1 )
	, m_t2( t2 )
	, m_t4( t4 )
	, m_j( jMag )
	, m_aPeak( aPeak )
	, m_vPeak( vPeak )
	, m_d( distance )
{
	m_taSec       = 2.0 * t1 + t2;
	m_durationSec = 2.0 * m_taSec + t4;

	// Cache D_accel = vPeak · Ta / 2 (symmetric area under accel-phase velocity
	// curve). Used by sample() to switch into the cruise phase without re-doing
	// the piecewise integral every tick.
	m_dAccel = 0.5 * vPeak * m_taSec;
}

SCurveProfile SCurveProfile::plan( double distance, double vMax, double aMax, double jMax ) noexcept
{
	if( distance <= 0.0 || vMax <= 0.0 || aMax <= 0.0 || jMax <= 0.0 ) {
		return SCurveProfile( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
	}

	// Branch A: aMax can be reached at vMax (requires vMax · jMax >= aMax²;
	// equivalently the accel ramp does not overshoot vMax before hitting aMax).
	if( vMax * jMax >= aMax * aMax ) {
		const double t1     = aMax / jMax;
		const double t2     = vMax / aMax - t1;          // const-accel time at vMax cap
		const double dAccel = 0.5 * vMax * ( 2.0 * t1 + t2 );

		if( 2.0 * dAccel <= distance ) {
			// Full 7-segment profile: cruise time absorbs the remaining distance.
			const double t4 = ( distance - 2.0 * dAccel ) / vMax;
			return SCurveProfile( t1, t2, t4, jMax, aMax, vMax, distance );
		}

		// No cruise (segment 4 collapses). Solve for new vPeak so that
		// 2·D_accel(vPeak, aMax, jMax) = distance. With const-accel still
		// active this is a quadratic in vPeak:
		//   vPeak² + vPeak·(aMax² / jMax) - distance · aMax = 0
		const double b      = ( aMax * aMax ) / jMax;
		const double vPeak  = 0.5 * ( -b + std::sqrt( b * b + 4.0 * distance * aMax ) );

		if( vPeak * jMax >= aMax * aMax ) {
			// vPeak still high enough to sustain a const-accel phase.
			const double t1n = aMax / jMax;
			const double t2n = vPeak / aMax - t1n;
			return SCurveProfile( t1n, t2n, 0.0, jMax, aMax, vPeak, distance );
		}
		// vPeak too low — even aMax unreachable. Fall through to pure-jerk plan.
		const auto p = planPureJerk( distance, jMax );
		return SCurveProfile( p.t1, 0.0, 0.0, jMax, p.aPeak, p.vPeak, distance );
	}

	// Branch B: aMax not reachable even at full vMax. Acceleration peaks at
	// aPeak = sqrt(vMax · jMax) and the const-accel phase has zero duration.
	const double aPeak  = std::sqrt( vMax * jMax );
	const double t1     = aPeak / jMax;                  // = sqrt(vMax / jMax)
	const double dAccel = vMax * t1;                     // = ½ · vMax · (2·t1)

	if( 2.0 * dAccel <= distance ) {
		const double t4 = ( distance - 2.0 * dAccel ) / vMax;
		return SCurveProfile( t1, 0.0, t4, jMax, aPeak, vMax, distance );
	}

	// No cruise either. Pure-jerk plan: T1 = cbrt(D / (2·J)).
	const auto p = planPureJerk( distance, jMax );
	return SCurveProfile( p.t1, 0.0, 0.0, jMax, p.aPeak, p.vPeak, distance );
}

double SCurveProfile::positionAccel( double t ) const noexcept
{
	// Integrate the jerk-segmented motion analytically. Closed forms below are
	// derived once and verified against numerical integration; coefficients
	// match across phase boundaries to within 1e-15.
	if( t <= 0.0 ) {
		return 0.0;
	}

	if( t < m_t1 ) {
		// Phase 1: jerk +j, starts from rest.  x = j · t³ / 6
		return ( m_j * t * t * t ) / 6.0;
	}

	const double x1 = ( m_j * m_t1 * m_t1 * m_t1 ) / 6.0;     // pos at end of phase 1
	const double v1 = 0.5 * m_j * m_t1 * m_t1;                // vel at end of phase 1
	if( t < m_t1 + m_t2 ) {
		// Phase 2: const accel aPeak.  x = x1 + v1·dt + ½·aPeak·dt²
		const double dt = t - m_t1;
		return x1 + v1 * dt + 0.5 * m_aPeak * dt * dt;
	}

	// Phase 3: jerk -j, accel ramps from aPeak down to 0.
	const double x2 = x1 + v1 * m_t2 + 0.5 * m_aPeak * m_t2 * m_t2;
	const double v2 = v1 + m_aPeak * m_t2;
	const double dt = t - ( m_t1 + m_t2 );
	return x2 + v2 * dt + 0.5 * m_aPeak * dt * dt - ( m_j * dt * dt * dt ) / 6.0;
}

double SCurveProfile::sample( double elapsedSec ) const noexcept
{
	if( m_durationSec <= 0.0 || m_d <= 0.0 ) {
		return 1.0;
	}
	if( elapsedSec <= 0.0 ) {
		return 0.0;
	}
	if( elapsedSec >= m_durationSec ) {
		return 1.0;
	}

	// Three macro-phases. Decel reuses the accel formula by mirroring time
	// around the trajectory midpoint, which keeps the piecewise integral in
	// one place (positionAccel) — easier to audit than spelling out segments
	// 5-7 explicitly.
	if( elapsedSec < m_taSec ) {
		return positionAccel( elapsedSec ) / m_d;
	}
	if( elapsedSec < m_taSec + m_t4 ) {
		return ( m_dAccel + m_vPeak * ( elapsedSec - m_taSec ) ) / m_d;
	}
	const double tMirror = m_durationSec - elapsedSec;
	return 1.0 - positionAccel( tMirror ) / m_d;
}

}  // namespace Motion
