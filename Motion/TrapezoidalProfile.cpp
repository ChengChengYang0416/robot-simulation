#include "TrapezoidalProfile.h"

#include <cmath>

namespace Motion {

TrapezoidalProfile::TrapezoidalProfile( double durationSec, double accelTimeSec ) noexcept
{
	m_durationSec = durationSec;
	m_ta          = accelTimeSec;
	// Normalised-area constraint vPeak * (T - ta) = 1; reduces to 2 / T when
	// ta = T / 2 (triangular profile), matching textbook trapezoidal math.
	if( durationSec <= 0.0 || accelTimeSec <= 0.0 ) {
		m_vPeakNorm = 0.0;
	} else {
		m_vPeakNorm = 1.0 / ( durationSec - accelTimeSec );
	}
}

TrapezoidalProfile TrapezoidalProfile::plan( double distance, double vMax, double aMax ) noexcept
{
	if( distance <= 0.0 || vMax <= 0.0 || aMax <= 0.0 ) {
		return TrapezoidalProfile( 0.0, 0.0 );
	}

	// dAccel: distance required to ramp from 0 to vMax at aMax. If 2 * dAccel
	// <= D the cruise phase has positive duration → full trapezoid. Otherwise
	// the move is short: the profile becomes triangular with a peak velocity
	// vPeak = sqrt(D * aMax) that never reaches vMax.
	const double dAccel = ( vMax * vMax ) / ( 2.0 * aMax );
	double T;
	double ta;
	if( 2.0 * dAccel <= distance ) {
		ta = vMax / aMax;
		T  = ta + distance / vMax;       // = 2 * ta + cruise; cruise = D / vMax - ta
	} else {
		ta = std::sqrt( distance / aMax );
		T  = 2.0 * ta;                    // triangular: no cruise phase
	}
	return TrapezoidalProfile( T, ta );
}

double TrapezoidalProfile::sample( double elapsedSec ) const noexcept
{
	if( m_durationSec <= 0.0 ) {
		return 1.0;
	}
	if( elapsedSec <= 0.0 ) {
		return 0.0;
	}
	if( elapsedSec >= m_durationSec ) {
		return 1.0;
	}

	// Three phases over [0, T]:
	//   accel  : t ∈ [0, ta)         s = ½ * (vPeak / ta) * t²
	//   cruise : t ∈ [ta, T - ta)    s = ½ * vPeak * ta + vPeak * (t - ta)
	//   decel  : t ∈ [T - ta, T]     s = 1 - ½ * (vPeak / ta) * (T - t)²
	// Coefficients are chosen so the phase boundaries match exactly and the
	// total integral equals 1; numerical drift at the joins is below 1e-15.
	const double tDecelStart = m_durationSec - m_ta;
	if( elapsedSec < m_ta ) {
		return 0.5 * ( m_vPeakNorm / m_ta ) * elapsedSec * elapsedSec;
	}
	if( elapsedSec < tDecelStart ) {
		return 0.5 * m_vPeakNorm * m_ta + m_vPeakNorm * ( elapsedSec - m_ta );
	}
	const double tr = m_durationSec - elapsedSec;
	return 1.0 - 0.5 * ( m_vPeakNorm / m_ta ) * tr * tr;
}

}  // namespace Motion
