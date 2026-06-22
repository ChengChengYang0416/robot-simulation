#pragma once

#include "MotionProfile.h"

namespace Motion {

class TrapezoidalProfile final : public Profile
{
// Trapezoidal velocity profile with zero start / end velocity and a constant-
// acceleration ramp on both ends. The profile is normalised so that the
// integral of ds/dt over [0, T] equals 1 regardless of physical distance D,
// which lets callers multiply s by their own dimension delta and reuse the
// same profile across heterogeneous units (mm and degrees in MoveL,
// degrees only in MoveJ).
public:
	TrapezoidalProfile( const TrapezoidalProfile& )            = default;
	TrapezoidalProfile& operator=( const TrapezoidalProfile& ) = default;

	[[nodiscard]] static TrapezoidalProfile plan( double distance,
												  double vMax,
												  double aMax ) noexcept;
	// Shortest-time trapezoidal traversal of distance honouring vMax and aMax.
	// Degenerates to a triangular profile (no cruise phase, peak velocity
	// below vMax) when 2 * dAccel exceeds the requested distance, so callers
	// do not need to special-case short moves.
	// All three arguments must share consistent units (e.g. mm + mm/s + mm/s²).

	[[nodiscard]] double sample( double elapsedSec ) const noexcept override;

private:
	TrapezoidalProfile( double durationSec, double accelTimeSec ) noexcept;

	double m_ta        = 0.0;
	// Acceleration / deceleration phase duration in seconds; for a triangular
	// profile this equals T / 2.

	double m_vPeakNorm = 0.0;
	// Peak ds/dt in units of 1/s. Derived from the normalised-area constraint
	// vPeak * (T - ta) = 1, which reduces to 2 / T for the triangular case.
};

}  // namespace Motion
