#pragma once

#include <memory>

namespace OccBridge::Motion {

class Profile
{
// Time-parameterised normalised-arc-length profile s(t) ∈ [0, 1]. A trajectory
// player computes the total duration once, then samples s(elapsed) per frame
// and lerps every endpoint dimension by that same s so all dimensions stay
// synchronised. Concrete subclasses differ only in the shape of s(t) (linear,
// trapezoidal, S-curve, ...), letting MoveL / MoveJ swap velocity profiles
// without touching the player loop.
public:
	virtual ~Profile() = default;

	[[nodiscard]] double durationSec() const noexcept { return m_durationSec; }
	// Total motion duration in seconds; zero when the profile is degenerate
	// (distance = 0 or invalid construction arguments).

	[[nodiscard]] virtual double sample( double elapsedSec ) const noexcept = 0;
	// Returns s ∈ [0, 1] for the elapsed time. Implementations must clamp to 0
	// for t ≤ 0 and to 1 for t ≥ durationSec so callers can rely on monotone
	// progress without bounds checks.

protected:
	double m_durationSec = 0.0;

	Profile()                            = default;
	// Defaulted copy / move so concrete subclasses can be value types. Slicing
	// would be unsafe in principle, but no caller ever stores or copies a base
	// Profile by value — all polymorphic ownership goes through pointers.
	Profile( const Profile& )            = default;
	Profile& operator=( const Profile& ) = default;
	Profile( Profile&& )                 = default;
	Profile& operator=( Profile&& )      = default;
};

class LinearProfile final : public Profile
{
// Constant-velocity profile: s(t) = t / T. Equivalent to the original
// MoveL / MoveJ behaviour before velocity planning was introduced; kept as a
// drop-in fallback and as a baseline for the trapezoidal numerics.
public:
	explicit LinearProfile( double durationSec ) noexcept;
	LinearProfile( const LinearProfile& )            = default;
	LinearProfile& operator=( const LinearProfile& ) = default;

	[[nodiscard]] double sample( double elapsedSec ) const noexcept override;
};

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

}  // namespace OccBridge::Motion
