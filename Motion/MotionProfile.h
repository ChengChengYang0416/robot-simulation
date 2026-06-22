#pragma once

namespace Motion {

class Profile
{
// Time-parameterised normalised-arc-length profile s(t) ∈ [0, 1]. A trajectory
// player computes the total duration once, then samples s(elapsed) per frame
// and lerps every endpoint dimension by that same s so all dimensions stay
// synchronised. Concrete subclasses (LinearProfile, TrapezoidalProfile,
// future SCurveProfile, ...) differ only in the shape of s(t), letting
// MoveL / MoveJ swap velocity profiles without touching the player loop.
//
// This header intentionally contains only the abstract interface; concrete
// shapes live in their own headers so adding a new profile (e.g. S-curve)
// means adding files, not editing this one.
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

}  // namespace Motion
