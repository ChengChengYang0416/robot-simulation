#pragma once

#include "Quaternion.h"

namespace OccBridge {

class PoseInterpolator
// Stateful, single-segment slerp of TCP orientation. Owns the start/end
// quaternions for one MoveL segment so the player tick can sample by progress
// fraction s in [0, 1] without re-running fromZyxDeg every frame.
//
// Why a class instead of a free function: callers (MoveLTimer_Tick) need to
// sample many times per segment with the *same* endpoints; recomputing the
// quaternions on every tick would double the trig cost and prevent the
// totalAngleRad() / totalAngleDeg() helpers from being O(1) lookups. The state
// is intentionally tiny (two quats + cached angle) so per-segment construction
// is essentially free and the object is trivially copyable.
//
// Threading: not thread-safe. Each motion player owns its own instance.
{
public:
	PoseInterpolator() noexcept = default;

	void begin( double startRxDeg, double startRyDeg, double startRzDeg,
				double targetRxDeg, double targetRyDeg, double targetRzDeg ) noexcept;
	// Resets the interpolator to a new segment. Both endpoints are converted
	// to quaternions once and the geodesic angle between them is cached so
	// totalAngleRad() / totalAngleDeg() are O(1) afterwards. Safe to call
	// repeatedly to chain segments.

	void sample( double s, double& outRxDeg, double& outRyDeg, double& outRzDeg ) const noexcept;
	// Returns the interpolated ZYX-intrinsic Euler angles at progress s. Values
	// of s outside [0, 1] are clamped so the caller does not need to special-case
	// the boundary conditions (overshoot from a slightly leading virtual clock,
	// in particular, is benign and produces the exact endpoint).

	[[nodiscard]] double totalAngleRad() const noexcept { return m_angleRad; }
	[[nodiscard]] double totalAngleDeg() const noexcept;
	// Geodesic rotation angle between begin's start and target endpoints. Use
	// this as the "angular distance" argument to a motion profile so the planned
	// duration tracks the *true* shortest-arc length, not the per-axis-max
	// approximation that the old element-wise Euler lerp had to use.

private:
	Quat   m_qStart{};
	Quat   m_qTarget{};
	double m_angleRad{ 0.0 };
};

}  // namespace OccBridge
