#pragma once

#include "MotionProfile.h"

namespace Motion {

class SCurveProfile final : public Profile
{
// Seven-segment jerk-limited (S-curve) profile. Velocity ramps follow a
// continuous-acceleration shape so the jerk magnitude is bounded by jMax,
// eliminating the acceleration step that a trapezoidal profile imposes at
// every phase boundary and giving visibly smoother starts / stops.
//
// Segment layout (zero initial / final velocity, symmetric accel / decel):
//   1. jerk +jMax        — accel ramps  0  →  aPeak
//   2. const accel       — accel = aPeak
//   3. jerk -jMax        — accel ramps  aPeak  →  0
//   4. cruise            — velocity = vPeak
//   5. jerk -jMax        — accel ramps  0  →  -aPeak
//   6. const decel       — accel = -aPeak
//   7. jerk +jMax        — accel ramps  -aPeak →  0
//
// Three degenerate cases collapse subsegments cleanly:
//   * v · j  <  a²  : aPeak < aMax, segments 2 and 6 vanish.
//   * 2 · D_accel(v, a, j) > D : no cruise (segment 4 vanishes), reduce vPeak.
//   * D very small : neither aMax nor vMax reached (segments 2, 4, 6 vanish).
// The plan() factory selects the right branch analytically; no iteration.
//
// The profile is normalised: sample(t) returns position(t) / D, so callers
// multiply by their own dimension delta the same way they do for the other
// profile shapes.
public:
	SCurveProfile( const SCurveProfile& )            = default;
	SCurveProfile& operator=( const SCurveProfile& ) = default;

	[[nodiscard]] static SCurveProfile plan( double distance,
											 double vMax,
											 double aMax,
											 double jMax ) noexcept;
	// Shortest-time 7-segment traversal of distance honouring vMax, aMax, jMax.
	// Degrades to fewer segments as constraints become non-binding (see class
	// comment). All four arguments must share consistent units, with jerk in
	// units/s³.

	[[nodiscard]] double sample( double elapsedSec ) const noexcept override;

private:
	SCurveProfile( double t1, double t2, double t4,
				   double jMag, double aPeak, double vPeak,
				   double distance ) noexcept;

	[[nodiscard]] double positionAccel( double t ) const noexcept;
	// Position reached at time t within the acceleration phase [0, 2·T1 + T2].
	// Pulled out so sample() can reuse it for the decel phase via symmetry
	// (s_decel(t) = D - s_accel(T_total - t)).

	double m_t1   = 0.0;  // jerk phase duration (segments 1, 3, 5, 7)
	double m_t2   = 0.0;  // const-accel duration (segments 2 and 6)
	double m_t4   = 0.0;  // cruise duration (segment 4)
	double m_j    = 0.0;  // jerk magnitude used in jerk phases
	double m_aPeak = 0.0; // peak |accel|; equals m_j · m_t1
	double m_vPeak = 0.0; // peak (cruise) velocity
	double m_d    = 0.0;  // physical distance, kept so sample() can normalise

	double m_taSec = 0.0; // cached 2·T1 + T2 (end of accel phase) for hot path
	double m_dAccel = 0.0; // cached position reached at end of accel phase
};

}  // namespace Motion
