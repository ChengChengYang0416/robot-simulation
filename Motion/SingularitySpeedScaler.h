#pragma once

namespace Motion {

enum class SingularityLevel : int
// Severity gate consumed by trajectory players (MoveL / MoveJ). Mirrors the
// classification produced by Kinematics/SingularityMonitor so HMI code can
// pass the native value through without remapping.
{
	Normal   = 0,
	Warning  = 1,
	Critical = 2,
};

class SingularitySpeedScaler
{
// Closed-loop time-scaling policy for trajectory players. Each tick the player
// passes the latest dimensionless manipulability ratio and gets back a factor
// in [0, 1]; the player advances its virtual elapsed time by dt * factor
// instead of wall-clock dt. The factor is a piecewise linear ramp:
//
//   ratio >= warnRatio       => 1.0      (full speed)
//   ratio in [crit, warn)    => linear   (warnRatio -> 1, critRatio -> 0)
//   ratio <  critRatio       => 0.0      (frozen; player should abort)
//
// "ratio" is intended to be min(wristRatio, armRatio) from SingularityMonitor
// so a single threshold pair covers either subspace dropping rank. Callers
// that prefer a weighted combination compute their own scalar.
//
// Stop-on-critical decision is left to the player: this class only computes
// the scale factor and reports whether the most recent sample crossed *into*
// Critical, so the player can pop a single dialog per crossing instead of
// every tick.
//
// Zero heap allocation, no exceptions. Lives in Motion (not Kinematics)
// because the policy is part of "trajectory execution" — Motion already owns
// the velocity-profile family and is the natural home for runtime tempo
// adjustments. The class itself does not depend on Kinematics types; the
// ratio scalar fully encodes the singularity state.
public:
	explicit SingularitySpeedScaler( double warnRatio = 0.05,
									 double critRatio = 0.01 ) noexcept;
	// Defaults match SingularityMonitor's defaults (Warning at 5% of nominal
	// volume, Critical at 1%). A misconfigured pair where warn <= crit is
	// silently corrected to warn = crit + epsilon so the ramp stays valid.

	[[nodiscard]] double warnRatio() const noexcept { return m_warnRatio; }
	[[nodiscard]] double critRatio() const noexcept { return m_critRatio; }
	[[nodiscard]] SingularityLevel lastLevel() const noexcept { return m_lastLevel; }

	void reset() noexcept;
	// Clears the "critical announced" latch so a fresh motion can warn again
	// on the next critical crossing. Called by the player when (re)starting
	// MoveL / MoveJ.

	[[nodiscard]] double scale( double ratio ) noexcept;
	// Returns the speed-scale factor for the given ratio. Side effect: updates
	// lastLevel() so the player can read the classification without
	// recomputing thresholds.

	[[nodiscard]] bool shouldAnnounceCritical() noexcept;
	// Returns true exactly once per critical episode. The latch resets when
	// the metric recovers to Warning / Normal so a motion that grazes
	// critical, recovers, then re-enters can still trigger a second alert.

	[[nodiscard]] static double combine( double wristRatio,
										 double armRatio ) noexcept;
	// Convenience: ratio = min(wristRatio, armRatio). Static because callers
	// that already hold both values may want the combined scalar before
	// constructing a scaler instance.

private:
	double           m_warnRatio        = 0.05;
	double           m_critRatio        = 0.01;
	SingularityLevel m_lastLevel        = SingularityLevel::Normal;
	bool             m_criticalAnnounced = false;
};

}  // namespace Motion
