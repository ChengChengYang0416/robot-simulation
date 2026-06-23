#pragma once

namespace OccBridge {

class RobotKinematics;

enum class SingularityKind : int
// What kind of degeneracy the arm is in. For 6-DOF arms with a spherical wrist
// (Pieper geometry) the Jacobian factors into an arm-block (cols 0..2) and a
// wrist-block (cols 3..5); the kind is derived from whichever sub-block lost
// rank first. Combined is reported when both are below threshold.
{
	None     = 0,
	Wrist    = 1,   // joints 4-6 axes (near-)colinear; typically |sin q5| small
	Elbow    = 2,   // joints 2-3 reach a full-extension / full-retraction line
	Shoulder = 3,   // wrist centre on z0 axis: q1 unobservable
	Combined = 4,   // arm + wrist both degenerate at the same time
};

enum class SingularityLevel : int
// Severity gate for higher-level motion planners (Auto / MoveL speed scaling).
{
	Normal   = 0,
	Warning  = 1,
	Critical = 2,
};

struct SingularityThresholds
// All thresholds are dimensionless ratios so they transfer across robots of
// different sizes. Defaults chosen empirically for the LA series; callers
// (e.g. HMI Auto-mode) can override per-robot.
{
	// Wrist: ratio |det(J_w_block)| / 1.0. For a spherical wrist this is
	// approximately |sin(q5)| (see SingularityMonitor.cpp derivation note),
	// so 0.05 ~= 2.9 deg from full alignment, 0.01 ~= 0.6 deg.
	double wristWarn     = 0.05;
	double wristCritical = 0.01;

	// Arm: ratio |det(J_arm_block)| / L_ref^3, where L_ref is the sum of
	// reach-determining link lengths (a1 + a2 + L3). For LA906 L_ref ~= 910 mm,
	// so a 0.05 ratio means the position sub-volume collapsed below ~5% of its
	// nominal value.
	double armWarn     = 0.05;
	double armCritical = 0.01;
};

struct SingularityReport
// Snapshot of the manipulability at the current joint state. Values are raw
// (un-normalised); the levels and kind use the configured thresholds.
{
	double manipulability         = 0.0;   // |det(J)|
	double wristManipulability    = 0.0;   // |det(J[3:6, 3:6])|
	double armManipulability      = 0.0;   // |det(J[0:3, 0:3])|
	double wristRatio             = 0.0;   // wristManipulability / 1.0
	double armRatio               = 0.0;   // armManipulability / L_ref^3
	SingularityKind  kind         = SingularityKind::None;
	SingularityLevel level        = SingularityLevel::Normal;
	bool             valid        = false; // false when kin is not configured
};

[[nodiscard]] SingularityReport evaluate( RobotKinematics&             kin,
										  const SingularityThresholds& thresholds = {} ) noexcept;
// Computes Jacobian-based singularity metrics for the current joint state.
// Mutates kin only via computeCumulative() (recomputes the FK cache); joint
// angles are not touched. Returns a report with valid=false if kin is not
// configured.

}  // namespace OccBridge
