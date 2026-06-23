#pragma once

#include <array>
#include <gp_Trsf.hxx>

namespace OccBridge {

class RobotKinematics;

enum class IkStatus : int
{
	Converged            = 0,
	MaxIterations        = 1,
	InvalidConfiguration = 2,
	// Returned when kin has not been configured (no parts or axis map),
	// when the axis map does not cover all 6 axes, or when an axis maps
	// to an out-of-range part index.
};

struct IkOptions
{
	int    maxIterations         = 100;
	double positionToleranceMm   = 0.05;
	double orientationToleranceRad = 5.0e-4;   // ~0.029 deg
	double damping               = 0.05;       // DLS lambda; larger = more stable near singularities, slower convergence
	double maxStepDeg            = 5.0;        // per-axis clamp applied to each iteration's delta-q

	bool   useJointLimits        = false;
	std::array<double, 6> jointMinDeg = { -180.0, -180.0, -180.0, -180.0, -180.0, -180.0 };
	std::array<double, 6> jointMaxDeg = {  180.0,  180.0,  180.0,  180.0,  180.0,  180.0 };
};

struct IkResult
{
	IkStatus status                = IkStatus::InvalidConfiguration;
	int      iterations            = 0;
	double   positionErrorMm       = 0.0;
	double   orientationErrorRad   = 0.0;
	std::array<double, 6> jointAnglesDeg{};
};

[[nodiscard]] IkResult solveIkDls( RobotKinematics&             kin,
								   const gp_Trsf&               targetTcp,
								   const std::array<double, 6>& seedAnglesDeg,
								   const IkOptions&             options = {} );
// Damped Least Squares IK for a 6-DOF revolute arm.
//
// Algorithm (per iteration):
//   1. Apply current q to kin and delegate to Jacobian::build() (runs FK in
//      pre-allocated buffers and emits the 6x6 geometric Jacobian J).
//   2. Compose 6-vector pose error e = [target.p - p_tcp ; axis*angle(R_target * R_tcp^T)].
//   3. Solve (J*J^T + lambda^2 * I) * y = e via Gaussian elimination (6x6 SPD).
//   4. delta_q (rad) = J^T * y; clamp per-axis to maxStepDeg, convert to deg, accumulate.
//   5. Apply joint limits (if enabled) and repeat.
//
// Leaves kin's joint angles set to the final iterate (whether converged or not) and
// performs zero heap allocations in the iteration loop. Returns status + final pose
// error so callers can distinguish "reached target" from "stuck near a singularity".

}  // namespace OccBridge
