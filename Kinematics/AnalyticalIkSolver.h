#pragma once

#include "IkSolver.h"

namespace OccBridge {

[[nodiscard]] IkResult solveIkAnalytical( const RobotKinematics&       kin,
										  const gp_Trsf&               targetTcp,
										  const std::array<double, 6>& seedAnglesDeg,
										  const IkOptions&             options = {} );
// Closed-form (Pieper) IK for a 6-DOF arm with a spherical wrist.
//
// Algorithm:
//   1. Verify the geometric assumptions for closed-form decoupling:
//        a4 == 0, a5 == 0, a6 == 0, d5 == 0 (last three axes intersect at the
//        wrist centre). Returns InvalidConfiguration if the loaded DH params
//        do not satisfy them — the caller is expected to fall back to the
//        iterative DLS solver in that case.
//   2. Wrist centre:  WC = p_target - d6 · z6_target  (z6_target is the third
//      column of R_target).
//   3. q1 from the projection of WC on the base plane (two branches: forward
//      shoulder and back shoulder, the latter at q1 + π).
//   4. q2, q3 from a 2-link planar problem in the shoulder plane, with the
//      "second link" length L3 = sqrt(a3² + d4²) and an offset angle
//      ψ = atan2(a3, d4) that absorbs the elbow / wrist common-normal offset.
//      Two branches: elbow-up and elbow-down.
//   5. R_3^6 = (R_0^3)^T · R_target, decomposed as ZYZ Euler angles to
//      recover q4, q5, q6 (two branches via ±q5). When |sin q5| < ε the wrist
//      is in a gimbal singularity and only q4 + q6 is observable; the solver
//      keeps q4 at the seed and assigns the remainder to q6.
//   6. Up to 2 × 2 × 2 = 8 candidate solutions are filtered by joint limits
//      (when options.useJointLimits is true) and the one minimising the
//      cyclic-wrap distance to seedAnglesDeg is returned.
//
// Cost: O(1) trigonometric ops, no iteration, no heap. Robust against
// numerical jitter near workspace boundaries via clamped acos arguments.
// The options.maxIterations / damping / maxStepDeg fields are ignored;
// positionToleranceMm and orientationToleranceRad are only used to populate
// the returned IkResult error fields, not to drive convergence.

}  // namespace OccBridge
