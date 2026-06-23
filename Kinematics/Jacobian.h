#pragma once

#include <array>

namespace OccBridge {

class RobotKinematics;

namespace Jacobian {

struct Matrix6x6
// Plain 6x6 storage for the geometric Jacobian; rows 0..2 are the linear-velocity
// block (mm / rad), rows 3..5 are the angular-velocity block (1 / rad). Columns
// correspond to axes 0..5 of RobotKinematics in the same order.
{
	double m[ 6 ][ 6 ]{};
};

[[nodiscard]] bool build( RobotKinematics& kin, Matrix6x6& outJ ) noexcept;
// Computes the 6x6 geometric Jacobian at the kin's current joint state. Calls
// kin.computeCumulative() internally so the caller does not need to pre-stage FK.
// Returns false when kin has no parts, the axis map is incomplete, or any axis
// maps to an out-of-range part index.
//
// Convention (standard DH with revolute joints):
//   J_v[:, i] = z_i x (p_tcp - p_i)
//   J_w[:, i] = z_i
// where z_i and p_i are the +Z direction and origin of the parent frame of the
// part driven by axis i, all expressed in world coordinates.

[[nodiscard]] double determinant( const Matrix6x6& J ) noexcept;
// Returns det(J) via LU with partial pivoting (O(n^3), no heap). Sign included.

[[nodiscard]] double manipulability( const Matrix6x6& J ) noexcept;
// Yoshikawa's manipulability index for a square Jacobian: sqrt(det(J * J^T))
// reduces to |det(J)|. Drops to 0 at any kinematic singularity.

[[nodiscard]] double subDeterminant3x3( const Matrix6x6& J,
										int              row0,
										int              col0 ) noexcept;
// Determinant of the 3x3 sub-block starting at (row0, col0). Used by the
// singularity monitor to isolate arm vs wrist sub-spaces.

}  // namespace Jacobian

}  // namespace OccBridge
