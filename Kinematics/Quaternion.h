#pragma once

namespace OccBridge {

struct Quat
// Hamilton-convention unit quaternion: q = w + x*i + y*j + z*k. No invariant is
// enforced by the struct itself; helpers below produce normalised results, but
// stored values may drift slightly and should be re-normalised after long chains.
{
	double w{ 1.0 };
	double x{ 0.0 };
	double y{ 0.0 };
	double z{ 0.0 };
};

namespace Quaternion {

[[nodiscard]] Quat fromZyxDeg( double rxDeg, double ryDeg, double rzDeg ) noexcept;
// Builds a unit quaternion from ZYX intrinsic Euler angles (R = Rz * Ry * Rx),
// the same convention used by TcpPoseSolver and RobotSceneFacade::solveTcpIk.
// Input in degrees so the caller does not need to convert at every call site.

void toZyxDeg( const Quat& q, double& outRxDeg, double& outRyDeg, double& outRzDeg ) noexcept;
// Inverse of fromZyxDeg. Returns three angles in degrees that, when fed back into
// fromZyxDeg, reproduce the same rotation (up to representation choice at the
// gimbal-lock singularity where rz is conventionally pinned to 0).

[[nodiscard]] Quat normalise( const Quat& q ) noexcept;
// Returns q / |q|. Falls back to identity (1,0,0,0) when |q| is below epsilon so
// callers never have to guard against division-by-zero from a degenerate input.

[[nodiscard]] double dot( const Quat& a, const Quat& b ) noexcept;
// Standard 4D inner product. Used by slerp to pick the short-rotation hemisphere
// (dot < 0 means q and -q represent the same rotation and we should flip one).

[[nodiscard]] Quat slerp( const Quat& q0, const Quat& q1, double t ) noexcept;
// Spherical linear interpolation between two unit quaternions, t in [0, 1]. The
// result is the rotation that traces the great-circle arc from q0 to q1 at
// constant angular velocity. Automatically negates q1 when dot(q0, q1) < 0 so
// the path is always the short way. Falls back to normalised lerp when q0 and
// q1 are nearly parallel (dot > 0.9995) so sin(theta) does not blow up.

[[nodiscard]] double angleBetweenRad( const Quat& a, const Quat& b ) noexcept;
// Returns the geodesic rotation angle in radians between two unit quaternions,
// i.e. 2 * acos(|dot(a, b)|). This is the true rotation angle (always >= 0,
// shortest-arc); useful as the "angular distance" of a MoveL segment when sizing
// a velocity profile so that the planned duration matches the actual arc length
// rather than a per-axis-max approximation.

}  // namespace Quaternion

}  // namespace OccBridge
