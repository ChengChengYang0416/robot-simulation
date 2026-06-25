#include "Quaternion.h"

#include <cmath>

namespace OccBridge {
namespace Quaternion {

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kEpsilon  = 1.0e-12;

// dot threshold above which slerp degenerates into a normalised linear blend;
// at this point sin(theta) underflows and the great-circle arc differs from the
// straight chord by less than ~1e-4 rad, far below any robot-control tolerance.
constexpr double kSlerpLinearThreshold = 0.9995;

}  // namespace

Quat fromZyxDeg( double rxDeg, double ryDeg, double rzDeg ) noexcept
// Derivation: q = qz * qy * qx (Hamilton product), where qa = (cos(a/2),
// sin(a/2) along the corresponding axis). Expanding the triple product gives
// the closed-form below, identical to the ZYX intrinsic / XYZ extrinsic form
// found in any quaternion textbook. Mirrors the matrix convention used by
// TcpPoseSolver so a round-trip ABC -> Quat -> ABC is bit-for-bit stable.
{
	const double rxHalf = 0.5 * rxDeg * kDegToRad;
	const double ryHalf = 0.5 * ryDeg * kDegToRad;
	const double rzHalf = 0.5 * rzDeg * kDegToRad;

	const double cx = std::cos( rxHalf );
	const double sx = std::sin( rxHalf );
	const double cy = std::cos( ryHalf );
	const double sy = std::sin( ryHalf );
	const double cz = std::cos( rzHalf );
	const double sz = std::sin( rzHalf );

	Quat q;
	q.w = cz * cy * cx + sz * sy * sx;
	q.x = cz * cy * sx - sz * cx * sy;
	q.y = cz * cx * sy + sz * cy * sx;
	q.z = sz * cy * cx - cz * sy * sx;
	return q;
}

void toZyxDeg( const Quat& q, double& outRxDeg, double& outRyDeg, double& outRzDeg ) noexcept
// Goes via the rotation matrix R = Rz * Ry * Rx (only the four entries needed
// for the atan2 extraction are computed) so the algorithm matches TcpPoseSolver
// element-for-element. Gimbal lock (|sy_proj| < eps) pins rz to 0 by convention
// and recovers rx from a different matrix column, again matching TcpPoseSolver.
{
	const double r00 = 1.0 - 2.0 * ( q.y * q.y + q.z * q.z );
	const double r10 = 2.0 * ( q.x * q.y + q.w * q.z );
	const double r20 = 2.0 * ( q.x * q.z - q.w * q.y );
	const double r21 = 2.0 * ( q.y * q.z + q.w * q.x );
	const double r22 = 1.0 - 2.0 * ( q.x * q.x + q.y * q.y );

	// Extra entries only needed in the gimbal-lock branch.
	const double r11 = 1.0 - 2.0 * ( q.x * q.x + q.z * q.z );
	const double r12 = 2.0 * ( q.y * q.z - q.w * q.x );

	const double syProj = std::sqrt( r00 * r00 + r10 * r10 );

	double rxRad;
	double ryRad;
	double rzRad;
	if( syProj > kEpsilon ) {
		rxRad = std::atan2( r21, r22 );
		ryRad = std::atan2( -r20, syProj );
		rzRad = std::atan2( r10, r00 );
	} else {
		rxRad = std::atan2( -r12, r11 );
		ryRad = std::atan2( -r20, syProj );
		rzRad = 0.0;
	}

	outRxDeg = rxRad * kRadToDeg;
	outRyDeg = ryRad * kRadToDeg;
	outRzDeg = rzRad * kRadToDeg;
}

Quat normalise( const Quat& q ) noexcept
{
	const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	if( n2 < kEpsilon ) {
		return Quat{};  // Identity rotation: safe fallback for degenerate input.
	}
	const double inv = 1.0 / std::sqrt( n2 );
	return Quat{ q.w * inv, q.x * inv, q.y * inv, q.z * inv };
}

double dot( const Quat& a, const Quat& b ) noexcept
{
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Quat slerp( const Quat& q0, const Quat& q1, double t ) noexcept
{
	// Pick the short-rotation hemisphere: q and -q encode the same rotation but
	// only one of the two lerp/slerp paths is the short way. Without this flip
	// a 359-degree input would orbit the long way around.
	Quat   target = q1;
	double cosTheta = dot( q0, q1 );
	if( cosTheta < 0.0 ) {
		target.w = -target.w;
		target.x = -target.x;
		target.y = -target.y;
		target.z = -target.z;
		cosTheta = -cosTheta;
	}

	if( cosTheta > kSlerpLinearThreshold ) {
		// Nearly parallel: sin(theta) loses precision so fall back to a
		// normalised linear blend. The path length error is below 1e-4 rad
		// in this regime, well below any control-loop tolerance.
		Quat   r{};
		r.w = q0.w + t * ( target.w - q0.w );
		r.x = q0.x + t * ( target.x - q0.x );
		r.y = q0.y + t * ( target.y - q0.y );
		r.z = q0.z + t * ( target.z - q0.z );
		return normalise( r );
	}

	const double theta    = std::acos( cosTheta );
	const double sinTheta = std::sin( theta );
	const double s0       = std::sin( ( 1.0 - t ) * theta ) / sinTheta;
	const double s1       = std::sin( t * theta ) / sinTheta;

	Quat r{};
	r.w = s0 * q0.w + s1 * target.w;
	r.x = s0 * q0.x + s1 * target.x;
	r.y = s0 * q0.y + s1 * target.y;
	r.z = s0 * q0.z + s1 * target.z;
	return r;
}

double angleBetweenRad( const Quat& a, const Quat& b ) noexcept
{
	// Absolute value because q and -q describe the same rotation; without it
	// the angle would be ambiguous in (0, pi) vs (pi, 2*pi).
	double cosTheta = std::fabs( dot( a, b ) );
	if( cosTheta > 1.0 ) {
		cosTheta = 1.0;  // Guard against fp drift just above 1.0 feeding acos.
	}
	return 2.0 * std::acos( cosTheta );
}

}  // namespace Quaternion
}  // namespace OccBridge
