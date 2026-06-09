#include "TcpPoseSolver.h"

#include <cmath>
#include "TransformBuilder.h"

namespace OccBridge {

std::array<double, 6> solveTcpPose( const gp_Trsf& t )
// Decomposes a TCP transform into translation + ZYX intrinsic Euler angles.
// Returns [x, y, z, rx, ry, rz]; translation in mm, rotations in degrees.
{
	std::array<double, 6> out{};

	// Translation: gp_Trsf::Value uses 1-based indices; column 4 is translation.
	out[ 0 ] = t.Value( 1, 4 );
	out[ 1 ] = t.Value( 2, 4 );
	out[ 2 ] = t.Value( 3, 4 );

	// Rotation matrix elements (row, col), 1-based.
	const double r00 = t.Value( 1, 1 );
	const double r10 = t.Value( 2, 1 );
	const double r20 = t.Value( 3, 1 );
	const double r21 = t.Value( 3, 2 );
	const double r22 = t.Value( 3, 3 );

	const double sy = std::sqrt( r00 * r00 + r10 * r10 );
	double rx, ry, rz;
	if( sy > Transform::kEpsilon ) {
		rx = std::atan2( r21, r22 );
		ry = std::atan2( -r20, sy );
		rz = std::atan2( r10, r00 );
	} else {
		// Gimbal lock: ry is +/- 90 deg, rz is set to 0 by convention.
		rx = std::atan2( -t.Value( 2, 3 ), t.Value( 2, 2 ) );
		ry = std::atan2( -r20, sy );
		rz = 0.0;
	}

	constexpr double kRadToDeg = 180.0 / Transform::kPi;
	out[ 3 ] = rx * kRadToDeg;
	out[ 4 ] = ry * kRadToDeg;
	out[ 5 ] = rz * kRadToDeg;
	return out;
}

}  // namespace OccBridge
