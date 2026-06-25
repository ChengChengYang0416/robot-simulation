#include "CartesianJogStepper.h"

#include "Quaternion.h"

#include <cmath>

namespace OccBridge {

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

}  // namespace

CartesianJogStepper::CartesianJogStepper() noexcept = default;

CartesianJogStepper::CartesianJogStepper( double linSpeedMmPerSec, double angSpeedDegPerSec ) noexcept
	: m_linSpeedMmPerSec( linSpeedMmPerSec )
	, m_angSpeedDegPerSec( angSpeedDegPerSec )
{}

bool CartesianJogStepper::step( const double currentPose[ 6 ], int axisIndex, int dir, double dt,
								double outPose[ 6 ] ) const noexcept
{
	if( currentPose == nullptr || outPose == nullptr ) {
		return false;
	}
	if( axisIndex < 0 || axisIndex > 5 ) {
		return false;
	}
	if( dir != 1 && dir != -1 ) {
		return false;
	}

	// Start by copying the current pose; only the affected component(s) below
	// will be overwritten, so a partial result on early return still represents
	// a valid (unchanged) pose rather than garbage.
	for( int i = 0; i < 6; ++i ) {
		outPose[ i ] = currentPose[ i ];
	}

	if( axisIndex < 3 ) {
		// Position: bump the chosen world component directly. Workspace-limit
		// rejection is the IK's job — this stepper produces the *desired* pose
		// and lets the IK refuse if it's unreachable.
		outPose[ axisIndex ] = currentPose[ axisIndex ] + dir * m_linSpeedMmPerSec * dt;
		return true;
	}

	// Orientation: build a world-frame axis-angle delta and *left-multiply*
	// the current orientation quaternion. Euler addition would seem simpler
	// but accumulates drift any time two consecutive jogs span a non-trivial
	// arc (the ZYX order means Rx+ in Euler is not the same as a world-X
	// rotation once Ry or Rz are non-zero — quaternion composition handles
	// the basis rotation correctly).
	const double angleRad = dir * m_angSpeedDegPerSec * dt * kDegToRad;
	const int    axisBit  = axisIndex - 3;
	const double ax = ( axisBit == 0 ) ? 1.0 : 0.0;
	const double ay = ( axisBit == 1 ) ? 1.0 : 0.0;
	const double az = ( axisBit == 2 ) ? 1.0 : 0.0;

	const Quat qCurrent = Quaternion::fromZyxDeg( currentPose[ 3 ], currentPose[ 4 ], currentPose[ 5 ] );
	const Quat qDelta   = Quaternion::fromAxisAngleRad( ax, ay, az, angleRad );
	const Quat qNew     = Quaternion::normalise( Quaternion::multiply( qDelta, qCurrent ) );

	double rx = 0.0;
	double ry = 0.0;
	double rz = 0.0;
	Quaternion::toZyxDeg( qNew, rx, ry, rz );
	outPose[ 3 ] = rx;
	outPose[ 4 ] = ry;
	outPose[ 5 ] = rz;
	return true;
}

}  // namespace OccBridge
