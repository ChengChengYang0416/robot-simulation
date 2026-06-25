#include "PoseInterpolator.h"

namespace OccBridge {

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

}  // namespace

void PoseInterpolator::begin( double startRxDeg, double startRyDeg, double startRzDeg,
							  double targetRxDeg, double targetRyDeg, double targetRzDeg ) noexcept
{
	m_qStart   = Quaternion::fromZyxDeg( startRxDeg, startRyDeg, startRzDeg );
	m_qTarget  = Quaternion::fromZyxDeg( targetRxDeg, targetRyDeg, targetRzDeg );
	m_angleRad = Quaternion::angleBetweenRad( m_qStart, m_qTarget );
}

void PoseInterpolator::sample( double s, double& outRxDeg, double& outRyDeg, double& outRzDeg ) const noexcept
{
	// Clamp the progress fraction at the boundary; the player advances a
	// virtual clock that may step slightly past 1.0 on the last frame, and we
	// want that to land exactly on the target rather than overshoot.
	double sc = s;
	if( sc < 0.0 ) {
		sc = 0.0;
	} else if( sc > 1.0 ) {
		sc = 1.0;
	}

	const Quat q = Quaternion::slerp( m_qStart, m_qTarget, sc );
	Quaternion::toZyxDeg( q, outRxDeg, outRyDeg, outRzDeg );
}

double PoseInterpolator::totalAngleDeg() const noexcept
{
	return m_angleRad * kRadToDeg;
}

}  // namespace OccBridge
