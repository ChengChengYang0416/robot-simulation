#include "SingularitySpeedScaler.h"

#include <algorithm>

namespace Motion {

SingularitySpeedScaler::SingularitySpeedScaler( double warnRatio,
												double critRatio ) noexcept
	: m_warnRatio( warnRatio )
	, m_critRatio( critRatio )
{
	// Guard against an inverted pair: a misconfigured caller with warn <= crit
	// would produce negative scale factors. Bump warn just above crit so the
	// ramp degenerates to a step instead of going negative.
	if( m_warnRatio < m_critRatio ) {
		m_warnRatio = m_critRatio + 1.0e-6;
	}
}

void SingularitySpeedScaler::reset() noexcept
{
	m_lastLevel        = SingularityLevel::Normal;
	m_criticalAnnounced = false;
}

double SingularitySpeedScaler::scale( double ratio ) noexcept
// Piecewise-linear ramp; std::clamp not used because the three branches each
// also update m_lastLevel which the player consumes for UI badge colour.
{
	if( ratio >= m_warnRatio ) {
		m_lastLevel = SingularityLevel::Normal;
		return 1.0;
	}
	if( ratio <= m_critRatio ) {
		m_lastLevel = SingularityLevel::Critical;
		return 0.0;
	}
	m_lastLevel = SingularityLevel::Warning;
	const double span = m_warnRatio - m_critRatio;
	return ( ratio - m_critRatio ) / span;
}

bool SingularitySpeedScaler::shouldAnnounceCritical() noexcept
// One-shot latch: true on the first poll that finds lastLevel == Critical,
// false on subsequent polls until the metric recovers (resetting the latch).
{
	if( m_lastLevel != SingularityLevel::Critical ) {
		m_criticalAnnounced = false;
		return false;
	}
	if( m_criticalAnnounced ) {
		return false;
	}
	m_criticalAnnounced = true;
	return true;
}

double SingularitySpeedScaler::combine( double wristRatio, double armRatio ) noexcept
{
	return std::min( wristRatio, armRatio );
}

}  // namespace Motion
