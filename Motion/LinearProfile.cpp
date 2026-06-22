#include "LinearProfile.h"

#include <algorithm>

namespace Motion {

LinearProfile::LinearProfile( double durationSec ) noexcept
{
	m_durationSec = std::max( 0.0, durationSec );
}

double LinearProfile::sample( double elapsedSec ) const noexcept
{
	if( m_durationSec <= 0.0 ) {
		return 1.0;
	}
	if( elapsedSec <= 0.0 ) {
		return 0.0;
	}
	if( elapsedSec >= m_durationSec ) {
		return 1.0;
	}
	return elapsedSec / m_durationSec;
}

}  // namespace Motion
