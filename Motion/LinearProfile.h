#pragma once

#include "MotionProfile.h"

namespace Motion {

class LinearProfile final : public Profile
{
// Constant-velocity profile: s(t) = t / T. Equivalent to the original
// MoveL / MoveJ behaviour before velocity planning was introduced; kept as a
// drop-in fallback and as a baseline for verifying the trapezoidal numerics.
public:
	explicit LinearProfile( double durationSec ) noexcept;

	LinearProfile( const LinearProfile& )            = default;
	LinearProfile& operator=( const LinearProfile& ) = default;

	[[nodiscard]] double sample( double elapsedSec ) const noexcept override;
};

}  // namespace Motion
