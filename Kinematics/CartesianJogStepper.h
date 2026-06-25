#pragma once

namespace OccBridge {

class CartesianJogStepper
// Open-loop Cartesian jog stepper for Manual-mode World-frame JOG. Given the
// current TCP pose [x, y, z, rx, ry, rz] (mm, deg, ZYX-intrinsic Euler — the
// project-wide convention) and one held jog button (axisIndex 0..5, dir ±1),
// produces the next TCP pose by integrating the configured linear or angular
// speed over one timer tick. Position axes are a trivial Cartesian bump;
// orientation axes are composed via *world-frame quaternion multiplication*,
// not Euler addition, so accumulated drift / gimbal-lock pathologies that
// would corrupt long-running JOG sessions never appear.
//
// Statefulness is limited to the two configurable speed caps; the step
// function itself is pure (no internal pose memory — caller threads the
// current pose in each tick), so the class is safe to reuse across sessions
// and free of ordering hazards.
{
public:
	CartesianJogStepper() noexcept;
	CartesianJogStepper( double linSpeedMmPerSec, double angSpeedDegPerSec ) noexcept;

	[[nodiscard]] double linSpeed() const noexcept { return m_linSpeedMmPerSec; }
	[[nodiscard]] double angSpeed() const noexcept { return m_angSpeedDegPerSec; }
	void setLinSpeed( double mmPerSec )  noexcept { m_linSpeedMmPerSec  = mmPerSec; }
	void setAngSpeed( double degPerSec ) noexcept { m_angSpeedDegPerSec = degPerSec; }

	[[nodiscard]] bool step( const double currentPose[ 6 ], int axisIndex, int dir, double dt,
							  double outPose[ 6 ] ) const noexcept;
	// axisIndex 0..2  → world X / Y / Z translation in mm.
	// axisIndex 3..5  → world Rx / Ry / Rz rotation in deg (right-handed, fixed
	//                   world axes; the wrist re-orients relative to the world
	//                   not to the tool — for tool-local JOG swap the quaternion
	//                   composition order, see Quaternion::multiply doc).
	// dir is +1 or -1; any other value or out-of-range axisIndex returns false
	// and leaves outPose untouched. Sub-millisecond dt is honoured (no internal
	// clamping) so callers can drive this from any timer resolution.

private:
	double m_linSpeedMmPerSec{ 50.0 };
	double m_angSpeedDegPerSec{ 20.0 };
};

}  // namespace OccBridge
