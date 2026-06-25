#include "MotionProfileBridge.h"

#include "../Motion/MotionProfile.h"
#include "../Motion/LinearProfile.h"
#include "../Motion/TrapezoidalProfile.h"
#include "../Motion/SCurveProfile.h"
#include "../Motion/SingularitySpeedScaler.h"
#include "../Kinematics/PoseInterpolator.h"
#include "../Kinematics/CartesianJogStepper.h"

namespace OccBridge {

MotionProfile::MotionProfile( Motion::Profile* native )
	: m_pNative( native )
// Takes ownership of the native pointer. The static factories are the only
// callers, and they always pass a heap-allocated subclass instance.
{
}

MotionProfile::~MotionProfile()
// Standard managed/native cleanup: destructor (Dispose) delegates to the
// finalizer so the native pointer is freed deterministically.
{
	this->!MotionProfile();
}

MotionProfile::!MotionProfile()
// Releases the native Profile via its virtual destructor.
{
	if( m_pNative != nullptr ) {
		delete m_pNative;
		m_pNative = nullptr;
	}
}

double MotionProfile::DurationSec::get()
// Forwards to the native accessor; returns 0 when the wrapper has been disposed.
{
	return ( m_pNative != nullptr ) ? m_pNative->durationSec() : 0.0;
}

double MotionProfile::Sample( double elapsedSec )
// Forwards to native Profile::sample(); returns 1 (motion complete) when the
// wrapper has been disposed, matching the saturated-end-of-trajectory contract.
{
	return ( m_pNative != nullptr ) ? m_pNative->sample( elapsedSec ) : 1.0;
}

MotionProfile^ MotionProfile::CreateLinear( double durationSec )
// Heap-allocates a native LinearProfile and hands ownership to a new managed
// wrapper. The native heap allocation is intentional: the managed finalizer
// owns the lifetime and we want polymorphic Profile pointers.
{
	auto* native = new Motion::LinearProfile( durationSec );
	return gcnew MotionProfile( native );
}

MotionProfile^ MotionProfile::CreateTrapezoidal( double distance, double vMax, double aMax )
// Plans a trapezoidal profile via the native factory and wraps it. The native
// plan() returns a value type, so we copy it onto the heap to satisfy the
// polymorphic-pointer ownership model.
{
	auto plan = Motion::TrapezoidalProfile::plan( distance, vMax, aMax );
	auto* native = new Motion::TrapezoidalProfile( plan );
	return gcnew MotionProfile( native );
}

MotionProfile^ MotionProfile::CreateSCurve( double distance, double vMax, double aMax, double jMax )
// Plans a jerk-limited S-curve via the native factory and wraps it. Same
// value-then-copy-to-heap pattern as CreateTrapezoidal so the managed wrapper
// always owns a polymorphic Profile*.
{
	auto plan = Motion::SCurveProfile::plan( distance, vMax, aMax, jMax );
	auto* native = new Motion::SCurveProfile( plan );
	return gcnew MotionProfile( native );
}

SpeedScaler::SpeedScaler()
	: m_pNative( new Motion::SingularitySpeedScaler() )
{
}

SpeedScaler::SpeedScaler( double warnRatio, double critRatio )
	: m_pNative( new Motion::SingularitySpeedScaler( warnRatio, critRatio ) )
{
}

SpeedScaler::~SpeedScaler()
{
	this->!SpeedScaler();
}

SpeedScaler::!SpeedScaler()
{
	if( m_pNative != nullptr ) {
		delete m_pNative;
		m_pNative = nullptr;
	}
}

double SpeedScaler::WarnRatio::get()
{
	return ( m_pNative != nullptr ) ? m_pNative->warnRatio() : 0.0;
}

double SpeedScaler::CritRatio::get()
{
	return ( m_pNative != nullptr ) ? m_pNative->critRatio() : 0.0;
}

SingularityLevel SpeedScaler::LastLevel::get()
// Cast through int because the managed and native enums share numeric values
// but live in separate namespaces; the static_cast is the cheapest projection.
{
	if( m_pNative == nullptr ) {
		return SingularityLevel::Normal;
	}
	return static_cast<SingularityLevel>( static_cast<int>( m_pNative->lastLevel() ) );
}

void SpeedScaler::Reset()
{
	if( m_pNative != nullptr ) {
		m_pNative->reset();
	}
}

double SpeedScaler::Scale( double ratio )
// Disposed wrapper falls back to "full speed" so a missing scaler never
// freezes a trajectory; the player can still observe LastLevel == Normal.
{
	return ( m_pNative != nullptr ) ? m_pNative->scale( ratio ) : 1.0;
}

bool SpeedScaler::ShouldAnnounceCritical()
{
	return ( m_pNative != nullptr ) ? m_pNative->shouldAnnounceCritical() : false;
}

double SpeedScaler::Combine( double wristRatio, double armRatio )
{
	return Motion::SingularitySpeedScaler::combine( wristRatio, armRatio );
}

// =================== PoseInterp (slerp wrapper) =================== //

PoseInterp::PoseInterp()
	: m_pNative( new ::OccBridge::PoseInterpolator() )
{}

PoseInterp::~PoseInterp()
{
	this->!PoseInterp();
}

PoseInterp::!PoseInterp()
{
	if( m_pNative != nullptr ) {
		delete m_pNative;
		m_pNative = nullptr;
	}
}

void PoseInterp::Begin( cli::array<double>^ startAbcDeg, cli::array<double>^ targetAbcDeg )
{
	// Defensive guards: HMI code in mid-refactor can pass null or short arrays;
	// silently ignore so the player can detect the no-op via TotalAngleDeg == 0.
	if( m_pNative == nullptr ) {
		return;
	}
	if( startAbcDeg == nullptr || targetAbcDeg == nullptr ) {
		return;
	}
	if( startAbcDeg->Length < 3 || targetAbcDeg->Length < 3 ) {
		return;
	}
	m_pNative->begin( startAbcDeg[ 0 ], startAbcDeg[ 1 ], startAbcDeg[ 2 ],
					  targetAbcDeg[ 0 ], targetAbcDeg[ 1 ], targetAbcDeg[ 2 ] );
}

void PoseInterp::Sample( double s, cli::array<double>^ outAbcDeg )
{
	if( m_pNative == nullptr ) {
		return;
	}
	if( outAbcDeg == nullptr || outAbcDeg->Length < 3 ) {
		return;
	}
	double rx = 0.0;
	double ry = 0.0;
	double rz = 0.0;
	m_pNative->sample( s, rx, ry, rz );
	outAbcDeg[ 0 ] = rx;
	outAbcDeg[ 1 ] = ry;
	outAbcDeg[ 2 ] = rz;
}

double PoseInterp::TotalAngleDeg::get()
{
	return ( m_pNative != nullptr ) ? m_pNative->totalAngleDeg() : 0.0;
}

// =================== CartesianJog (world-frame JOG stepper) =================== //

CartesianJog::CartesianJog()
	: m_pNative( new ::OccBridge::CartesianJogStepper() )
{}

CartesianJog::CartesianJog( double linSpeedMmPerSec, double angSpeedDegPerSec )
	: m_pNative( new ::OccBridge::CartesianJogStepper( linSpeedMmPerSec, angSpeedDegPerSec ) )
{}

CartesianJog::~CartesianJog()
{
	this->!CartesianJog();
}

CartesianJog::!CartesianJog()
{
	if( m_pNative != nullptr ) {
		delete m_pNative;
		m_pNative = nullptr;
	}
}

double CartesianJog::LinSpeedMmPerSec::get()
{
	return ( m_pNative != nullptr ) ? m_pNative->linSpeed() : 0.0;
}

void CartesianJog::LinSpeedMmPerSec::set( double value )
{
	if( m_pNative != nullptr ) {
		m_pNative->setLinSpeed( value );
	}
}

double CartesianJog::AngSpeedDegPerSec::get()
{
	return ( m_pNative != nullptr ) ? m_pNative->angSpeed() : 0.0;
}

void CartesianJog::AngSpeedDegPerSec::set( double value )
{
	if( m_pNative != nullptr ) {
		m_pNative->setAngSpeed( value );
	}
}

bool CartesianJog::Step( cli::array<double>^ currentPose, int axisIndex, int dir, double dt,
						 cli::array<double>^ outPose )
{
	if( m_pNative == nullptr ) {
		return false;
	}
	if( currentPose == nullptr || outPose == nullptr ) {
		return false;
	}
	if( currentPose->Length < 6 || outPose->Length < 6 ) {
		return false;
	}

	// Pin the managed arrays for the duration of the native call so the GC
	// cannot relocate them mid-step. The step itself is O(1) so the pin window
	// is microscopic and contention with concurrent GC is a non-issue.
	pin_ptr<double> pCur = &currentPose[ 0 ];
	pin_ptr<double> pOut = &outPose[ 0 ];
	return m_pNative->step( pCur, axisIndex, dir, dt, pOut );
}

}  // namespace OccBridge
