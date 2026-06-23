#include "MotionProfileBridge.h"

#include "../Motion/MotionProfile.h"
#include "../Motion/LinearProfile.h"
#include "../Motion/TrapezoidalProfile.h"
#include "../Motion/SCurveProfile.h"
#include "../Motion/SingularitySpeedScaler.h"

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

}  // namespace OccBridge
