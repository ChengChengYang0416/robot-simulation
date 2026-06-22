#include "MotionProfileBridge.h"

#include "../Kinematics/MotionProfile.h"

namespace OccBridge {

MotionProfile::MotionProfile( OccBridge::Motion::Profile* native )
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
	auto* native = new OccBridge::Motion::LinearProfile( durationSec );
	return gcnew MotionProfile( native );
}

MotionProfile^ MotionProfile::CreateTrapezoidal( double distance, double vMax, double aMax )
// Plans a trapezoidal profile via the native factory and wraps it. The native
// plan() returns a value type, so we copy it onto the heap to satisfy the
// polymorphic-pointer ownership model.
{
	auto plan = OccBridge::Motion::TrapezoidalProfile::plan( distance, vMax, aMax );
	auto* native = new OccBridge::Motion::TrapezoidalProfile( plan );
	return gcnew MotionProfile( native );
}

}  // namespace OccBridge
