#pragma once

namespace Motion {
	class Profile;
}

using namespace System;

namespace OccBridge {

	public ref class MotionProfile
	{
	// Managed wrapper around Motion::Profile. The C# layer owns one of these per
	// active trajectory player (MoveL / MoveJ); construction goes through the
	// static factory methods so the polymorphic native type is never named on
	// the managed side. All numerical work happens in the native Motion library,
	// keeping motion-planning math independent of WPF / dispatcher timing and
	// free of OCCT and kinematics dependencies for unit testability.
	public:
		~MotionProfile();
		// Standard dispose pattern; delegates to the finalizer.

		!MotionProfile();
		// Releases the owned native Profile pointer.

		property double DurationSec {
			double get();
		}
		// Total motion duration in seconds. Mirrors Profile::durationSec().

		double Sample( double elapsedSec );
		// Returns s ∈ [0, 1] for the elapsed time. Forwards to Profile::sample().

		static MotionProfile^ CreateLinear( double durationSec );
		// Builds a LinearProfile (constant-velocity baseline).

		static MotionProfile^ CreateTrapezoidal( double distance, double vMax, double aMax );
		// Plans a TrapezoidalProfile via TrapezoidalProfile::plan(). The native
		// code degrades to a triangular profile when the distance is too short to
		// reach vMax, so callers do not need to detect that case explicitly.

	private:
		MotionProfile( Motion::Profile* native );
		// Internal constructor used by the static factories; takes ownership of
		// the native pointer.

		Motion::Profile* m_pNative;
	};

}  // namespace OccBridge
