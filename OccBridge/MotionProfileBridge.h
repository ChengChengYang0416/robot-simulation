#pragma once

namespace Motion {
	class Profile;
	class SingularitySpeedScaler;
}

using namespace System;

namespace OccBridge {

	public enum class SingularityLevel : int
	// Managed mirror of Motion::SingularityLevel. The numeric values must stay in
	// sync; the enum is public so HMI code can switch on the level without an
	// additional projection layer.
	{
		Normal   = 0,
		Warning  = 1,
		Critical = 2,
	};

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

		static MotionProfile^ CreateSCurve( double distance, double vMax, double aMax, double jMax );
		// Plans a jerk-limited 7-segment S-curve via SCurveProfile::plan(). The
		// native code degrades to fewer segments when aMax or vMax cannot be
		// reached, so callers do not need to detect those cases explicitly.

	private:
		MotionProfile( Motion::Profile* native );
		// Internal constructor used by the static factories; takes ownership of
		// the native pointer.

		Motion::Profile* m_pNative;
	};

	public ref class SpeedScaler
	{
	// Managed wrapper around Motion::SingularitySpeedScaler. One instance per
	// trajectory player (MoveL / MoveJ); the player calls Scale() every tick
	// with the latest manipulability ratio (typically min(wristRatio, armRatio)
	// from OccViewerControl::GetManipulability) and multiplies its wall-clock
	// dt by the returned factor before sampling the MotionProfile.
	public:
		SpeedScaler();
		// Uses the default thresholds (warn 0.05, crit 0.01) matching the native
		// SingularityMonitor defaults.

		SpeedScaler( double warnRatio, double critRatio );
		// Custom thresholds for callers that have tuned the scaler against a
		// specific robot. The constructor silently corrects an inverted pair so
		// invalid input still produces a usable ramp.

		~SpeedScaler();
		!SpeedScaler();

		property double WarnRatio { double get(); }
		property double CritRatio { double get(); }
		property SingularityLevel LastLevel { SingularityLevel get(); }
		// Severity classified at the most recent Scale() call. Useful for HMI
		// badge colouring without recomputing thresholds in C# code.

		void Reset();
		// Clears the "critical announced" latch; the player calls this when
		// (re)starting a motion so a fresh warning can fire on the next critical
		// crossing.

		double Scale( double ratio );
		// Returns the speed-scale factor for the given dimensionless ratio.
		// Range [0, 1]; 0 means the player should freeze the trajectory at the
		// current pose (and typically abort).

		bool ShouldAnnounceCritical();
		// True exactly once per critical episode; the player uses this to pop a
		// dialog or change status colour without flooding the UI thread every
		// tick.

		static double Combine( double wristRatio, double armRatio );
		// Convenience for the typical input: min(wristRatio, armRatio). Static
		// because callers that already hold both values can compute the
		// combined scalar before constructing a SpeedScaler instance.

	private:
		Motion::SingularitySpeedScaler* m_pNative;
	};

}  // namespace OccBridge
