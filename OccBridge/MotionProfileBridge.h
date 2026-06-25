#pragma once

namespace Motion {
	class Profile;
	class SingularitySpeedScaler;
}

namespace OccBridge {
	class PoseInterpolator;
	class CartesianJogStepper;
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

	public ref class PoseInterp
	{
	// Managed wrapper around OccBridge::PoseInterpolator (native Kinematics lib).
	// One instance per trajectory player (typically MoveL); Begin() seeds the
	// segment endpoints once and Sample() returns the slerp-interpolated ZYX
	// Euler angles at the given progress fraction. Replaces the old element-wise
	// ABC lerp in MoveLTimer_Tick so long-distance orientation changes follow
	// the geodesic great-circle arc with constant angular velocity instead of
	// drifting off the shortest path.
	//
	// Managed name shortened to avoid a name clash with the native class, which
	// lives in the same `OccBridge` namespace as this ref class.
	public:
		PoseInterp();

		~PoseInterp();
		!PoseInterp();

		void Begin( cli::array<double>^ startAbcDeg, cli::array<double>^ targetAbcDeg );
		// Both arrays must contain at least 3 elements [rx, ry, rz] in degrees.
		// Silently does nothing for null or short input so a half-initialised
		// MoveL player does not crash the UI thread; the player should check
		// TotalAngleDeg afterwards to detect the no-op case.

		void Sample( double s, cli::array<double>^ outAbcDeg );
		// Fills outAbcDeg[0..2] with the interpolated [rx, ry, rz] in degrees.
		// Out-of-range s is clamped to [0, 1] inside the native code so the
		// caller can pass a slightly-over-1.0 virtual clock value safely.

		property double TotalAngleDeg { double get(); }
		// True geodesic rotation angle between the begin endpoints, in degrees.
		// Use as the angular-distance argument when planning a motion profile so
		// the duration matches the actual arc length rather than a per-axis-max
		// approximation.

	private:
		::OccBridge::PoseInterpolator* m_pNative;
	};

	public ref class CartesianJog
	{
	// Managed wrapper around OccBridge::CartesianJogStepper (native Kinematics lib).
	// One instance lives in MainWindow for the duration of the session; the JOG
	// timer tick calls Step() with the latest TCP pose and held button to get the
	// next desired pose, then routes it through SolveTcpIk. Speed caps are
	// settable properties so an HMI speed-multiplier slider can rescale on the
	// fly without re-creating the wrapper.
	//
	// Managed name shortened (Jog vs JogStepper) to match the existing PoseInterp
	// / SpeedScaler abbreviation pattern in this bridge.
	public:
		CartesianJog();
		CartesianJog( double linSpeedMmPerSec, double angSpeedDegPerSec );

		~CartesianJog();
		!CartesianJog();

		property double LinSpeedMmPerSec  { double get(); void set( double value ); }
		property double AngSpeedDegPerSec { double get(); void set( double value ); }

		bool Step( cli::array<double>^ currentPose, int axisIndex, int dir, double dt,
				   cli::array<double>^ outPose );
		// currentPose / outPose must each contain at least 6 elements
		// [x, y, z, rx, ry, rz] in mm and degrees. axisIndex 0..2 → world XYZ
		// translation, 3..5 → world Rx/Ry/Rz rotation. dir must be ±1. Returns
		// false (and leaves outPose untouched) on null / short / out-of-range
		// input so the caller can use the return as a "stop jog" signal.

	private:
		::OccBridge::CartesianJogStepper* m_pNative;
	};

}  // namespace OccBridge
