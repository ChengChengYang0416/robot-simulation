using System;

namespace Hmi.Motion
{
	/// <summary>
	/// Thin abstraction between <see cref="WaypointSequencePlayer"/> and the
	/// concrete MoveL / MoveJ kernels (currently housed on MainWindow). The
	/// player never talks to OccBridge / DispatcherTimer directly; it asks the
	/// executor to run one segment and awaits <see cref="SegmentFinished"/>.
	///
	/// All members are called on the WPF dispatcher thread;
	/// <see cref="SegmentFinished"/> fires on the dispatcher thread too, so the
	/// player's handler can safely re-enter and dispatch the next segment.
	/// </summary>
	public interface IWaypointSegmentExecutor
	{
		/// <summary>
		/// Dispatch a Cartesian straight-line segment to <paramref name="targetPose"/>.
		/// Returns true when a segment actually started (<see cref="SegmentFinished"/>
		/// will fire exactly once); false when rejected (no robot loaded, already at
		/// target, invalid pose) — no event fires in the reject case, so the caller
		/// must handle "skip to next" itself.
		/// </summary>
		bool ExecuteMoveL( double[] targetPose, double vRatio, string statusLabel );

		/// <summary>
		/// Joint-space variant. Executor is responsible for the IK from
		/// <paramref name="targetPose"/> to joint angles before slewing.
		/// </summary>
		bool ExecuteMoveJ( double[] targetPose, double vRatio, string statusLabel );

		/// <summary>
		/// Abort any in-flight segment. If a segment was live,
		/// <see cref="SegmentFinished"/> fires once with <c>ok=false</c>; if not,
		/// the call is a silent no-op.
		/// </summary>
		void Cancel();

		/// <summary>
		/// Fires once per successfully-dispatched segment. Bool argument:
		/// <c>true</c> when the segment reached its endpoint; <c>false</c> when
		/// aborted (user Stop, singularity abort, external cancel).
		/// </summary>
		event Action<bool> SegmentFinished;
	}
}
