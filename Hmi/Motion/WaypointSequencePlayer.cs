using System;
using System.Collections.Generic;
using Hmi.Models;

namespace Hmi.Motion
{
	/// <summary>Life-cycle state of a <see cref="WaypointSequencePlayer"/>.</summary>
	public enum PlayerState
	{
		Idle,
		Running,
		Paused,
		Completed,
		Aborted
	}

	/// <summary>
	/// Stop-at-corner waypoint sequence player. Walks a snapshot of the input
	/// waypoints in order, dispatching one segment at a time to the injected
	/// <see cref="IWaypointSegmentExecutor"/> and advancing on
	/// <c>SegmentFinished(true)</c>. Sprint #5 (=roadmap 3.11) will replace
	/// stop-at-corner with blending; this player's public API stays the same.
	///
	/// UI-thread only. All methods must be called on the WPF dispatcher thread,
	/// since the executor's DispatcherTimer runs there and events fire there.
	/// </summary>
	public sealed class WaypointSequencePlayer
	{
		public const double MinSpeedOverride = Waypoint.MinRatio;
		public const double MaxSpeedOverride = Waypoint.MaxRatio;

		private readonly IWaypointSegmentExecutor _executor;

		// Queue snapshot taken at Start()/Step() time so mid-run edits to the
		// source store don't reorder rows under us. Waypoint objects are shared
		// by reference — edits *ahead* of the current index take effect on
		// dispatch of that segment; edits to already-dispatched segments do not
		// affect the in-flight motion (kernel copies pose on dispatch).
		private List<Waypoint> _queue;
		private int _index;
		private bool _pauseRequested;
		private bool _stepOnly;

		private PlayerState _state = PlayerState.Idle;
		private bool _loop;
		private double _speedOverride = 1.0;

		/// <summary>Fires on every state transition; UI uses it to refresh button enables.</summary>
		public event Action<PlayerState> StateChanged;

		public WaypointSequencePlayer( IWaypointSegmentExecutor executor )
		{
			if( executor == null ) throw new ArgumentNullException( nameof( executor ) );
			_executor = executor;
			_executor.SegmentFinished += OnSegmentFinished;
		}

		public PlayerState State => _state;
		public bool Loop { get => _loop; set => _loop = value; }
		public int CurrentIndex => _index;
		public int QueueCount => _queue?.Count ?? 0;

		/// <summary>
		/// Speed override multiplier composed with each waypoint's own VRatio at
		/// dispatch time (clamped into <see cref="Waypoint.MinRatio"/> …
		/// <see cref="Waypoint.MaxRatio"/>). Live edits apply to the *next*
		/// segment; the currently running segment keeps its planned profile.
		/// </summary>
		public double SpeedOverride {
			get => _speedOverride;
			set => _speedOverride = Clamp( value, MinSpeedOverride, MaxSpeedOverride );
		}

		/// <summary>
		/// Start walking the queue from index 0. Snapshots the waypoint list so
		/// mid-run reorder / delete doesn't perturb sequence ordering. No-op if
		/// already <see cref="PlayerState.Running"/> / <see cref="PlayerState.Paused"/>
		/// (call <see cref="Stop"/> first).
		/// </summary>
		public bool Start( IReadOnlyList<Waypoint> waypoints )
		{
			if( _state == PlayerState.Running || _state == PlayerState.Paused ) return false;
			if( waypoints == null || waypoints.Count == 0 ) return false;
			_queue = new List<Waypoint>( waypoints );
			_index = 0;
			_pauseRequested = false;
			_stepOnly = false;
			return DispatchCurrent();
		}

		/// <summary>
		/// Single-step: dispatch one segment then transition to
		/// <see cref="PlayerState.Paused"/>. From Idle/Completed/Aborted starts
		/// fresh at index 0; from Paused advances one more.
		/// </summary>
		public bool Step( IReadOnlyList<Waypoint> waypoints )
		{
			if( _state == PlayerState.Running ) return false;
			if( _state == PlayerState.Paused ) {
				_stepOnly = true;
				_pauseRequested = false;
				return DispatchCurrent();
			}
			if( waypoints == null || waypoints.Count == 0 ) return false;
			_queue = new List<Waypoint>( waypoints );
			_index = 0;
			_stepOnly = true;
			_pauseRequested = false;
			return DispatchCurrent();
		}

		/// <summary>
		/// Request pause at end of current segment. No-op unless
		/// <see cref="PlayerState.Running"/>; the running segment continues to
		/// completion, then the player transitions to Paused instead of
		/// dispatching the next segment.
		/// </summary>
		public void Pause()
		{
			if( _state == PlayerState.Running ) _pauseRequested = true;
		}

		/// <summary>Resume from Paused; dispatches the pending next segment.</summary>
		public bool Resume()
		{
			if( _state != PlayerState.Paused ) return false;
			_pauseRequested = false;
			_stepOnly = false;
			return DispatchCurrent();
		}

		/// <summary>
		/// Abort. From Running: state → Aborted immediately, executor Cancel()
		/// fires <c>SegmentFinished(false)</c> which lands in
		/// <see cref="OnSegmentFinished"/> and is ignored (state already
		/// Aborted). From Paused: state → Aborted, no executor call needed.
		/// </summary>
		public void Stop()
		{
			if( _state != PlayerState.Running && _state != PlayerState.Paused ) return;
			bool wasRunning = _state == PlayerState.Running;
			_state = PlayerState.Aborted;
			RaiseStateChanged();
			if( wasRunning ) _executor.Cancel();
		}

		/// <summary>Reset from any terminal state back to Idle. No-op on Running/Paused.</summary>
		public void Reset()
		{
			if( _state == PlayerState.Running || _state == PlayerState.Paused ) return;
			_queue = null;
			_index = 0;
			_pauseRequested = false;
			_stepOnly = false;
			if( _state != PlayerState.Idle ) {
				_state = PlayerState.Idle;
				RaiseStateChanged();
			}
		}

		// -------------------------------------------------------------------
		// Internals
		// -------------------------------------------------------------------

		private bool DispatchCurrent()
		{
			if( _queue == null || _index >= _queue.Count ) return false;
			var wp = _queue[ _index ];
			double vRatio = Clamp( wp.VRatio * _speedOverride, Waypoint.MinRatio, Waypoint.MaxRatio );
			string label = $"Seq {_index + 1}/{_queue.Count} → {wp.Name}";

			bool started = wp.Motion == MotionType.J
				? _executor.ExecuteMoveJ( wp.PoseXyzAbc, vRatio, label )
				: _executor.ExecuteMoveL( wp.PoseXyzAbc, vRatio, label );

			if( !started ) {
				// Executor rejected the segment (no robot, IK fail, degenerate
				// zero-length segment). Skip and try the next one so a queue
				// with a repeated / already-at-target waypoint doesn't stall.
				_index++;
				if( _index < _queue.Count ) return DispatchCurrent();
				if( _loop ) { _index = 0; return DispatchCurrent(); }
				_state = PlayerState.Completed;
				RaiseStateChanged();
				return false;
			}

			if( _state != PlayerState.Running ) {
				_state = PlayerState.Running;
				RaiseStateChanged();
			}
			return true;
		}

		private void OnSegmentFinished( bool ok )
		{
			// Ignore executor events when we're not actively driving it. Manual
			// Start button clicks and the user Stop-Motion button also flow
			// through the shared executor and would otherwise perturb the
			// sequence state machine.
			if( _state != PlayerState.Running ) return;

			if( !ok ) {
				_state = PlayerState.Aborted;
				RaiseStateChanged();
				return;
			}

			_index++;
			bool endOfQueue = _index >= _queue.Count;
			bool pauseNow = _pauseRequested || _stepOnly;
			_pauseRequested = false;
			_stepOnly = false;

			if( endOfQueue ) {
				if( !_loop ) {
					_state = PlayerState.Completed;
					RaiseStateChanged();
					return;
				}
				_index = 0;
			}

			if( pauseNow ) {
				_state = PlayerState.Paused;
				RaiseStateChanged();
				return;
			}
			DispatchCurrent();
		}

		private void RaiseStateChanged() => StateChanged?.Invoke( _state );

		private static double Clamp( double v, double lo, double hi )
		{
			if( v < lo ) return lo;
			if( v > hi ) return hi;
			return v;
		}
	}
}
