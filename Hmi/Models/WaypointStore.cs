using System.Collections.ObjectModel;
using System.Globalization;

namespace Hmi.Models
{
	/// <summary>
	/// Owns the observable collection of taught waypoints plus the mutation ops
	/// the Teach tab UI needs (Add / RemoveAt / MoveUp / MoveDown / Clear / Rename
	/// / CaptureFromPose / NextAutoName).
	///
	/// No UI / no OCCT / no serialization here — 3.9 will bolt on Save/Load and
	/// 3.10's <c>WaypointSequencePlayer</c> only reads <see cref="Waypoints"/>.
	/// </summary>
	public sealed class WaypointStore
	{
		/// <summary>
		/// Live collection bound by the DataGrid in 3.7.2 and iterated by
		/// <c>WaypointSequencePlayer</c> in 3.10. Mutate through the store's
		/// methods; direct <see cref="ObservableCollection{T}"/> mutation is also
		/// legal (the store keeps no derived state).
		/// </summary>
		public ObservableCollection<Waypoint> Waypoints { get; } = new ObservableCollection<Waypoint>();

		public int Count => Waypoints.Count;

		/// <summary>Append a fully-constructed waypoint. No-op on <c>null</c>.</summary>
		public void Add( Waypoint wp )
		{
			if( wp == null ) return;
			Waypoints.Add( wp );
		}

		/// <summary>
		/// Snapshot the given TCP pose into a new waypoint (defaults: MotionType.L,
		/// V/A ratio = 1.0, blend = 0) and append. <paramref name="name"/> null → auto.
		/// Returns the new waypoint, or <c>null</c> if <paramref name="pose"/> is
		/// invalid (null or shorter than 6).
		/// </summary>
		public Waypoint CaptureFromPose( double[] pose, MotionType motion = MotionType.L, string name = null )
		{
			if( pose == null || pose.Length < Waypoint.PoseElementCount ) return null;
			var wp = new Waypoint {
				Name = string.IsNullOrEmpty( name ) ? NextAutoName() : name,
				Motion = motion
			};
			wp.SetPose( pose );
			Waypoints.Add( wp );
			return wp;
		}

		public bool RemoveAt( int index )
		{
			if( index < 0 || index >= Waypoints.Count ) return false;
			Waypoints.RemoveAt( index );
			return true;
		}

		/// <summary>Swap with the previous row. No-op at index 0 / out-of-range.</summary>
		public bool MoveUp( int index )
		{
			if( index <= 0 || index >= Waypoints.Count ) return false;
			Waypoints.Move( index, index - 1 );
			return true;
		}

		/// <summary>Swap with the next row. No-op at last row / out-of-range.</summary>
		public bool MoveDown( int index )
		{
			if( index < 0 || index >= Waypoints.Count - 1 ) return false;
			Waypoints.Move( index, index + 1 );
			return true;
		}

		public void Clear() => Waypoints.Clear();

		public bool Rename( int index, string newName )
		{
			if( index < 0 || index >= Waypoints.Count ) return false;
			Waypoints[ index ].Name = newName ?? string.Empty;
			return true;
		}

		/// <summary>
		/// Returns the next <c>P{N}</c> name where <c>N</c> is one greater than the
		/// max integer suffix currently in the list. Empty list → "P1". Non
		/// P-prefixed names are ignored, so operators can rename freely without
		/// tripping the auto-numberer.
		/// </summary>
		public string NextAutoName()
		{
			int maxN = 0;
			for( int i = 0; i < Waypoints.Count; i++ ) {
				var n = Waypoints[ i ].Name;
				if( string.IsNullOrEmpty( n ) || n[ 0 ] != 'P' ) continue;
				if( int.TryParse( n.Substring( 1 ), NumberStyles.Integer, CultureInfo.InvariantCulture, out int v ) && v > maxN ) {
					maxN = v;
				}
			}
			return "P" + ( maxN + 1 ).ToString( CultureInfo.InvariantCulture );
		}
	}
}
