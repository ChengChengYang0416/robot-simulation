using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Hmi.Models
{
	/// <summary>
	/// Motion segment type between waypoints.
	///   L = MoveL (Cartesian straight-line TCP interpolation)
	///   J = MoveJ (joint-space slew)
	/// </summary>
	public enum MotionType
	{
		L = 0,
		J = 1
	}

	/// <summary>
	/// One taught pose in the teach-pendant queue.
	///
	/// Pure data holder — depends only on <see cref="INotifyPropertyChanged"/> so
	/// 3.7.2's DataGrid cell edits round-trip into the store. No OCCT / no WPF.
	///
	/// PoseXyzAbc layout matches the OccBridge / MainWindow convention returned by
	/// <c>_viewer.GetTcpPose()</c>:
	///   [0..2] = X / Y / Z in mm (world frame)
	///   [3..5] = A / B / C Euler in degrees
	///
	/// <see cref="VRatio"/> / <see cref="ARatio"/> are override multipliers in
	/// [<see cref="MinRatio"/>, <see cref="MaxRatio"/>]. They scale the
	/// MoveL / MoveJ speed constants at dispatch time (3.8.2 / 3.10), *not*
	/// absolute velocities — so per-axis limits from 2.12 still bind.
	///
	/// <see cref="BlendRadius"/> is reserved for 3.11; kept &gt;= 0 for now,
	/// 0 = stop-at-corner (current Sprint #3 behaviour).
	/// </summary>
	public sealed class Waypoint : INotifyPropertyChanged
	{
		public const int PoseElementCount = 6;
		public const double MinRatio = 0.1;
		public const double MaxRatio = 2.0;
		public const double DefaultRatio = 1.0;

		public event PropertyChangedEventHandler PropertyChanged;

		private string _name = string.Empty;
		private readonly double[] _pose = new double[ PoseElementCount ];
		private MotionType _motion = MotionType.L;
		private double _vRatio = DefaultRatio;
		private double _aRatio = DefaultRatio;
		private double _blendRadius;

		public string Name {
			get => _name;
			set {
				var v = value ?? string.Empty;
				if( _name == v ) return;
				_name = v;
				OnChanged();
			}
		}

		/// <summary>
		/// Live 6-element XYZ (mm) + ABC (deg) array. Read-only reference for
		/// callers who need the raw buffer (e.g. dispatch to MoveL kernel). Use
		/// <see cref="SetPose"/> for bulk writes so change notification fires.
		/// </summary>
		public double[] PoseXyzAbc => _pose;

		// Per-component accessors so DataGrid columns can bind by path in 3.7.2
		// (WPF 4.8 array-indexer paths are awkward; scalar props are cleaner).
		public double X { get => _pose[ 0 ]; set => SetPoseComponent( 0, value ); }
		public double Y { get => _pose[ 1 ]; set => SetPoseComponent( 1, value ); }
		public double Z { get => _pose[ 2 ]; set => SetPoseComponent( 2, value ); }
		public double A { get => _pose[ 3 ]; set => SetPoseComponent( 3, value ); }
		public double B { get => _pose[ 4 ]; set => SetPoseComponent( 4, value ); }
		public double C { get => _pose[ 5 ]; set => SetPoseComponent( 5, value ); }

		public MotionType Motion {
			get => _motion;
			set {
				if( _motion == value ) return;
				_motion = value;
				OnChanged();
			}
		}

		public double VRatio {
			get => _vRatio;
			set {
				var v = Clamp( value, MinRatio, MaxRatio );
				if( _vRatio == v ) return;
				_vRatio = v;
				OnChanged();
			}
		}

		public double ARatio {
			get => _aRatio;
			set {
				var v = Clamp( value, MinRatio, MaxRatio );
				if( _aRatio == v ) return;
				_aRatio = v;
				OnChanged();
			}
		}

		public double BlendRadius {
			get => _blendRadius;
			set {
				var v = value < 0.0 ? 0.0 : value;
				if( _blendRadius == v ) return;
				_blendRadius = v;
				OnChanged();
			}
		}

		/// <summary>
		/// Bulk pose write. Ignores <c>null</c> / short arrays. Fires per-component
		/// change notifications for any element that actually changed so DataGrid
		/// scalar columns refresh; the <see cref="PoseXyzAbc"/> reference itself is
		/// stable and does not need a notification.
		/// </summary>
		public void SetPose( double[] pose )
		{
			if( pose == null || pose.Length < PoseElementCount ) return;
			for( int i = 0; i < PoseElementCount; i++ ) {
				if( _pose[ i ] == pose[ i ] ) continue;
				_pose[ i ] = pose[ i ];
				OnChanged( PoseComponentNames[ i ] );
			}
		}

		private void SetPoseComponent( int index, double value )
		{
			if( _pose[ index ] == value ) return;
			_pose[ index ] = value;
			OnChanged( PoseComponentNames[ index ] );
		}

		private static readonly string[] PoseComponentNames = { nameof( X ), nameof( Y ), nameof( Z ), nameof( A ), nameof( B ), nameof( C ) };

		private static double Clamp( double v, double lo, double hi )
		{
			if( v < lo ) return lo;
			if( v > hi ) return hi;
			return v;
		}

		private void OnChanged( [CallerMemberName] string propertyName = null )
			=> PropertyChanged?.Invoke( this, new PropertyChangedEventArgs( propertyName ) );
	}
}
