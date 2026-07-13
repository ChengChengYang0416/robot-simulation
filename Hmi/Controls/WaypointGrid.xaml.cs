using System;
using System.Windows;
using System.Windows.Controls;
using Hmi.Models;

namespace Hmi.Controls
{
	/// <summary>
	/// Teach-pendant UI: DataGrid bound to a <see cref="WaypointStore"/> plus
	/// Add / Delete / ↑ / ↓ / Clear buttons. Add captures the current TCP pose
	/// through an injected <see cref="PoseProvider"/> callback so this control
	/// never takes a direct dependency on OccBridge / MainWindow.
	///
	/// Data-binding round-trips (edit a cell → the underlying <see cref="Waypoint"/>
	/// mutates) rely on <see cref="Waypoint"/>'s INPC implementation added in 3.7.1;
	/// no ViewModel layer is needed at this stage.
	/// </summary>
	public partial class WaypointGrid : UserControl
	{
		private WaypointStore _store;

		public WaypointGrid()
		{
			InitializeComponent();
		}

		/// <summary>
		/// Store this control drives. Assigning rebinds the DataGrid; assigning
		/// null clears the binding. Safe to leave null at construction — the
		/// button handlers no-op until a store is present.
		/// </summary>
		public WaypointStore Store {
			get => _store;
			set {
				_store = value;
				Grid.ItemsSource = value?.Waypoints;
			}
		}

		/// <summary>
		/// Callback the Add button uses to snapshot the current TCP pose. When
		/// unset or returning null (e.g. no robot loaded), Add falls back to an
		/// all-zeros waypoint so operators can teach off-line and edit numerically.
		/// </summary>
		public Func<double[]> PoseProvider { get; set; }

		/// <summary>
		/// Callback the Start button raises with the selected waypoint. MainWindow
		/// dispatches to the MoveL / MoveJ kernel based on <see cref="Waypoint.Motion"/>;
		/// this control stays ignorant of motion primitives. Unset → button is a no-op.
		/// 3.10 will layer <c>WaypointSequencePlayer</c> on top of the same kernels;
		/// the Start button remains the single-shot / manual entry point.
		/// </summary>
		public Action<Waypoint> StartRequested { get; set; }

		private void BtnAdd_Click( object sender, RoutedEventArgs e )
		{
			if( _store == null ) return;
			var pose = PoseProvider?.Invoke();
			var wp = pose != null
				? _store.CaptureFromPose( pose )
				: _store.CaptureFromPose( new double[ Waypoint.PoseElementCount ] );
			if( wp == null ) return;
			Grid.SelectedItem = wp;
			Grid.ScrollIntoView( wp );
		}

		private void BtnDelete_Click( object sender, RoutedEventArgs e )
		{
			if( _store == null ) return;
			int idx = Grid.SelectedIndex;
			if( !_store.RemoveAt( idx ) ) return;
			int newIdx = Math.Min( idx, _store.Count - 1 );
			if( newIdx >= 0 ) {
				Grid.SelectedIndex = newIdx;
			}
		}

		private void BtnUp_Click( object sender, RoutedEventArgs e )
		{
			if( _store == null ) return;
			int idx = Grid.SelectedIndex;
			if( _store.MoveUp( idx ) ) {
				Grid.SelectedIndex = idx - 1;
			}
		}

		private void BtnDown_Click( object sender, RoutedEventArgs e )
		{
			if( _store == null ) return;
			int idx = Grid.SelectedIndex;
			if( _store.MoveDown( idx ) ) {
				Grid.SelectedIndex = idx + 1;
			}
		}

		private void BtnClear_Click( object sender, RoutedEventArgs e )
		{
			if( _store == null || _store.Count == 0 ) return;
			var result = MessageBox.Show(
				Window.GetWindow( this ),
				$"Clear all {_store.Count} waypoints? This cannot be undone.",
				"Clear Waypoints",
				MessageBoxButton.OKCancel,
				MessageBoxImage.Warning );
			if( result == MessageBoxResult.OK ) {
				_store.Clear();
			}
		}

		private void BtnStart_Click( object sender, RoutedEventArgs e )
		{
			if( StartRequested == null ) return;
			var wp = Grid.SelectedItem as Waypoint;
			if( wp == null ) return;
			StartRequested( wp );
		}
	}
}
