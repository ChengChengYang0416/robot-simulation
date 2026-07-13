using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using Hmi.Models;
using Hmi.Motion;

namespace Hmi.Controls
{
	/// <summary>
	/// Teach-pendant UI: DataGrid bound to a <see cref="WaypointStore"/> plus
	/// Add / Delete / ↑ / ↓ / Clear buttons, a single-shot Start button, and
	/// (when a <see cref="Player"/> is wired) a Play / Pause / Step / Stop /
	/// Loop / Speed sequence control row backed by
	/// <see cref="WaypointSequencePlayer"/>.
	///
	/// Add captures the current TCP pose through an injected
	/// <see cref="PoseProvider"/> callback so this control never takes a direct
	/// dependency on OccBridge / MainWindow. Data-binding round-trips (edit a
	/// cell → the underlying <see cref="Waypoint"/> mutates) rely on the INPC
	/// implementation added in 3.7.1; no ViewModel layer is needed at this stage.
	/// </summary>
	public partial class WaypointGrid : UserControl
	{
		private WaypointStore _store;
		private WaypointSequencePlayer _player;

		public WaypointGrid()
		{
			InitializeComponent();
			UpdateSequenceButtonStates();
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
		/// </summary>
		public Action<Waypoint> StartRequested { get; set; }

		/// <summary>
		/// Sequence player driving Play / Pause / Step / Stop / Loop / Speed.
		/// Setting the property (re)subscribes to <see cref="WaypointSequencePlayer.StateChanged"/>
		/// so button enables track the state machine. Null clears wiring — the
		/// sequence-row buttons then remain disabled.
		/// </summary>
		public WaypointSequencePlayer Player {
			get => _player;
			set {
				if( _player != null ) _player.StateChanged -= OnPlayerStateChanged;
				_player = value;
				if( _player != null ) {
					_player.StateChanged += OnPlayerStateChanged;
					_player.Loop          = ChkLoop.IsChecked == true;
					_player.SpeedOverride = SldSpeed.Value;
				}
				UpdateSequenceButtonStates();
			}
		}

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

		// -------------------------------------------------------------------
		// Sequence controls (3.8.3 / 3.10)
		// -------------------------------------------------------------------

		private void BtnPlay_Click( object sender, RoutedEventArgs e )
		{
			if( _player == null || _store == null ) return;
			// Play doubles as Resume so a single obvious action button covers
			// both "start fresh" and "continue after Pause".
			if( _player.State == PlayerState.Paused ) {
				_player.Resume();
			} else {
				_player.Start( _store.Waypoints );
			}
		}

		private void BtnPause_Click( object sender, RoutedEventArgs e )
		{
			_player?.Pause();
		}

		private void BtnStep_Click( object sender, RoutedEventArgs e )
		{
			if( _player == null || _store == null ) return;
			_player.Step( _store.Waypoints );
		}

		private void BtnStop_Click( object sender, RoutedEventArgs e )
		{
			_player?.Stop();
		}

		private void ChkLoop_Changed( object sender, RoutedEventArgs e )
		{
			if( _player != null ) _player.Loop = ChkLoop.IsChecked == true;
		}

		private void SldSpeed_ValueChanged( object sender, RoutedPropertyChangedEventArgs<double> e )
		{
			if( _player != null ) _player.SpeedOverride = e.NewValue;
			// TxtSpeed is null during InitializeComponent's first slider-value binding.
			if( TxtSpeed != null ) {
				TxtSpeed.Text = e.NewValue.ToString( "F2", CultureInfo.InvariantCulture ) + "×";
			}
		}

		private void OnPlayerStateChanged( PlayerState state )
		{
			UpdateSequenceButtonStates();
		}

		private void UpdateSequenceButtonStates()
		{
			var state = _player?.State ?? PlayerState.Idle;
			bool hasPlayer = _player != null;
			BtnPlay.IsEnabled  = hasPlayer && state != PlayerState.Running;
			BtnPause.IsEnabled = hasPlayer && state == PlayerState.Running;
			BtnStep.IsEnabled  = hasPlayer && state != PlayerState.Running;
			BtnStop.IsEnabled  = hasPlayer && ( state == PlayerState.Running || state == PlayerState.Paused );
			ChkLoop.IsEnabled  = hasPlayer;
			SldSpeed.IsEnabled = hasPlayer;
		}
	}
}

