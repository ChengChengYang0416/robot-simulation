using Microsoft.Win32;
using OccBridge;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Web.Script.Serialization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Interop;
using System.Windows.Threading;
using System.Runtime.InteropServices;

namespace Hmi
{
	public partial class MainWindow : Window
	{
		private readonly OccViewerControl _viewer;
		private bool _robotLoaded;

		// Joint state for JOG (replaces sliders)
		private const int JointCount = 6;
		private readonly double[] _jointAngles = new double[ JointCount ];
		private readonly double[] _jointMin = { -170, -96, -195, -170, -120, -360 };
		private readonly double[] _jointMax = {  170, 130,   65,  170,  120,  360 };

		// JOG runtime
		private const double JogSpeedDegPerSec = 30.0;
		private const int JogTickMs = 30;
		private DispatcherTimer _jogTimer;
		private int _jogAxis = -1;
		private int _jogDir;
		private DateTime _jogLastTick;

		// MoveL runtime: linear interpolation of [X, Y, Z, A, B, C] from start to target,
		// running IK each frame and committing the joint solution to the scene.
		// 3 ms is aspirational; DispatcherTimer typically lands at the WPF render rate
		// (~16 ms), so the wall-clock based progress ratio keeps motion duration honest.
		private const int MoveLTickMs = 3;
		private const double MoveLLinearSpeedMmPerSec   = 100.0;
		private const double MoveLLinearAccelMmPerSec2  = 400.0;   // reaches vMax in 0.25 s
		private const double MoveLLinearJerkMmPerSec3   = 4000.0;  // reaches aMax in 0.10 s
		private const double MoveLAngularSpeedDegPerSec = 45.0;
		private const double MoveLAngularAccelDegPerSec2 = 180.0;  // reaches vMax in 0.25 s
		private const double MoveLAngularJerkDegPerSec3  = 1800.0; // reaches aMax in 0.10 s
		private DispatcherTimer _moveLTimer;
		private double[] _moveLStart;
		private double[] _moveLTarget;
		private DateTime _moveLT0;
		private OccBridge.MotionProfile _moveLProfile;

		// MoveJ runtime: joint-space interpolation. One-shot IK at the start resolves the
		// target TCP pose into joint angles; the tick then lerps each axis independently
		// with the slowest axis pacing the total duration so all axes finish together.
		// Skips the per-tick IK that MoveL pays, never hits Cartesian singularities, but
		// the TCP traces an arc rather than a straight line in Cartesian space.
		private const int MoveJTickMs = 3;
		private const double MoveJSpeedDegPerSec  = 60.0;
		private const double MoveJAccelDegPerSec2 = 240.0;        // reaches vMax in 0.25 s
		private const double MoveJJerkDegPerSec3  = 2400.0;       // reaches aMax in 0.10 s
		private DispatcherTimer _moveJTimer;
		private double[] _moveJStart;
		private double[] _moveJTarget;
		private DateTime _moveJT0;
		private OccBridge.MotionProfile _moveJProfile;

		private const string RegistryKey = @"SOFTWARE\RobotSimulation";
		private const string RegistryValue = "LastModelFolder";

		public MainWindow()
		{
			InitializeComponent();
			Icon = new BitmapImage( new Uri( "pack://application:,,,/robot-icon.png" ) );
			_viewer = new OccViewerControl();
			WinFormsHost.Child = _viewer;
			SetStatus( "Ready" );

			var lastFolder = GetLastModelFolder();
			if( lastFolder != null && Directory.Exists( lastFolder ) ) {
				Loaded += ( s, e ) => LoadRobotFromFolder( lastFolder );
			}
		}

		private static readonly SolidColorBrush BrushGreen = new SolidColorBrush( Color.FromRgb( 0x4C, 0xAF, 0x50 ) );
		private static readonly SolidColorBrush BrushOrange = new SolidColorBrush( Color.FromRgb( 0xFF, 0x98, 0x00 ) );

		private void SetStatus( string text, bool loading = false )
		{
			TxtStatus.Text = text;
			StatusBarMain.Background = loading ? BrushOrange : BrushGreen;
		}

		private void BtnOpenFolder_Click( object sender, RoutedEventArgs e )
		{
			var path = ShowFolderDialog( "Select a folder containing .step and .json files" );
			if( path != null ) {
				LoadRobotFromFolder( path );
			}
		}

		private string ShowFolderDialog( string title )
		{
			var dialog = (IFileOpenDialog)new FileOpenDialog();
			dialog.GetOptions( out var options );
			dialog.SetOptions( options | 0x00000020 ); // FOS_PICKFOLDERS
			dialog.SetTitle( title );
			var hwnd = new WindowInteropHelper( this ).Handle;
			if( dialog.Show( hwnd ) != 0 ) {
				return null;
			}
			dialog.GetResult( out var item );
			item.GetDisplayName( 0x80058000, out var folderPath ); // SIGDN_FILESYSPATH
			return folderPath;
		}

		[ComImport, Guid( "DC1C5A9C-E88A-4DDE-A5A1-60F82A20AEF7" )]
		private class FileOpenDialog { }

		[ComImport, Guid( "D57C7288-D4AD-4768-BE02-9D969532D960" ),
		 InterfaceType( ComInterfaceType.InterfaceIsIUnknown )]
		private interface IFileOpenDialog
		{
			[PreserveSig] int Show( IntPtr hwndOwner );
			void SetFileTypes( uint cFileTypes, IntPtr rgFilterSpec );
			void SetFileTypeIndex( uint iFileType );
			void GetFileTypeIndex( out uint piFileType );
			void Advise( IntPtr pfde, out uint pdwCookie );
			void Unadvise( uint dwCookie );
			void SetOptions( uint fos );
			void GetOptions( out uint pfos );
			void SetDefaultFolder( IShellItem psi );
			void SetFolder( IShellItem psi );
			void GetFolder( out IShellItem ppsi );
			void GetCurrentSelection( out IShellItem ppsi );
			void SetFileName( [MarshalAs( UnmanagedType.LPWStr )] string pszName );
			void GetFileName( [MarshalAs( UnmanagedType.LPWStr )] out string pszName );
			void SetTitle( [MarshalAs( UnmanagedType.LPWStr )] string pszTitle );
			void SetOkButtonLabel( [MarshalAs( UnmanagedType.LPWStr )] string pszText );
			void SetFileNameLabel( [MarshalAs( UnmanagedType.LPWStr )] string pszLabel );
			void GetResult( out IShellItem ppsi );
		}

		[ComImport, Guid( "43826D1E-E718-42EE-BC55-A1E261C37BFE" ),
		 InterfaceType( ComInterfaceType.InterfaceIsIUnknown )]
		private interface IShellItem
		{
			void BindToHandler( IntPtr pbc, ref Guid bhid, ref Guid riid, out IntPtr ppv );
			void GetParent( out IShellItem ppsi );
			void GetDisplayName( uint sigdnName, [MarshalAs( UnmanagedType.LPWStr )] out string ppszName );
		}

		private void LoadRobotFromFolder( string folderPath )
		{
			var jsonFiles = Directory.GetFiles( folderPath, "*.json" );
			if( jsonFiles.Length == 0 ) {
				SetStatus( "No .json config file found in the selected folder." );
				return;
			}
			var jsonPath = jsonFiles[ 0 ];

			try {
				var json = File.ReadAllText( jsonPath );
				var serializer = new JavaScriptSerializer();
				var data = serializer.Deserialize<Dictionary<string, object>>( json );
				var partInfos = (ArrayList)data[ "PartInfos" ];

				var parts = new RobotPartInfo[ partInfos.Count ];
				for( int i = 0; i < partInfos.Count; i++ ) {
					var p = (Dictionary<string, object>)partInfos[ i ];
					var part = new RobotPartInfo();

					var cadFilePath = (string)p[ "CadFilePath" ];
					var cadFileName = Path.GetFileName( cadFilePath );
					part.FilePath = Path.Combine( folderPath, cadFileName );

					part.DH_a = Convert.ToDouble( p[ "a" ] );
					part.DH_alpha = Convert.ToDouble( p[ "alpha" ] );
					part.DH_d = Convert.ToDouble( p[ "d" ] );
					part.DH_theta = Convert.ToDouble( p[ "theta" ] );
					part.Offset = ParseDoubleArray( (string)p[ "Offset" ] );
					part.ParentIdx = Convert.ToInt32( p[ "ParentDHIdx" ] );

					var colors = ParseDoubleArray( (string)p[ "CadColor" ] );
					part.ColorR = (int)colors[ 0 ];
					part.ColorG = (int)colors[ 1 ];
					part.ColorB = (int)colors[ 2 ];

					parts[ i ] = part;
				}

				var axisMapRaw = ParseNestedIntArray( (string)data[ "AxisToPartMap" ] );

				// Apply AxisLimits from JSON to sliders
				if( data.ContainsKey( "AxisLimits" ) ) {
					var limits = ParseNestedDoubleArray( (string)data[ "AxisLimits" ] );
					ApplyAxisLimits( limits );
				}

				if( _viewer.LoadRobotArm( parts, axisMapRaw, ( current, total ) => {
					SetStatus( $"Loading part {current}/{total}...", true );
					var frame = new DispatcherFrame();
					Dispatcher.CurrentDispatcher.BeginInvoke( DispatcherPriority.Background,
						new Action( () => frame.Continue = false ) );
					Dispatcher.PushFrame( frame );
				} ) ) {
					_robotLoaded = true;
					ResetSliders();
					UpdateDashboard();
					SetLastModelFolder( folderPath );
					SetStatus( $"Loaded: {Path.GetFileName( jsonPath )}, {parts.Length} part(s)." );
				} else {
					SetStatus( "Failed to load robot arm." );
				}
			} catch( Exception ex ) {
				SetStatus( $"Load error: {ex.Message}" );
			}
		}

		private static string GetLastModelFolder()
		{
			using( var key = Registry.CurrentUser.OpenSubKey( RegistryKey ) ) {
				return key?.GetValue( RegistryValue ) as string;
			}
		}

		private static void SetLastModelFolder( string path )
		{
			using( var key = Registry.CurrentUser.CreateSubKey( RegistryKey ) ) {
				key.SetValue( RegistryValue, path );
			}
		}

		private static double[] ParseDoubleArray( string s )
		{
			// Parse "[1.0,2.0,3.0]" -> double[]
			s = s.Trim( '[', ']', ' ' );
			var parts = s.Split( ',' );
			var result = new double[ parts.Length ];
			for( int i = 0; i < parts.Length; i++ ) {
				result[ i ] = double.Parse( parts[ i ].Trim(), CultureInfo.InvariantCulture );
			}
			return result;
		}

		private void BtnClear_Click( object sender, RoutedEventArgs e )
		{
			StopMoveL( silent: true );
			StopMoveJ( silent: true );
			_viewer.ClearScene();
			_robotLoaded = false;
			ResetSliders();
			Dashboard.Reset();
			SetStatus( "Scene cleared." );
		}

		private void BtnIso_Click( object sender, RoutedEventArgs e )
		{
			_viewer.SetViewIso();
			MenuIso.IsChecked = true;
			MenuTop.IsChecked = false;
		}

		private void BtnTop_Click( object sender, RoutedEventArgs e )
		{
			_viewer.SetViewTop();
			MenuIso.IsChecked = false;
			MenuTop.IsChecked = true;
		}

		private void BtnFitAll_Click( object sender, RoutedEventArgs e )
		{
			_viewer.FitAllView();
		}

		private void SaveScreenshot_Executed( object sender, System.Windows.Input.ExecutedRoutedEventArgs e )
		{
			// Default filename includes a timestamp so rapid captures do not overwrite each other.
			var dialog = new SaveFileDialog {
				Title = "Save Screenshot",
				Filter = "PNG image (*.png)|*.png|JPEG image (*.jpg)|*.jpg|BMP image (*.bmp)|*.bmp|TIFF image (*.tiff)|*.tiff",
				FileName = "robot-" + DateTime.Now.ToString( "yyyyMMdd-HHmmss", CultureInfo.InvariantCulture ) + ".png",
				AddExtension = true,
			};
			if( dialog.ShowDialog( this ) != true ) {
				return;
			}

			if( _viewer.SaveScreenshot( dialog.FileName ) ) {
				SetStatus( "Screenshot saved: " + dialog.FileName );
			} else {
				SetStatus( "Failed to save screenshot." );
			}
		}

		private void JogBtn_Down( object sender, System.Windows.Input.MouseButtonEventArgs e )
		{
			if( !_robotLoaded ) {
				return;
			}
			var btn = sender as System.Windows.Controls.Button;
			if( btn == null || !TryParseJogTag( btn.Tag, out int axis, out int dir ) ) {
				return;
			}
			btn.CaptureMouse();
			StartJog( axis, dir );
		}

		private void JogBtn_Up( object sender, System.Windows.Input.MouseButtonEventArgs e )
		{
			var btn = sender as System.Windows.Controls.Button;
			if( btn != null && btn.IsMouseCaptured ) {
				btn.ReleaseMouseCapture();
			}
			StopJog();
		}

		private void JogBtn_Leave( object sender, System.Windows.Input.MouseEventArgs e )
		{
			// If the user drags the cursor off the button while held, stop jogging.
			var btn = sender as System.Windows.Controls.Button;
			if( btn != null && btn.IsMouseCaptured ) {
				btn.ReleaseMouseCapture();
				StopJog();
			}
		}

		private static bool TryParseJogTag( object tag, out int axis, out int dir )
		{
			axis = -1;
			dir = 0;
			var s = tag as string;
			if( string.IsNullOrEmpty( s ) ) {
				return false;
			}
			var parts = s.Split( ':' );
			if( parts.Length != 2 ) {
				return false;
			}
			return int.TryParse( parts[ 0 ], out axis ) && int.TryParse( parts[ 1 ], out dir );
		}

		private void StartJog( int axis, int dir )
		{
			if( axis < 0 || axis >= JointCount || dir == 0 ) {
				return;
			}
			_jogAxis = axis;
			_jogDir = dir;
			_jogLastTick = DateTime.UtcNow;

			if( _jogTimer == null ) {
				_jogTimer = new DispatcherTimer( DispatcherPriority.Render ) {
					Interval = TimeSpan.FromMilliseconds( JogTickMs ),
				};
				_jogTimer.Tick += JogTimer_Tick;
			}
			_jogTimer.Start();
		}

		private void StopJog()
		{
			_jogTimer?.Stop();
			_jogAxis = -1;
			_jogDir = 0;
		}

		private void JogTimer_Tick( object sender, EventArgs e )
		{
			if( !_robotLoaded || _jogAxis < 0 ) {
				StopJog();
				return;
			}
			var now = DateTime.UtcNow;
			double dt = ( now - _jogLastTick ).TotalSeconds;
			_jogLastTick = now;

			double next = _jointAngles[ _jogAxis ] + _jogDir * JogSpeedDegPerSec * dt;
			next = Math.Max( _jointMin[ _jogAxis ], Math.Min( _jointMax[ _jogAxis ], next ) );

			if( Math.Abs( next - _jointAngles[ _jogAxis ] ) < 1e-6 ) {
				return; // already at limit
			}
			ApplyJointAngle( _jogAxis, next );
		}

		private void ApplyJointAngle( int axis, double angle )
		{
			_jointAngles[ axis ] = angle;
			_viewer.SetJointAngle( axis, angle );
			UpdateDashboard();
		}

		private void ResetSliders()
		{
			StopJog();
			StopMoveL( silent: true );
			StopMoveJ( silent: true );
			for( int i = 0; i < JointCount; i++ ) {
				_jointAngles[ i ] = 0.0;
				if( _robotLoaded ) {
					_viewer.SetJointAngle( i, 0.0 );
				}
			}
			UpdateDashboard();
		}

		private void UpdateDashboard()
		{
			// Pushes the latest joint angles and TCP pose (from native) to the dashboard.
			Dashboard.UpdateJoints( _jointAngles );

			var pose = _viewer.GetTcpPose();
			if( pose != null ) {
				Dashboard.UpdateTcpPose( pose );
			}
		}

		private void ModeTab_SelectionChanged( object sender, SelectionChangedEventArgs e )
		{
			if( e.Source != ModeTabs ) {
				return;
			}
			var header = ( ModeTabs.SelectedItem as TabItem )?.Header?.ToString() ?? "?";
			SetStatus( $"Mode: {header}" );
		}

		private void BtnResetHome_Click( object sender, RoutedEventArgs e )
		{
			ResetSliders();
		}

		private void BtnCopyCurrentPose_Click( object sender, RoutedEventArgs e )
		{
			var pose = _viewer?.GetTcpPose();
			if( pose == null || pose.Length < 6 ) {
				SetStatus( "TCP pose unavailable" );
				return;
			}
			TxtPoseX.Text = pose[ 0 ].ToString( "F3", CultureInfo.InvariantCulture );
			TxtPoseY.Text = pose[ 1 ].ToString( "F3", CultureInfo.InvariantCulture );
			TxtPoseZ.Text = pose[ 2 ].ToString( "F3", CultureInfo.InvariantCulture );
			TxtPoseA.Text = pose[ 3 ].ToString( "F3", CultureInfo.InvariantCulture );
			TxtPoseB.Text = pose[ 4 ].ToString( "F3", CultureInfo.InvariantCulture );
			TxtPoseC.Text = pose[ 5 ].ToString( "F3", CultureInfo.InvariantCulture );
		}

		private bool TryReadPoseInputs( out double[] pose )
		{
			pose = new double[ 6 ];
			var inputs = new[] { TxtPoseX, TxtPoseY, TxtPoseZ, TxtPoseA, TxtPoseB, TxtPoseC };
			for( int i = 0; i < inputs.Length; i++ ) {
				if( !double.TryParse( inputs[ i ].Text, NumberStyles.Float, CultureInfo.InvariantCulture, out pose[ i ] ) ) {
					return false;
				}
			}
			return true;
		}

		private void BtnMoveL_Click( object sender, RoutedEventArgs e )
		{
			// Linear TCP motion: lerp position in mm and RPY in deg (after wrapping each
			// component delta to [-180, 180] so we always take the short way around),
			// solving IK per frame with the previous joint state as seed so the solver
			// tracks a single continuous branch. The progress function s(t) comes from a
			// MotionProfile so the same player can switch between linear, trapezoidal,
			// or future S-curve shapes without touching the tick loop.
			if( !_robotLoaded ) {
				SetStatus( "Load a robot first." );
				return;
			}
			if( !TryReadPoseInputs( out var target ) ) {
				SetStatus( "Invalid pose input: enter numeric XYZ (mm) and ABC (deg)." );
				return;
			}
			var current = _viewer.GetTcpPose();
			if( current == null || current.Length < 6 ) {
				SetStatus( "TCP pose unavailable." );
				return;
			}

			double dx = target[ 0 ] - current[ 0 ];
			double dy = target[ 1 ] - current[ 1 ];
			double dz = target[ 2 ] - current[ 2 ];
			double linearDist = Math.Sqrt( dx * dx + dy * dy + dz * dz );

			double dA = NormalizeAngleDeg( target[ 3 ] - current[ 3 ] );
			double dB = NormalizeAngleDeg( target[ 4 ] - current[ 4 ] );
			double dC = NormalizeAngleDeg( target[ 5 ] - current[ 5 ] );
			double angularDist = Math.Max( Math.Abs( dA ), Math.Max( Math.Abs( dB ), Math.Abs( dC ) ) );

			// Plan a trapezoidal profile for each dimension under its own vMax/aMax, then
			// keep the longer one as the dominant profile. The other dimension follows
			// the same s(t), which means it moves slower than its limit — that is the
			// price of synchronisation, and guarantees no dimension ever exceeds its cap.
			// Plan a jerk-limited S-curve profile for each dimension under its own
			// vMax / aMax / jMax, then keep the longer one as the dominant profile. The
			// other dimension follows the same s(t), which means it moves slower than
			// its limit — that is the price of synchronisation, and guarantees no
			// dimension ever exceeds its cap. Compared to a trapezoidal profile the
			// S-curve adds a constant-jerk phase on each end so the acceleration
			// itself ramps continuously, eliminating the velocity "kink" at phase
			// boundaries; useful when the robot is mounted on a flexible structure.
			var linPlan = OccBridge.MotionProfile.CreateSCurve(
				linearDist,  MoveLLinearSpeedMmPerSec,  MoveLLinearAccelMmPerSec2,  MoveLLinearJerkMmPerSec3 );
			var angPlan = OccBridge.MotionProfile.CreateSCurve(
				angularDist, MoveLAngularSpeedDegPerSec, MoveLAngularAccelDegPerSec2, MoveLAngularJerkDegPerSec3 );
			OccBridge.MotionProfile profile = ( linPlan.DurationSec >= angPlan.DurationSec ) ? linPlan : angPlan;

			if( profile.DurationSec < 1.0e-6 ) {
				SetStatus( "MoveL: already at target." );
				return;
			}

			// Cancel any in-flight motion so back-to-back clicks restart cleanly.
			StopMoveL( silent: true );
			StopMoveJ( silent: true );

			_moveLStart  = new[] { current[ 0 ], current[ 1 ], current[ 2 ],
								   current[ 3 ], current[ 4 ], current[ 5 ] };
			_moveLTarget = new[] { target[ 0 ], target[ 1 ], target[ 2 ],
								   current[ 3 ] + dA, current[ 4 ] + dB, current[ 5 ] + dC };
			_moveLProfile = profile;
			_moveLT0 = DateTime.UtcNow;

			if( _moveLTimer == null ) {
				_moveLTimer = new DispatcherTimer( DispatcherPriority.Render ) {
					Interval = TimeSpan.FromMilliseconds( MoveLTickMs ),
				};
				_moveLTimer.Tick += MoveLTimer_Tick;
			}
			_moveLTimer.Start();
			UpdateStopButtonState();
			SetStatus( $"MoveL trap: {linearDist:F1} mm / {angularDist:F1}° in {profile.DurationSec:F2} s" );
		}

		private void BtnStopMotion_Click( object sender, RoutedEventArgs e )
		{
			StopMoveL( silent: true );
			StopMoveJ( silent: true );
			SetStatus( "Motion stopped." );
			UpdateStopButtonState();
		}

		private void MoveLTimer_Tick( object sender, EventArgs e )
		{
			if( !_robotLoaded || _moveLStart == null || _moveLTarget == null || _moveLProfile == null ) {
				StopMoveL( silent: true );
				return;
			}

			double elapsed = ( DateTime.UtcNow - _moveLT0 ).TotalSeconds;
			double s = _moveLProfile.Sample( elapsed );
			bool reachedEnd = elapsed >= _moveLProfile.DurationSec;

			var interp = new double[ 6 ];
			for( int i = 0; i < 6; i++ ) {
				interp[ i ] = _moveLStart[ i ] + s * ( _moveLTarget[ i ] - _moveLStart[ i ] );
			}

			var outAngles = new double[ JointCount ];
			int status = _viewer.SolveTcpIk( interp, _jointMin, _jointMax, outAngles );
			if( status != 0 ) {
				StopMoveL( silent: true );
				SetStatus( status == 2
					? "MoveL aborted: IK did not converge mid-trajectory (singularity or unreachable point)."
					: "MoveL aborted: invalid IK configuration." );
				return;
			}

			ApplyAllJointAngles( outAngles );

			if( reachedEnd ) {
				StopMoveL( silent: true );
				SetStatus( "MoveL: target reached." );
			}
		}

		private void StopMoveL( bool silent )
		{
			_moveLTimer?.Stop();
			_moveLStart = null;
			_moveLTarget = null;
			_moveLProfile = null;
			UpdateStopButtonState();
			if( !silent ) {
				SetStatus( "MoveL stopped." );
			}
		}

		private void ApplyAllJointAngles( double[] anglesDeg )
		{
			// Batch variant of ApplyJointAngle: writes the joint cache, pushes a single
			// batched update across the C++/CLI boundary (one native redraw), then
			// refreshes the dashboard exactly once. Replaces the per-axis loop that
			// caused six full scene redraws per MoveL tick and produced visible jitter.
			for( int i = 0; i < JointCount; i++ ) {
				_jointAngles[ i ] = anglesDeg[ i ];
			}
			_viewer.SetJointAngles( anglesDeg );
			UpdateDashboard();
		}

		private static double NormalizeAngleDeg( double a )
		{
			// Wrap an angle delta to (-180, 180] so MoveL takes the short rotational path.
			while( a >   180.0 ) a -= 360.0;
			while( a <= -180.0 ) a += 360.0;
			return a;
		}

		private void BtnMoveJ_Click( object sender, RoutedEventArgs e )
		{
			// Joint-space motion: one-shot IK at the target TCP pose resolves a joint
			// vector, then every axis lerps from its current angle to that target with
			// the slowest axis pacing the total duration. All axes therefore start and
			// finish in lockstep, which is the defining MoveJ property; the TCP traces
			// whatever curve falls out of the kinematics, not a straight line.
			if( !_robotLoaded ) {
				SetStatus( "Load a robot first." );
				return;
			}
			if( !TryReadPoseInputs( out var target ) ) {
				SetStatus( "Invalid pose input: enter numeric XYZ (mm) and ABC (deg)." );
				return;
			}

			// Resolve target joints once. SolveTcpIk leaves the scene's joint state
			// untouched, so the lerp source below is still the live current angles.
			var targetJoints = new double[ JointCount ];
			int status = _viewer.SolveTcpIk( target, _jointMin, _jointMax, targetJoints );
			if( status != 0 ) {
				if( status == 2 ) {
					SetStatus( "MoveJ: target unreachable (IK did not converge)." );
					MessageBox.Show( this,
						"Inverse kinematics did not converge. The target pose may be outside the workspace or near a singularity.",
						"Auto Mode", MessageBoxButton.OK, MessageBoxImage.Warning );
				} else {
					SetStatus( "MoveJ: invalid IK configuration." );
				}
				return;
			}

			// Slowest-axis pacing under trapezoidal limits: plan a profile for every
			// axis using shared vMax/aMax, pick the one with the longest duration as
			// dominant. All axes follow the same s(t) so they reach the endpoint
			// together; the faster axes simply move below their cap. Per-axis caps
			// (roadmap 2.12) will refine which axis ends up dominant.
			// Slowest-axis pacing under jerk-limited S-curve: plan a profile for every
			// axis using shared vMax / aMax / jMax, pick the one with the longest
			// duration as dominant. All axes follow the same s(t) so they reach the
			// endpoint together; the faster axes simply move below their cap. Per-axis
			// caps (roadmap 2.12) will refine which axis ends up dominant.
			OccBridge.MotionProfile dominant = null;
			double maxDelta = 0.0;
			for( int i = 0; i < JointCount; i++ ) {
				double delta = Math.Abs( targetJoints[ i ] - _jointAngles[ i ] );
				var p = OccBridge.MotionProfile.CreateSCurve(
					delta, MoveJSpeedDegPerSec, MoveJAccelDegPerSec2, MoveJJerkDegPerSec3 );
				if( dominant == null || p.DurationSec > dominant.DurationSec ) {
					dominant = p;
					maxDelta = delta;
				}
			}

			if( dominant == null || dominant.DurationSec < 1.0e-6 ) {
				SetStatus( "MoveJ: already at target." );
				return;
			}

			StopMoveL( silent: true );
			StopMoveJ( silent: true );

			_moveJStart   = (double[])_jointAngles.Clone();
			_moveJTarget  = targetJoints;
			_moveJProfile = dominant;
			_moveJT0 = DateTime.UtcNow;

			if( _moveJTimer == null ) {
				_moveJTimer = new DispatcherTimer( DispatcherPriority.Render ) {
					Interval = TimeSpan.FromMilliseconds( MoveJTickMs ),
				};
				_moveJTimer.Tick += MoveJTimer_Tick;
			}
			_moveJTimer.Start();
			UpdateStopButtonState();
			SetStatus( $"MoveJ trap: max Δ {maxDelta:F1}° in {dominant.DurationSec:F2} s" );
		}

		private void MoveJTimer_Tick( object sender, EventArgs e )
		{
			if( !_robotLoaded || _moveJStart == null || _moveJTarget == null || _moveJProfile == null ) {
				StopMoveJ( silent: true );
				return;
			}

			double elapsed = ( DateTime.UtcNow - _moveJT0 ).TotalSeconds;
			double s = _moveJProfile.Sample( elapsed );
			bool reachedEnd = elapsed >= _moveJProfile.DurationSec;

			var interp = new double[ JointCount ];
			for( int i = 0; i < JointCount; i++ ) {
				interp[ i ] = _moveJStart[ i ] + s * ( _moveJTarget[ i ] - _moveJStart[ i ] );
			}
			ApplyAllJointAngles( interp );

			if( reachedEnd ) {
				StopMoveJ( silent: true );
				SetStatus( "MoveJ: target reached." );
			}
		}

		private void StopMoveJ( bool silent )
		{
			_moveJTimer?.Stop();
			_moveJStart = null;
			_moveJTarget = null;
			_moveJProfile = null;
			UpdateStopButtonState();
			if( !silent ) {
				SetStatus( "MoveJ stopped." );
			}
		}

		private void UpdateStopButtonState()
		{
			// Enables the Stop button whenever any trajectory player is active so a single
			// control can interrupt either MoveL or MoveJ without per-motion toggling.
			if( BtnStopMotion == null ) {
				return;
			}
			bool busy = ( _moveLTimer != null && _moveLTimer.IsEnabled )
					 || ( _moveJTimer != null && _moveJTimer.IsEnabled );
			BtnStopMotion.IsEnabled = busy;
		}

		private void TglDragEnable_Changed( object sender, RoutedEventArgs e )
		{
			// TODO: integrate with viewer drag/gizmo once available.
			var enabled = TglDragEnable.IsChecked == true;
			SetStatus( enabled ? "Drag enabled (stub)" : "Drag disabled" );
		}

		private void ApplyAxisLimits( double[][] limits )
		{
			for( int i = 0; i < Math.Min( limits.Length, JointCount ); i++ ) {
				if( limits[ i ].Length >= 2 ) {
					_jointMin[ i ] = limits[ i ][ 0 ];
					_jointMax[ i ] = limits[ i ][ 1 ];
				}
			}
		}

		private static double[][] ParseNestedDoubleArray( string s )
		{
			s = s.Trim();
			if( s.StartsWith( "[" ) ) {
				s = s.Substring( 1 );
			}
			if( s.EndsWith( "]" ) ) {
				s = s.Substring( 0, s.Length - 1 );
			}

			var result = new List<double[]>();
			int depth = 0;
			int start = -1;
			for( int i = 0; i < s.Length; i++ ) {
				if( s[ i ] == '[' ) {
					depth++;
					if( depth == 1 ) {
						start = i + 1;
					}
				} else if( s[ i ] == ']' ) {
					depth--;
					if( depth == 0 && start >= 0 ) {
						var inner = s.Substring( start, i - start );
						var vals = inner.Split( ',' );
						result.Add( vals.Select( v => double.Parse( v.Trim(), CultureInfo.InvariantCulture ) ).ToArray() );
						start = -1;
					}
				}
			}
			return result.ToArray();
		}

		private static int[][] ParseNestedIntArray( string s )
		{
			// Parse "[[1,1],[2,2],[3,3]]" -> int[][]
			// Only strip the outermost brackets
			s = s.Trim();
			if( s.StartsWith( "[" ) ) {
				s = s.Substring( 1 );
			}
			if( s.EndsWith( "]" ) ) {
				s = s.Substring( 0, s.Length - 1 );
			}

			var result = new List<int[]>();
			int depth = 0;
			int start = -1;
			for( int i = 0; i < s.Length; i++ ) {
				if( s[ i ] == '[' ) {
					depth++;
					if( depth == 1 ) {
						start = i + 1;
					}
				} else if( s[ i ] == ']' ) {
					depth--;
					if( depth == 0 && start >= 0 ) {
						var inner = s.Substring( start, i - start );
						var vals = inner.Split( ',' );
						result.Add( vals.Select( v => int.Parse( v.Trim() ) ).ToArray() );
						start = -1;
					}
				}
			}
			return result.ToArray();
		}
	}
}
