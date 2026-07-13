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
		// Cartesian JOG (World frame) speed caps. Conservative defaults — the
		// player is open-loop joystick-style with no S-curve, so faster speeds
		// would feel twitchy on a mouse hold. A future speed-multiplier slider
		// can rescale via _cartesianJog.LinSpeedMmPerSec / .AngSpeedDegPerSec.
		private const double JogCartLinSpeedMmPerSec  = 50.0;
		private const double JogCartAngSpeedDegPerSec = 20.0;
		private enum JogFrame { Joint, World }
		private DispatcherTimer _jogTimer;
		private JogFrame _jogFrame = JogFrame.Joint;
		private int _jogAxis = -1;
		private int _jogDir;
		private DateTime _jogLastTick;
		// Cartesian JOG scratch state. Held as fields so the 30 ms tick allocates
		// nothing on the managed heap (which would otherwise feed gen0 collections
		// and undermine the 1.9 monitor chart noise floor).
		private OccBridge.CartesianJog _cartesianJog;
		private readonly double[] _jogCartTarget    = new double[ 6 ];
		private readonly double[] _jogCartOutAngles = new double[ JointCount ];

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
		// Singularity-aware tempo control: instead of sampling the profile at the
		// wall-clock elapsed time, the tick advances a virtual clock by dt * scale
		// where scale comes from the SpeedScaler ramp. ratio >= warn → scale 1
		// (full speed), ratio <= crit → scale 0 (player aborts), in between is a
		// linear ramp so deceleration is smooth.
		private double _moveLVirtualSec;
		private DateTime _moveLLastTickUtc;
		private OccBridge.SpeedScaler _moveLScaler;		// Quaternion-slerp orientation interpolator: replaces the per-axis ABC
		// lerp so long-distance orientation changes follow the geodesic great
		// circle with constant angular velocity instead of zig-zagging through
		// Euler space. Scratch buffer holds the per-tick sample output and is
		// reused so the tick allocates nothing on the managed heap.
		private OccBridge.PoseInterp _moveLPoseInterp;
		private readonly double[] _moveLAbcScratch = new double[ 3 ];
		// MoveJ runtime: joint-space interpolation. One-shot IK at the start resolves the
		// target TCP pose into joint angles; the tick then lerps each axis independently
		// with the slowest axis pacing the total duration so all axes finish together.
		// Skips the per-tick IK that MoveL pays, never hits Cartesian singularities, but
		// the TCP traces an arc rather than a straight line in Cartesian space.
		private const int MoveJTickMs = 3;
		private const double MoveJSpeedDegPerSec  = 60.0;
		private const double MoveJAccelDegPerSec2 = 240.0;        // reaches vMax in 0.25 s
		private const double MoveJJerkDegPerSec3  = 2400.0;       // reaches aMax in 0.10 s
		// Reset-to-home replays through the MoveJ S-curve at conservative caps —
		// pressing "Home" is a safety reset, not a race; halving the MoveJ caps
		// keeps the shape ratio identical so the motion still feels smooth, just
		// slower (~12 s for a full ±170° J1 swing instead of ¶6 s).
		private const double HomeReturnSpeedDegPerSec  = 30.0;
		private const double HomeReturnAccelDegPerSec2 = 120.0;
		private const double HomeReturnJerkDegPerSec3  = 1200.0;
		private DispatcherTimer _moveJTimer;
		private double[] _moveJStart;
		private double[] _moveJTarget;
		private DateTime _moveJT0;
		private OccBridge.MotionProfile _moveJProfile;
		private double _moveJVirtualSec;
		private DateTime _moveJLastTickUtc;

		// Shared scratch buffer for OccViewerControl.GetManipulability(). Reused
		// across MoveL ticks so the per-tick allocation cost is zero. (MoveJ is
		// joint-space and does not consult the Jacobian.)
		private readonly double[] _singularityMetrics = new double[ 5 ];

		private const string RegistryKey = @"SOFTWARE\RobotSimulation";
		private const string RegistryValue = "LastModelFolder";

		public MainWindow()
		{
			InitializeComponent();
			Icon = new BitmapImage( new Uri( "pack://application:,,,/robot-icon.png" ) );
			_viewer = new OccViewerControl();
			WinFormsHost.Child = _viewer;
			// Drag mode commits IK results directly to the native kinematics core,
			// bypassing SetJointAngle(s). Without this subscription the HMI's
			// _jointAngles cache stays stale after every drag and downstream
			// consumers (Reset-to-Home delta, MoveJ seed, dashboard readout) see
			// pre-drag values. OccViewerControl fires JointsChanged on every
			// mouse-up so this handler covers both drag and camera-only clicks.
			_viewer.JointsChanged += OnViewerJointsChanged;
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
					// Push per-axis joint limits so internal IK callers (drag mode) stay
					// within physical range without having to ferry the limit arrays
					// across the bridge on every drag tick.
					_viewer.SetJointLimits( _jointMin, _jointMax );
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
			// Reset the drag toggle before clearing so its event handler observes the
			// not-loaded state and skips re-enabling the gizmo against a torn-down anchor.
			if( TglDragEnable != null && TglDragEnable.IsChecked == true ) {
				TglDragEnable.IsChecked = false;
			}
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

		private void MenuTrailMode_Click( object sender, RoutedEventArgs e )
		{
			// Three checkable menu items share this handler and behave as a radio
			// group via Tag = "0" / "1" / "2". Manual mutual-exclusion is needed
			// because WPF's MenuItem doesn't ship a built-in radio mode; using
			// GroupName like RadioButton would require nested controls inside
			// MenuItems and break keyboard navigation.
			if( !( sender is MenuItem clicked ) || clicked.Tag == null ) {
				return;
			}
			if( !int.TryParse( clicked.Tag.ToString(), out int mode ) ) {
				return;
			}

			// Re-checking the currently selected item should keep it checked
			// (the IsCheckable toggle would otherwise uncheck it and leave the
			// menu showing "no mode selected" which doesn't match reality).
			clicked.IsChecked = true;
			MenuTrailOff   .IsChecked = ( mode == 0 );
			MenuTrailLine  .IsChecked = ( mode == 1 );
			MenuTrailFrames.IsChecked = ( mode == 2 );

			_viewer.SetTcpTrailMode( mode );
			SetStatus( mode == 0 ? "TCP trail off."
				: mode == 1 ? "TCP trail: path."
				: "TCP trail: path + frames." );
		}

		private void MenuTrailClear_Click( object sender, RoutedEventArgs e )
		{
			_viewer.ClearTcpTrail();
			SetStatus( "TCP trail cleared." );
		}

		private void MenuTrailColor_Click( object sender, RoutedEventArgs e )
		{
			// Preset colour picker: Tag = "R,G,B" (0..255 each) so the XAML stays
			// declarative and the code-behind stays a thin router. Using a
			// checkable-menu radio group matches the mode picker above; a real
			// colour picker would require Extended.Wpf.Toolkit which we deliberately
			// don't take as a dependency for six preset swatches.
			if( !( sender is MenuItem clicked ) || clicked.Tag == null ) {
				return;
			}
			var parts = clicked.Tag.ToString().Split( ',' );
			if( parts.Length != 3
				|| !int.TryParse( parts[ 0 ], out int r )
				|| !int.TryParse( parts[ 1 ], out int g )
				|| !int.TryParse( parts[ 2 ], out int b ) ) {
				return;
			}

			clicked.IsChecked = true;
			MenuTrailColorYellow .IsChecked = ReferenceEquals( clicked, MenuTrailColorYellow  );
			MenuTrailColorCyan   .IsChecked = ReferenceEquals( clicked, MenuTrailColorCyan    );
			MenuTrailColorMagenta.IsChecked = ReferenceEquals( clicked, MenuTrailColorMagenta );
			MenuTrailColorGreen  .IsChecked = ReferenceEquals( clicked, MenuTrailColorGreen   );
			MenuTrailColorRed    .IsChecked = ReferenceEquals( clicked, MenuTrailColorRed     );
			MenuTrailColorWhite  .IsChecked = ReferenceEquals( clicked, MenuTrailColorWhite   );

			_viewer.SetTcpTrailColor( r, g, b );
			SetStatus( "TCP trail color: " + clicked.Header.ToString() + "." );
		}

		private void MenuTrailMaxPoints_Click( object sender, RoutedEventArgs e )
		{
			// Preset ring-buffer caps. The native side clamps to a hard upper
			// bound, so a bad Tag can't blow up memory. Same radio-group idiom
			// as the mode picker.
			if( !( sender is MenuItem clicked ) || clicked.Tag == null ) {
				return;
			}
			if( !int.TryParse( clicked.Tag.ToString(), out int maxPoints ) ) {
				return;
			}

			clicked.IsChecked = true;
			MenuTrailMax500  .IsChecked = ( maxPoints == 500 );
			MenuTrailMax1000 .IsChecked = ( maxPoints == 1000 );
			MenuTrailMax2000 .IsChecked = ( maxPoints == 2000 );
			MenuTrailMax5000 .IsChecked = ( maxPoints == 5000 );
			MenuTrailMax10000.IsChecked = ( maxPoints == 10000 );

			_viewer.SetTcpTrailMaxPoints( maxPoints );
			SetStatus( "TCP trail max points: " + maxPoints + "." );
		}

		private void MenuTrailStride_Click( object sender, RoutedEventArgs e )
		{
			// Frame stride only takes visible effect while PolylineWithFrames is
			// active; the native side stores it regardless so a later mode switch
			// picks it up without needing another user click.
			if( !( sender is MenuItem clicked ) || clicked.Tag == null ) {
				return;
			}
			if( !int.TryParse( clicked.Tag.ToString(), out int stride ) ) {
				return;
			}

			clicked.IsChecked = true;
			MenuTrailStride10 .IsChecked = ( stride == 10 );
			MenuTrailStride25 .IsChecked = ( stride == 25 );
			MenuTrailStride50 .IsChecked = ( stride == 50 );
			MenuTrailStride100.IsChecked = ( stride == 100 );
			MenuTrailStride200.IsChecked = ( stride == 200 );

			_viewer.SetTcpTrailFrameStride( stride );
			SetStatus( "TCP trail frame stride: " + stride + "." );
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
			if( btn == null || !TryParseJogTag( btn.Tag, out JogFrame frame, out int axis, out int dir ) ) {
				return;
			}
			btn.CaptureMouse();
			StartJog( frame, axis, dir );
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

		private static bool TryParseJogTag( object tag, out JogFrame frame, out int axis, out int dir )
		{
			// Two accepted formats:
			//   "axis:dir"        → Joint frame (legacy / 6-axis joint buttons)
			//   "W:axis:dir"      → World frame (Cartesian JOG, axis 0..5 = X/Y/Z/Rx/Ry/Rz)
			// Keeping the legacy 2-part form means the existing joint buttons need no
			// XAML retag, and the new World buttons advertise their frame explicitly
			// in the tag itself so a misclick can't drive Joint logic into World axes.
			frame = JogFrame.Joint;
			axis = -1;
			dir = 0;
			var s = tag as string;
			if( string.IsNullOrEmpty( s ) ) {
				return false;
			}
			var parts = s.Split( ':' );
			if( parts.Length == 2 ) {
				return int.TryParse( parts[ 0 ], out axis ) && int.TryParse( parts[ 1 ], out dir );
			}
			if( parts.Length == 3 ) {
				if( parts[ 0 ] == "W" ) {
					frame = JogFrame.World;
				}
				else if( parts[ 0 ] == "J" ) {
					frame = JogFrame.Joint;
				}
				else {
					return false;
				}
				return int.TryParse( parts[ 1 ], out axis ) && int.TryParse( parts[ 2 ], out dir );
			}
			return false;
		}

		private void StartJog( JogFrame frame, int axis, int dir )
		{
			// Both frames address 0..5 (joint index vs world Cartesian channel) so
			// the axis range check collapses to a single bound. dir == 0 is rejected
			// up front to keep CartesianJogStepper::step from having to special-case
			// a zero direction on every tick.
			if( axis < 0 || axis >= 6 || dir == 0 ) {
				return;
			}
			_jogFrame = frame;
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

			if( _jogFrame == JogFrame.Joint ) {
				double next = _jointAngles[ _jogAxis ] + _jogDir * JogSpeedDegPerSec * dt;
				next = Math.Max( _jointMin[ _jogAxis ], Math.Min( _jointMax[ _jogAxis ], next ) );

				if( Math.Abs( next - _jointAngles[ _jogAxis ] ) < 1e-6 ) {
					return; // already at limit
				}
				ApplyJointAngle( _jogAxis, next );
				return;
			}

			// World-frame Cartesian JOG: build the next desired TCP pose via the
			// shared stepper (position lerp / quaternion-composed orientation),
			// then route through IK. IK failure → stop and surface the reason; the
			// hand-on-button operator gets immediate feedback if the wrist hit a
			// singularity or workspace boundary instead of silently freezing.
			if( _cartesianJog == null ) {
				_cartesianJog = new OccBridge.CartesianJog( JogCartLinSpeedMmPerSec, JogCartAngSpeedDegPerSec );
			}
			var pose = _viewer.GetTcpPose();
			if( pose == null || pose.Length < 6 ) {
				StopJog();
				return;
			}
			if( !_cartesianJog.Step( pose, _jogAxis, _jogDir, dt, _jogCartTarget ) ) {
				StopJog();
				return;
			}
			int status = _viewer.SolveTcpIk( _jogCartTarget, _jointMin, _jointMax, _jogCartOutAngles );
			if( status != 0 ) {
				StopJog();
				SetStatus( status == 2
					? "Cartesian JOG stopped: IK did not converge (workspace limit or singularity)."
					: "Cartesian JOG stopped: invalid IK configuration." );
				return;
			}
			ApplyAllJointAngles( _jogCartOutAngles );
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
			// Pushes the latest joint angles, TCP pose, and singularity status to the dashboard.
			Dashboard.UpdateJoints( _jointAngles );

			var pose = _viewer.GetTcpPose();
			if( pose != null ) {
				Dashboard.UpdateTcpPose( pose );
			}

			// Singularity badge: native SingularityMonitor runs only when a robot is
			// loaded. Returns false otherwise → leave the badge in its reset state.
			if( _viewer.GetManipulability( _singularityMetrics, out int kind, out int level ) ) {
				Dashboard.UpdateSingularity( level, kind,
					_singularityMetrics[ 3 ],   // wristRatio
					_singularityMetrics[ 4 ] ); // armRatio
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
			// Two reset paths:
			//   No robot loaded → just zero the HMI cache (no player available).
			//   Robot loaded   → play a smooth MoveJ back to the all-zeros pose so the
			//                    operator sees the motion (no instant teleport — also
			//                    avoids fake velocity / acceleration spikes that would
			//                    contaminate the 1.9 monitor chart).
			if( !_robotLoaded ) {
				ResetSliders();
				return;
			}
			var home = new double[ JointCount ];  // implicit zero-initialised
			StartMoveJToJointTarget( home,
				HomeReturnSpeedDegPerSec, HomeReturnAccelDegPerSec2, HomeReturnJerkDegPerSec3,
				"MoveJ Home" );
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

			// True geodesic rotation angle between start and target orientations,
			// computed once by the quaternion slerp helper. Replaces the old
			// max(|dA|, |dB|, |dC|) approximation, which over-estimated whenever
			// the three Euler deltas pointed along non-parallel axes and was
			// outright wrong near gimbal lock. Sizing the profile with the true
			// arc length means MoveL durations now match the actual orientation
			// path the wrist will walk.
			if( _moveLPoseInterp == null ) {
				_moveLPoseInterp = new OccBridge.PoseInterp();
			}
			var startAbc  = new[] { current[ 3 ], current[ 4 ], current[ 5 ] };
			var targetAbc = new[] { target[ 3 ],  target[ 4 ],  target[ 5 ]  };
			_moveLPoseInterp.Begin( startAbc, targetAbc );
			double angularDist = _moveLPoseInterp.TotalAngleDeg;

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
								   target[ 3 ],  target[ 4 ],  target[ 5 ]  };
			_moveLProfile = profile;
			_moveLT0 = DateTime.UtcNow;
			_moveLVirtualSec  = 0.0;
			_moveLLastTickUtc = _moveLT0;
			if( _moveLScaler == null ) {
				_moveLScaler = new OccBridge.SpeedScaler();
			}
			_moveLScaler.Reset();

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

			// Singularity-aware tempo: advance the virtual clock by dt * scale,
			// where scale comes from the SpeedScaler ramp on the latest Jacobian
			// manipulability ratio. Sampling the manipulability *before* applying
			// the new joint angles means the deceleration kicks in on the way
			// *into* the singular region rather than after the fact.
			DateTime now = DateTime.UtcNow;
			double   dt  = ( now - _moveLLastTickUtc ).TotalSeconds;
			_moveLLastTickUtc = now;

			double scale = 1.0;
			if( _viewer.GetManipulability( _singularityMetrics, out int _, out int _ ) ) {
				double ratio = OccBridge.SpeedScaler.Combine(
					_singularityMetrics[ 3 ],   // wristRatio
					_singularityMetrics[ 4 ] ); // armRatio
				scale = _moveLScaler.Scale( ratio );
				if( _moveLScaler.ShouldAnnounceCritical() ) {
					StopMoveL( silent: true );
					SetStatus( "MoveL aborted: singularity (critical) — no safe path." );
					MessageBox.Show( this,
						"Trajectory stopped: the robot crossed into a critical singularity (wrist or elbow rank loss).\n\n"
						+ "Adjust the target pose or seed posture to leave the degenerate region before retrying.",
						"MoveL", MessageBoxButton.OK, MessageBoxImage.Warning );
					return;
				}
			}
			_moveLVirtualSec += dt * scale;

			double s = _moveLProfile.Sample( _moveLVirtualSec );
			bool reachedEnd = _moveLVirtualSec >= _moveLProfile.DurationSec;

			var interp = new double[ 6 ];
			// Position: straight Cartesian lerp under the same s(t) as orientation,
			// so the two stay phase-locked and reach the endpoint together.
			for( int i = 0; i < 3; i++ ) {
				interp[ i ] = _moveLStart[ i ] + s * ( _moveLTarget[ i ] - _moveLStart[ i ] );
			}
			// Orientation: slerp on the segment endpoints set up in BtnMoveL_Click.
			// The scratch buffer is reused so this tick still allocates nothing
			// (the `new double[6] interp` above is the only per-tick allocation and
			// is small enough to land in gen0 every cycle).
			_moveLPoseInterp.Sample( s, _moveLAbcScratch );
			interp[ 3 ] = _moveLAbcScratch[ 0 ];
			interp[ 4 ] = _moveLAbcScratch[ 1 ];
			interp[ 5 ] = _moveLAbcScratch[ 2 ];

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

		private void OnViewerJointsChanged( double[] anglesDeg )
		{
			// Sync the HMI joint cache with whatever the native side committed. Fires
			// after every mouse-up so drag-mode IK commits become visible to Home /
			// MoveJ / dashboard without any explicit polling. A no-op payload (no
			// robot / non-drag click) short-circuits — the array reference check
			// avoids allocating garbage while a MoveL / MoveJ tick is also running
			// (their own ApplyAllJointAngles path is authoritative during motion).
			if( anglesDeg == null || anglesDeg.Length < JointCount ) {
				return;
			}
			if( ( _moveLTimer != null && _moveLTimer.IsEnabled )
			 || ( _moveJTimer != null && _moveJTimer.IsEnabled ) ) {
				return;
			}
			bool changed = false;
			for( int i = 0; i < JointCount; i++ ) {
				if( Math.Abs( _jointAngles[ i ] - anglesDeg[ i ] ) > 1e-9 ) {
					_jointAngles[ i ] = anglesDeg[ i ];
					changed = true;
				}
			}
			if( changed ) {
				UpdateDashboard();
			}
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

			StartMoveJToJointTarget( targetJoints,
				MoveJSpeedDegPerSec, MoveJAccelDegPerSec2, MoveJJerkDegPerSec3,
				"MoveJ trap" );
		}

		private bool StartMoveJToJointTarget( double[] targetJoints,
											  double speedDegPerSec,
											  double accelDegPerSec2,
											  double jerkDegPerSec3,
											  string statusLabel )
		{
			// Joint-space player kernel. Callers that already hold a resolved joint
			// vector (BtnMoveJ_Click after IK, BtnResetHome_Click with all-zeros,
			// future Waypoint queue with stored teach points) plug straight in here
			// and inherit the S-curve / dominant-axis pacing / singularity-aware
			// scaler infrastructure without duplicating the planning boilerplate.
			//
			// Slowest-axis pacing under jerk-limited S-curve: plan a profile for every
			// axis using the supplied vMax / aMax / jMax, pick the one with the longest
			// duration as dominant. All axes follow the same s(t) so they reach the
			// endpoint together; faster axes simply move below their cap. Per-axis
			// caps (roadmap 2.12) will refine which axis ends up dominant.
			OccBridge.MotionProfile dominant = null;
			double maxDelta = 0.0;
			for( int i = 0; i < JointCount; i++ ) {
				double delta = Math.Abs( targetJoints[ i ] - _jointAngles[ i ] );
				var p = OccBridge.MotionProfile.CreateSCurve(
					delta, speedDegPerSec, accelDegPerSec2, jerkDegPerSec3 );
				if( dominant == null || p.DurationSec > dominant.DurationSec ) {
					dominant = p;
					maxDelta = delta;
				}
			}

			if( dominant == null || dominant.DurationSec < 1.0e-6 ) {
				SetStatus( $"{statusLabel}: already at target." );
				return false;
			}

			StopJog();
			StopMoveL( silent: true );
			StopMoveJ( silent: true );

			_moveJStart   = (double[])_jointAngles.Clone();
			_moveJTarget  = (double[])targetJoints.Clone();
			_moveJProfile = dominant;
			_moveJT0 = DateTime.UtcNow;
			_moveJVirtualSec  = 0.0;
			_moveJLastTickUtc = _moveJT0;

			if( _moveJTimer == null ) {
				_moveJTimer = new DispatcherTimer( DispatcherPriority.Render ) {
					Interval = TimeSpan.FromMilliseconds( MoveJTickMs ),
				};
				_moveJTimer.Tick += MoveJTimer_Tick;
			}
			_moveJTimer.Start();
			UpdateStopButtonState();
			SetStatus( $"{statusLabel}: max Δ {maxDelta:F1}° in {dominant.DurationSec:F2} s" );
			return true;
		}

		private void MoveJTimer_Tick( object sender, EventArgs e )
		{
			if( !_robotLoaded || _moveJStart == null || _moveJTarget == null || _moveJProfile == null ) {
				StopMoveJ( silent: true );
				return;
			}

			DateTime now = DateTime.UtcNow;
			double   dt  = ( now - _moveJLastTickUtc ).TotalSeconds;
			_moveJLastTickUtc = now;

			// MoveJ is joint-space: target joints are already known, no IK runs each
			// frame, the Jacobian is irrelevant. The path stays well-defined even when
			// it crosses (or ends at) a singular configuration — e.g. the all-zeros
			// home pose typically sits on the wrist singularity. So we advance virtual
			// time at wall-clock rate without consulting the manipulability scaler;
			// MoveL still uses it, and the StatusDashboard badge keeps updating from
			// its own live query path.
			_moveJVirtualSec += dt;

			double s = _moveJProfile.Sample( _moveJVirtualSec );
			bool reachedEnd = _moveJVirtualSec >= _moveJProfile.DurationSec;

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
			// Drag mode requires a loaded robot to anchor the gizmo. Block the toggle
			// otherwise to avoid a silently-invisible gizmo confusing the operator.
			var enabled = TglDragEnable.IsChecked == true;
			if( enabled && !_robotLoaded ) {
				TglDragEnable.IsChecked = false;
				SetStatus( "Load a robot before enabling drag." );
				return;
			}
			_viewer?.SetDragEnabled( enabled );
			SetStatus( enabled ? "Drag enabled: grab a gizmo handle to move TCP."
								: "Drag disabled." );
		}

		private void RbGizmoMode_Checked( object sender, RoutedEventArgs e )
		{
			// Both Translate and Rotate radios route here. Only meaningful when the
			// Drag Target is TCP; if Joint is currently selected, PushGizmoMode still
			// pushes Joint to the native side and the Gizmo radios are effectively
			// stashed for the next time the operator switches back to TCP.
			// Guard on _viewer because the initial XAML IsChecked="True" fires the
			// Checked event during InitializeComponent(), before the viewer field is
			// assigned in the constructor.
			if( _viewer == null ) {
				return;
			}
			PushGizmoMode();
		}

		private void RbDragTarget_Checked( object sender, RoutedEventArgs e )
		{
			// Drag Target radios flip between TCP (uses the AIS_Manipulator gizmo,
			// styled by the Gizmo radios) and Joint (per-link picker, gizmo hidden).
			// Same InitializeComponent guard as RbGizmoMode_Checked.
			if( _viewer == null ) {
				return;
			}
			PushGizmoMode();
		}

		private void PushGizmoMode()
		{
			// Consolidates both radio groups into the single int the native facade's
			// setGizmoMode consumes. Mapping (must match IRobotScene::GizmoMode):
			//   Drag Target = Joint            → 2 (Joint)   — Gizmo radios ignored
			//   Drag Target = TCP + Translate → 0 (Translate)
			//   Drag Target = TCP + Rotate    → 1 (Rotate)
			int mode;
			string label;
			if( RbDragJoint != null && RbDragJoint.IsChecked == true ) {
				mode  = 2;
				label = "Drag target: joint — click a link to rotate its axis.";
			} else if( RbGizmoRotate != null && RbGizmoRotate.IsChecked == true ) {
				mode  = 1;
				label = "Gizmo: rotate — drag a ring to reorient TCP.";
			} else {
				mode  = 0;
				label = "Gizmo: translate — drag an arrow to move TCP.";
			}
			_viewer.SetGizmoMode( mode );
			SetStatus( label );
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
