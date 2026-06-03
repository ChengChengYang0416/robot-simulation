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

		private void BtnMoveTo_Click( object sender, RoutedEventArgs e )
		{
			// TODO: hook IK + MoveL once the kinematics bridge is wired up.
			MessageBox.Show( this, "Move To: IK + trajectory execution not implemented yet.",
				"Auto Mode", MessageBoxButton.OK, MessageBoxImage.Information );
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
