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
using System.Windows.Threading;

namespace Hmi
{
	public partial class MainWindow : Window
	{
		private readonly OccViewerControl _viewer;
		private bool _robotLoaded;
		private TextBlock[] _jointLabels;

		public MainWindow()
		{
			InitializeComponent();
			_viewer = new OccViewerControl();
			WinFormsHost.Child = _viewer;
			_jointLabels = new[] { LblJ1, LblJ2, LblJ3, LblJ4, LblJ5, LblJ6 };
			SetStatus( "Ready" );
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
			var dialog = new System.Windows.Forms.FolderBrowserDialog
			{
				Description = "Select a folder containing .step and .json files"
			};

			if( dialog.ShowDialog() == System.Windows.Forms.DialogResult.OK ) {
				LoadRobotFromFolder( dialog.SelectedPath );
			}
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
					SetStatus( $"Loaded: {Path.GetFileName( jsonPath )}, {parts.Length} part(s)." );
				} else {
					SetStatus( "Failed to load robot arm." );
				}
			} catch( Exception ex ) {
				SetStatus( $"Load error: {ex.Message}" );
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

		private void Slider_ValueChanged( object sender, RoutedPropertyChangedEventArgs<double> e )
		{
			if( !_robotLoaded ) {
				return;
			}

			var slider = (System.Windows.Controls.Slider)sender;
			int axisIndex = int.Parse( slider.Tag.ToString() );
			double angle = Math.Round( slider.Value );

			_viewer.SetJointAngle( axisIndex, angle );

			if( _jointLabels != null && axisIndex >= 0 && axisIndex < _jointLabels.Length ) {
				_jointLabels[ axisIndex ].Text = $"{(int)angle}°";
			}
		}

		private void ResetSliders()
		{
			var sliders = new[] { SliderJ1, SliderJ2, SliderJ3, SliderJ4, SliderJ5, SliderJ6 };
			foreach( var s in sliders ) {
				if( s != null ) {
					s.Value = 0;
				}
			}
			if( _jointLabels != null ) {
				foreach( var lbl in _jointLabels ) {
					if( lbl != null ) {
						lbl.Text = "0°";
					}
				}
			}
		}

		private void ApplyAxisLimits( double[][] limits )
		{
			var sliders = new[] { SliderJ1, SliderJ2, SliderJ3, SliderJ4, SliderJ5, SliderJ6 };
			for( int i = 0; i < Math.Min( limits.Length, sliders.Length ); i++ ) {
				if( limits[ i ].Length >= 2 && sliders[ i ] != null ) {
					sliders[ i ].Minimum = limits[ i ][ 0 ];
					sliders[ i ].Maximum = limits[ i ][ 1 ];
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
