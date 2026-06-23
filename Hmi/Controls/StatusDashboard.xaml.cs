using System.Globalization;
using System.Windows.Controls;
using System.Windows.Media;

namespace Hmi.Controls
{
	public partial class StatusDashboard : UserControl
	{
		private readonly TextBlock[] _jointTexts;
		private readonly TextBlock[] _poseTexts;

		// Cached brushes for the singularity badge so per-tick updates do not
		// allocate. Matches the level enum: 0=Normal, 1=Warning, 2=Critical.
		private static readonly Brush BrushNormal   = new SolidColorBrush( Color.FromRgb( 0xC8, 0xE6, 0xC9 ) );
		private static readonly Brush BrushWarning  = new SolidColorBrush( Color.FromRgb( 0xFF, 0xE0, 0x82 ) );
		private static readonly Brush BrushCritical = new SolidColorBrush( Color.FromRgb( 0xEF, 0x9A, 0x9A ) );
		private static readonly Brush BrushUnknown  = Brushes.LightGray;

		static StatusDashboard()
		{
			BrushNormal.Freeze();
			BrushWarning.Freeze();
			BrushCritical.Freeze();
		}

		public StatusDashboard()
		{
			InitializeComponent();
			_jointTexts = new[] { TxtJ1, TxtJ2, TxtJ3, TxtJ4, TxtJ5, TxtJ6 };
			_poseTexts = new[] { TxtX, TxtY, TxtZ, TxtA, TxtB, TxtC };
		}

		public void UpdateJoints( double[] anglesDeg )
		{
			// Updates the six joint readouts; expects array of length 6 in degrees
			if( anglesDeg == null ) {
				return;
			}
			int n = anglesDeg.Length < 6 ? anglesDeg.Length : 6;
			for( int i = 0; i < n; i++ ) {
				_jointTexts[ i ].Text = anglesDeg[ i ].ToString( "F2", CultureInfo.InvariantCulture );
			}
		}

		public void UpdateTcpPose( double[] pose )
		{
			// Updates TCP pose readouts; expects [x, y, z, rx, ry, rz] in mm and degrees
			if( pose == null ) {
				return;
			}
			int n = pose.Length < 6 ? pose.Length : 6;
			for( int i = 0; i < n; i++ ) {
				_poseTexts[ i ].Text = pose[ i ].ToString( "F2", CultureInfo.InvariantCulture );
			}
		}

		public void UpdateSingularity( int level, int kind, double wristRatio, double armRatio )
		{
			// Reflects the latest SingularityMonitor::SingularityReport on screen.
			// metrics layout (from OccViewerControl.GetManipulability):
			//   [0] manipulability, [1] wristManipulability, [2] armManipulability,
			//   [3] wristRatio,     [4] armRatio
			// `level`: 0 Normal / 1 Warning / 2 Critical
			// `kind` : 0 None / 1 Wrist / 2 Elbow / 3 Shoulder / 4 Combined
			string kindLabel;
			switch( kind ) {
				case 1: kindLabel = "Wrist"; break;
				case 2: kindLabel = "Elbow"; break;
				case 3: kindLabel = "Shoulder"; break;
				case 4: kindLabel = "Combined"; break;
				default: kindLabel = "None"; break;
			}
			string levelLabel;
			Brush badgeBrush;
			switch( level ) {
				case 1: levelLabel = "Warning"; badgeBrush = BrushWarning;  break;
				case 2: levelLabel = "Critical"; badgeBrush = BrushCritical; break;
				default: levelLabel = "Normal"; badgeBrush = BrushNormal;   break;
			}
			TxtSingStatus.Text = ( kind == 0 ) ? levelLabel : ( levelLabel + " · " + kindLabel );
			SingBadge.Background = badgeBrush;
			TxtSingWrist.Text = wristRatio.ToString( "F3", CultureInfo.InvariantCulture );
			TxtSingArm.Text   = armRatio.ToString( "F3", CultureInfo.InvariantCulture );
		}

		public void Reset()
		{
			// Resets all readouts to zero, used when scene is cleared
			for( int i = 0; i < _jointTexts.Length; i++ ) {
				_jointTexts[ i ].Text = "0.00";
			}
			for( int i = 0; i < _poseTexts.Length; i++ ) {
				_poseTexts[ i ].Text = "0.00";
			}
			TxtSingStatus.Text = "—";
			SingBadge.Background = BrushUnknown;
			TxtSingWrist.Text = "—";
			TxtSingArm.Text   = "—";
		}
	}
}
