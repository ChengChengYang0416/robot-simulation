using System.Globalization;
using System.Windows.Controls;

namespace Hmi.Controls
{
	public partial class StatusDashboard : UserControl
	{
		private readonly TextBlock[] _jointTexts;
		private readonly TextBlock[] _poseTexts;

		public StatusDashboard()
		{
			InitializeComponent();
			_jointTexts = new[] { TxtJ1, TxtJ2, TxtJ3, TxtJ4, TxtJ5, TxtJ6 };
			_poseTexts = new[] { TxtX, TxtY, TxtZ, TxtRx, TxtRy, TxtRz };
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

		public void Reset()
		{
			// Resets all readouts to zero, used when scene is cleared
			for( int i = 0; i < _jointTexts.Length; i++ ) {
				_jointTexts[ i ].Text = "0.00";
			}
			for( int i = 0; i < _poseTexts.Length; i++ ) {
				_poseTexts[ i ].Text = "0.00";
			}
		}
	}
}
