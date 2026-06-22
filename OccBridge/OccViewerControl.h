#pragma once

namespace OccBridge {
	class IRobotScene;
}

using namespace System;
using namespace System::Windows::Forms;

namespace OccBridge {

	public ref class RobotPartInfo
	{
	public:
		System::String^ FilePath;
		double DH_a, DH_alpha, DH_d, DH_theta;
		cli::array<double>^ Offset;  // 6 elements: tx, ty, tz, rx, ry, rz
		int ParentIdx;
		int ColorR, ColorG, ColorB;
	};

	public ref class OccViewerControl : public UserControl
	{
	public:
		OccViewerControl( void );
		// Constructor; creates the native scene via createRobotScene() and sets default control appearance

		~OccViewerControl( void );
		// Destructor; delegates to the finalizer to release native resources

		!OccViewerControl( void );
		// Finalizer; deletes the IRobotScene native object

		bool LoadStep( String^ path, bool append );
		// Loads the STEP file at the given path; clears the scene first when append is false

		bool LoadRobotArm( cli::array<RobotPartInfo^>^ parts,
					   cli::array<cli::array<int>^>^ axisToPartMap,
					   Action<int, int>^ progress );
		// Loads robot arm parts with per-part progress callback (current, total)

		void SetJointAngle( int axisIndex, double angleDeg );
		// Sets the joint angle (degrees) for the given 0-based axis

		void SetJointAngles( cli::array<double>^ anglesDeg );
		// Batch variant of SetJointAngle: pushes all six joint angles to the native
		// scene and triggers exactly one redraw. Required by MoveL / MoveJ tick paths
		// where 6 individual SetJointAngle calls would cause 6 full scene redraws.

		cli::array<double>^ GetTcpPose( void );
		// Returns the TCP pose as [x, y, z, rx, ry, rz] in mm and degrees,
		// or nullptr if no robot is currently loaded

		int SolveTcpIk( cli::array<double>^ targetXyzRpy,
						cli::array<double>^ jointMinDeg,
						cli::array<double>^ jointMaxDeg,
						cli::array<double>^ outAnglesDeg );
		// Solves IK for the target TCP pose using the current joint angles as seed.
		// targetXyzRpy / jointMinDeg / jointMaxDeg / outAnglesDeg must each be length 6.
		// Returns IRobotScene::IkSolveStatus cast to int: 0 = Converged, 1 = NoRobot,
		// 2 = NotConverged, 3 = InvalidConfig. The scene's joint state is left untouched
		// on every outcome; the caller commits the solution via SetJointAngle when 0.

		void ClearScene( void );
		// Removes all objects from the 3D scene

		void FitAllView( void );
		// Auto-fits the camera to show all scene objects

		void SetViewIso( void );
		// Switches to an isometric view projection

		void SetViewTop( void );
		// Switches to a top-down view projection

		bool SaveScreenshot( String^ path );
		// Saves the current view to an image file (PNG / JPG / BMP / TIFF inferred
		// from the path extension). Returns false when the viewer is not initialized,
		// the path is null/empty, or the underlying encoder fails.

	protected:
		virtual void OnHandleCreated( EventArgs^ e ) override;
		// Called once the window handle is created; initializes the OCCT viewer

		virtual void OnResize( EventArgs^ e ) override;
		// Notifies OCCT to adjust the viewport when the window is resized

		virtual void OnPaint( PaintEventArgs^ e ) override;
		// Delegates WinForms repaint to OCCT redraw to prevent blank flicker

		virtual void OnMouseDown( MouseEventArgs^ e ) override;
		// Forwards mouse-down events to the native viewer

		virtual void OnMouseMove( MouseEventArgs^ e ) override;
		// Forwards mouse-move events to the native viewer

		virtual void OnMouseUp( MouseEventArgs^ e ) override;
		// Forwards mouse-up events to the native viewer

		virtual void OnMouseWheel( MouseEventArgs^ e ) override;
		// Forwards mouse-wheel events to the native viewer

	private:
		IRobotScene* m_pNative;
		// Pointer to the abstract scene interface; concrete instance is built by
		// OccBridge::createRobotScene() so the managed wrapper never names the
		// implementation directly (DIP). Future tests can inject a fake by
		// exposing an alternate constructor that takes an IRobotScene*.

		bool m_bInitialized;
		// Tracks whether the OCCT viewer has been initialized to prevent invalid calls
	};

}
