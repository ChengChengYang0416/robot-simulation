#pragma once

class NativeOccView;

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
		// Constructor; creates a NativeOccView instance and sets default control appearance

		~OccViewerControl( void );
		// Destructor; delegates to the finalizer to release native resources

		!OccViewerControl( void );
		// Finalizer; deletes the NativeOccView native object

		bool LoadStep( String^ path, bool append );
		// Loads the STEP file at the given path; clears the scene first when append is false

		bool LoadRobotArm( cli::array<RobotPartInfo^>^ parts,
					   cli::array<cli::array<int>^>^ axisToPartMap,
					   Action<int, int>^ progress );
		// Loads robot arm parts with per-part progress callback (current, total)

		void SetJointAngle( int axisIndex, double angleDeg );
		// Sets the joint angle (degrees) for the given 0-based axis

		void ClearScene( void );
		// Removes all objects from the 3D scene

		void FitAllView( void );
		// Auto-fits the camera to show all scene objects

		void SetViewIso( void );
		// Switches to an isometric view projection

		void SetViewTop( void );
		// Switches to a top-down view projection

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
		NativeOccView* _native;
		// Pointer to the native OCCT viewer instance; manages the 3D scene and interactions

		bool _initialized;
		// Tracks whether the OCCT viewer has been initialized to prevent invalid calls
	};

}
