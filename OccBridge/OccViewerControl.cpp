#include <msclr/marshal_cppstd.h>
#include "OccViewerControl.h"
#include "NativeOccView.h"
#include "RobotPartDef.h"
#include <vector>
#include <string>

namespace OccBridge {

	OccViewerControl::OccViewerControl( void )
		: _native(new NativeOccView()), _initialized(false)
	// Creates the native viewer and sets default control appearance: dark background, fill parent, no double-buffering
	{
		this->Dock = DockStyle::Fill;
		this->DoubleBuffered = false;
		this->SetStyle(ControlStyles::Selectable, true);
	}

	OccViewerControl::~OccViewerControl( void )
	// Delegates to the finalizer to release unmanaged resources
	{
		this->!OccViewerControl();
	}

	OccViewerControl::!OccViewerControl( void )
	// Deletes the native viewer object to prevent memory leaks
	{
		if( _native != nullptr ) {
			delete _native;
			_native = nullptr;
		}
	}

	void OccViewerControl::OnHandleCreated(EventArgs^ e)
	// Initializes OCCT the first time the window handle is ready; skips if already initialized
	{
		UserControl::OnHandleCreated(e);
		if( !_initialized && this->Handle != IntPtr::Zero ) {
			_native->initialize(static_cast<HWND>(this->Handle.ToPointer()));
			_initialized = true;
		}
	}

	void OccViewerControl::OnResize(EventArgs^ e)
	// Notifies OCCT to adjust the viewport when the window is resized
	{
		UserControl::OnResize(e);
		if( _initialized ) {
			_native->resize(this->Width, this->Height);
		}
	}

	void OccViewerControl::OnPaint(PaintEventArgs^ e)
	// Delegates WinForms repaint to OCCT redraw to prevent blank flicker
	{
		UserControl::OnPaint(e);
		if( _initialized ) {
			_native->redraw();
		}
	}

	bool OccViewerControl::LoadStep(String^ path, bool append)
	// Marshals the managed string to a native wide string and forwards to the native loader
	{
		if( !_initialized ) {
			return false;
		}

		std::wstring nativePath = msclr::interop::marshal_as<std::wstring>(path);
		return _native->loadStep(nativePath.c_str(), append);
	}

	void OccViewerControl::ClearScene( void )
	// Forwards to the native clear, releasing all AIS objects and axis state
	{
		if( _initialized ) {
			_native->clearScene();
		}
	}

	bool OccViewerControl::LoadRobotArm(cli::array<RobotPartInfo^>^ parts,
										cli::array<cli::array<int>^>^ axisToPartMap)
	// Converts the managed arrays to native structs and forwards to the native loader
	{
		if( !_initialized ) {
			return false;
		}

		std::vector<RobotPartDef> nativeParts(parts->Length);
		for (int i = 0; i < parts->Length; i++)
		{
			System::String^ fp = parts[i]->FilePath;
			nativeParts[i].filePath = msclr::interop::marshal_as<std::wstring>(fp);
			nativeParts[i].dhA = parts[i]->DH_a;
			nativeParts[i].dhAlpha = parts[i]->DH_alpha;
			nativeParts[i].dhD = parts[i]->DH_d;
			nativeParts[i].dhTheta = parts[i]->DH_theta;
			for (int j = 0; j < 6; j++)
				nativeParts[i].offset[j] = parts[i]->Offset[j];
			nativeParts[i].parentIdx = parts[i]->ParentIdx;
			nativeParts[i].colorR = parts[i]->ColorR;
			nativeParts[i].colorG = parts[i]->ColorG;
			nativeParts[i].colorB = parts[i]->ColorB;
		}

		std::vector<int> nativeMap;
		if( axisToPartMap != nullptr ) {
			for (int i = 0; i < axisToPartMap->Length; i++)
			{
				nativeMap.push_back(axisToPartMap[i][0]);
				nativeMap.push_back(axisToPartMap[i][1]);
			}
		}

		return _native->loadRobotArm(nativeParts.data(), static_cast<int>(nativeParts.size()),
									 nativeMap.data(), static_cast<int>(nativeMap.size()));
	}

	void OccViewerControl::SetJointAngle(int axisIndex, double angleDeg)
	// Forwards the joint angle to the native viewer
	{
		if( _initialized ) {
			_native->setJointAngle(axisIndex, angleDeg);
		}
	}

	void OccViewerControl::FitAllView( void )
	// Forwards fit-all to the native viewer
	{
		if( _initialized ) {
			_native->fitAll();
		}
	}

	void OccViewerControl::SetViewIso( void )
	// Forwards isometric view switch to the native viewer
	{
		if( _initialized ) {
			_native->setViewIso();
		}
	}

	void OccViewerControl::SetViewTop( void )
	// Forwards top view switch to the native viewer
	{
		if( _initialized ) {
			_native->setViewTop();
		}
	}

	void OccViewerControl::OnMouseDown(MouseEventArgs^ e)
	// Ensures the control has keyboard focus, then forwards mouse-down to the native viewer
	{
		UserControl::OnMouseDown(e);
		if( !this->Focused ) {
			this->Focus();
		}
		if( _initialized ) {
			_native->onMouseDown(e->X, e->Y, static_cast<int>(e->Button));
		}
	}

	void OccViewerControl::OnMouseMove(MouseEventArgs^ e)
	// Forwards mouse-move to the native viewer for rotation, pan, or highlight update
	{
		UserControl::OnMouseMove(e);
		if( _initialized ) {
			_native->onMouseMove(e->X, e->Y, static_cast<int>(e->Button));
		}
	}

	void OccViewerControl::OnMouseUp(MouseEventArgs^ e)
	// Forwards mouse-up to the native viewer to stop rotation and pan
	{
		UserControl::OnMouseUp(e);
		if( _initialized ) {
			_native->onMouseUp();
		}
	}

	void OccViewerControl::OnMouseWheel(MouseEventArgs^ e)
	// Forwards mouse-wheel to the native viewer to zoom the 3D scene
	{
		UserControl::OnMouseWheel(e);
		if( _initialized ) {
			_native->onMouseWheel(e->Delta);
		}
	}

}
