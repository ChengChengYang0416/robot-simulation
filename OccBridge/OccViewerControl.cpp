#include <msclr/marshal_cppstd.h>
#include "OccViewerControl.h"
#include "NativeOccView.h"
#include "RobotPartDef.h"
#include <vector>
#include <string>

namespace OccBridge {

	OccViewerControl::OccViewerControl()
		: _native(new NativeOccView()), _initialized(false)
	{
		this->BackColor = System::Drawing::Color::FromArgb(46, 46, 46);
		this->Dock = DockStyle::Fill;
		this->DoubleBuffered = false;
		this->SetStyle(ControlStyles::Selectable, true);
	}

	OccViewerControl::~OccViewerControl()
	{
		this->!OccViewerControl();
	}

	OccViewerControl::!OccViewerControl()
	{
		if( _native != nullptr ) {
			delete _native;
			_native = nullptr;
		}
	}

	void OccViewerControl::OnHandleCreated(EventArgs^ e)
	{
		UserControl::OnHandleCreated(e);
		if( !_initialized && this->Handle != IntPtr::Zero ) {
			_native->initialize(static_cast<HWND>(this->Handle.ToPointer()));
			_initialized = true;
		}
	}

	void OccViewerControl::OnResize(EventArgs^ e)
	{
		UserControl::OnResize(e);
		if( _initialized ) {
			_native->resize(this->Width, this->Height);
		}
	}

	void OccViewerControl::OnPaint(PaintEventArgs^ e)
	{
		UserControl::OnPaint(e);
		if( _initialized ) {
			_native->redraw();
		}
	}

	bool OccViewerControl::LoadStep(String^ path, bool append)
	{
		if( !_initialized ) {
			return false;
		}

		std::wstring nativePath = msclr::interop::marshal_as<std::wstring>(path);
		return _native->loadStep(nativePath.c_str(), append);
	}

	void OccViewerControl::ClearScene()
	{
		if( _initialized ) {
			_native->clearScene();
		}
	}

	bool OccViewerControl::LoadRobotArm(cli::array<RobotPartInfo^>^ parts,
										cli::array<cli::array<int>^>^ axisToPartMap)
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
	{
		if( _initialized ) {
			_native->setJointAngle(axisIndex, angleDeg);
		}
	}

	void OccViewerControl::FitAllView()
	{
		if( _initialized ) {
			_native->fitAll();
		}
	}

	void OccViewerControl::SetViewIso()
	{
		if( _initialized ) {
			_native->setViewIso();
		}
	}

	void OccViewerControl::SetViewTop()
	{
		if( _initialized ) {
			_native->setViewTop();
		}
	}

	void OccViewerControl::OnMouseDown(MouseEventArgs^ e)
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
	{
		UserControl::OnMouseMove(e);
		if( _initialized ) {
			_native->onMouseMove(e->X, e->Y, static_cast<int>(e->Button));
		}
	}

	void OccViewerControl::OnMouseUp(MouseEventArgs^ e)
	{
		UserControl::OnMouseUp(e);
		if( _initialized ) {
			_native->onMouseUp();
		}
	}

	void OccViewerControl::OnMouseWheel(MouseEventArgs^ e)
	{
		UserControl::OnMouseWheel(e);
		if( _initialized ) {
			_native->onMouseWheel(e->Delta);
		}
	}

}
