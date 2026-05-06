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
		OccViewerControl();
		~OccViewerControl();
		!OccViewerControl();

		bool LoadStep(String^ path, bool append);
		bool LoadRobotArm(cli::array<RobotPartInfo^>^ parts,
						  cli::array<cli::array<int>^>^ axisToPartMap);
		void SetJointAngle(int axisIndex, double angleDeg);
		void ClearScene();
		void FitAllView();
		void SetViewIso();
		void SetViewTop();

	protected:
		virtual void OnHandleCreated(EventArgs^ e) override;
		virtual void OnResize(EventArgs^ e) override;
		virtual void OnPaint(PaintEventArgs^ e) override;
		virtual void OnMouseDown(MouseEventArgs^ e) override;
		virtual void OnMouseMove(MouseEventArgs^ e) override;
		virtual void OnMouseUp(MouseEventArgs^ e) override;
		virtual void OnMouseWheel(MouseEventArgs^ e) override;

	private:
		NativeOccView* _native;
		bool _initialized;
	};

}
