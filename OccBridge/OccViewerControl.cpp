#include "OccViewerControl.h"

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
        if (_native != nullptr)
        {
            delete _native;
            _native = nullptr;
        }
    }

    void OccViewerControl::OnHandleCreated(EventArgs^ e)
    {
        UserControl::OnHandleCreated(e);
        if (!_initialized && this->Handle != IntPtr::Zero)
        {
            _native->Initialize(static_cast<HWND>(this->Handle.ToPointer()));
            _initialized = true;
        }
    }

    void OccViewerControl::OnResize(EventArgs^ e)
    {
        UserControl::OnResize(e);
        if (_initialized)
        {
            _native->Resize(this->Width, this->Height);
        }
    }

    void OccViewerControl::OnPaint(PaintEventArgs^ e)
    {
        UserControl::OnPaint(e);
        if (_initialized)
        {
            _native->Redraw();
        }
    }

    bool OccViewerControl::LoadStep(String^ path, bool append)
    {
        if (!_initialized)
            return false;

        std::wstring nativePath = msclr::interop::marshal_as<std::wstring>(path);
        return _native->LoadStep(nativePath, append);
    }

    void OccViewerControl::ClearScene()
    {
        if (_initialized)
            _native->ClearScene();
    }

    bool OccViewerControl::LoadRobotArm(cli::array<RobotPartInfo^>^ parts,
                                        cli::array<cli::array<int>^>^ axisToPartMap)
    {
        if (!_initialized)
            return false;

        std::vector<RobotPartDef> nativeParts(parts->Length);
        for (int i = 0; i < parts->Length; i++)
        {
            System::String^ fp = parts[i]->FilePath;
            nativeParts[i].filePath = msclr::interop::marshal_as<std::wstring>(fp);
            nativeParts[i].dh_a = parts[i]->DH_a;
            nativeParts[i].dh_alpha = parts[i]->DH_alpha;
            nativeParts[i].dh_d = parts[i]->DH_d;
            nativeParts[i].dh_theta = parts[i]->DH_theta;
            for (int j = 0; j < 6; j++)
                nativeParts[i].offset[j] = parts[i]->Offset[j];
            nativeParts[i].parentIdx = parts[i]->ParentIdx;
            nativeParts[i].colorR = parts[i]->ColorR;
            nativeParts[i].colorG = parts[i]->ColorG;
            nativeParts[i].colorB = parts[i]->ColorB;
        }

        std::vector<std::pair<int,int>> nativeMap;
        if (axisToPartMap != nullptr)
        {
            for (int i = 0; i < axisToPartMap->Length; i++)
            {
                int a = axisToPartMap[i][0];
                int b = axisToPartMap[i][1];
                nativeMap.push_back(std::make_pair(a, b));
            }
        }

        return _native->LoadRobotArm(nativeParts, nativeMap);
    }

    void OccViewerControl::SetJointAngle(int axisIndex, double angleDeg)
    {
        if (_initialized)
            _native->SetJointAngle(axisIndex, angleDeg);
    }

    void OccViewerControl::FitAllView()
    {
        if (_initialized)
            _native->FitAll();
    }

    void OccViewerControl::SetViewIso()
    {
        if (_initialized)
            _native->SetViewIso();
    }

    void OccViewerControl::SetViewTop()
    {
        if (_initialized)
            _native->SetViewTop();
    }

    void OccViewerControl::OnMouseDown(MouseEventArgs^ e)
    {
        UserControl::OnMouseDown(e);
        if (!this->Focused)
            this->Focus();
        if (_initialized)
        {
            _native->OnMouseDown(e->X, e->Y, static_cast<int>(e->Button));
        }
    }

    void OccViewerControl::OnMouseMove(MouseEventArgs^ e)
    {
        UserControl::OnMouseMove(e);
        if (_initialized)
        {
            _native->OnMouseMove(e->X, e->Y, static_cast<int>(e->Button));
        }
    }

    void OccViewerControl::OnMouseUp(MouseEventArgs^ e)
    {
        UserControl::OnMouseUp(e);
        if (_initialized)
        {
            _native->OnMouseUp();
        }
    }

    void OccViewerControl::OnMouseWheel(MouseEventArgs^ e)
    {
        UserControl::OnMouseWheel(e);
        if (_initialized)
        {
            _native->OnMouseWheel(e->Delta);
        }
    }

}
