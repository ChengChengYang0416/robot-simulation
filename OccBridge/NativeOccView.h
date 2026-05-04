#pragma once

#include <windows.h>
#include <string>
#include <vector>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <gp_Trsf.hxx>
#include <TopLoc_Location.hxx>

struct RobotPartDef
{
    std::wstring filePath;
    double dh_a;
    double dh_alpha;
    double dh_d;
    double dh_theta;
    double offset[6];   // tx, ty, tz, rx_deg, ry_deg, rz_deg
    int parentIdx;       // parent DH frame index (-1 for root)
    int colorR, colorG, colorB;
};

class NativeOccView
{
public:
    NativeOccView();
    ~NativeOccView();

    void Initialize(HWND hwnd);
    void Resize(int width, int height);
    void Redraw();

    bool LoadStep(const std::wstring& filePath, bool append);
    bool LoadRobotArm(const std::vector<RobotPartDef>& parts,
                      const std::vector<std::pair<int,int>>& axisToPartMap);
    void SetJointAngle(int axisIndex, double angleDeg);
    void ClearScene();
    void FitAll();
    void SetViewIso();
    void SetViewTop();

    void OnMouseDown(int x, int y, int button);
    void OnMouseMove(int x, int y, int buttonMask);
    void OnMouseUp();
    void OnMouseWheel(int delta);

private:
    void UpdateRobotTransforms();

    HWND m_hwnd;
    int m_lastX;
    int m_lastY;
    bool m_isRotating;
    bool m_isPanning;

    Handle(Aspect_DisplayConnection) m_displayConnection;
    Handle(OpenGl_GraphicDriver) m_graphicDriver;
    Handle(V3d_Viewer) m_viewer;
    Handle(V3d_View) m_view;
    Handle(AIS_InteractiveContext) m_context;
    std::vector<Handle(AIS_Shape)> m_shapes;

    // Robot arm state
    std::vector<RobotPartDef> m_partDefs;
    std::vector<TopoDS_Shape> m_originalShapes;        // original un-transformed geometry
    std::vector<std::pair<int,int>> m_axisToPartMap;
    std::vector<double> m_jointAngles;
};
