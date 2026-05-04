#include <msclr/marshal_cppstd.h>
#include <string>
#include <cmath>

#include "NativeOccView.h"
#include <Quantity_Color.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static gp_Trsf MakeDHTransform(double a, double alpha_deg, double d, double theta_deg)
{
    double alpha_rad = alpha_deg * M_PI / 180.0;
    double theta_rad = theta_deg * M_PI / 180.0;

    // Standard DH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    gp_Trsf rz;
    if (std::abs(theta_rad) > 1e-10)
        rz.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), theta_rad);

    gp_Trsf tz;
    if (std::abs(d) > 1e-10)
        tz.SetTranslation(gp_Vec(0, 0, d));

    gp_Trsf tx;
    if (std::abs(a) > 1e-10)
        tx.SetTranslation(gp_Vec(a, 0, 0));

    gp_Trsf rx;
    if (std::abs(alpha_rad) > 1e-10)
        rx.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), alpha_rad);

    gp_Trsf result = rz;
    result.Multiply(tz);
    result.Multiply(tx);
    result.Multiply(rx);
    return result;
}

static gp_Trsf MakeOffsetTransform(double tx, double ty, double tz,
    double rx_deg, double ry_deg, double rz_deg)
{
    gp_Trsf rotX;
    if (std::abs(rx_deg) > 1e-10)
        rotX.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), rx_deg * M_PI / 180.0);

    gp_Trsf rotY;
    if (std::abs(ry_deg) > 1e-10)
        rotY.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), ry_deg * M_PI / 180.0);

    gp_Trsf rotZ;
    if (std::abs(rz_deg) > 1e-10)
        rotZ.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), rz_deg * M_PI / 180.0);

    gp_Trsf trans;
    if (std::abs(tx) > 1e-10 || std::abs(ty) > 1e-10 || std::abs(tz) > 1e-10)
        trans.SetTranslation(gp_Vec(tx, ty, tz));

    // T * Rz * Ry * Rx: cancels DH rotation, placing CAD in original orientation
    gp_Trsf result = trans;
    result.Multiply(rotZ);
    result.Multiply(rotY);
    result.Multiply(rotX);
    return result;
}

static std::string WideToUtf8(const std::wstring& wide)
{
    int len = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string utf8(len - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, &utf8[0], len, nullptr, nullptr);
    return utf8;
}

NativeOccView::NativeOccView()
    : m_hwnd(nullptr), m_lastX(0), m_lastY(0), m_isRotating(false), m_isPanning(false)
{
}

NativeOccView::~NativeOccView()
{
    m_shapes.clear();
    m_context.Nullify();
    m_view.Nullify();
    m_viewer.Nullify();
    m_graphicDriver.Nullify();
    m_displayConnection.Nullify();
}

void NativeOccView::Initialize(HWND hwnd)
{
    m_hwnd = hwnd;

    m_displayConnection = new Aspect_DisplayConnection();
    m_graphicDriver = new OpenGl_GraphicDriver(m_displayConnection);
    m_viewer = new V3d_Viewer(m_graphicDriver);
    m_viewer->SetDefaultLights();
    m_viewer->SetLightOn();
    m_context = new AIS_InteractiveContext(m_viewer);
    m_view = m_viewer->CreateView();

    Handle(WNT_Window) window = new WNT_Window((Aspect_Handle)m_hwnd);
    m_view->SetWindow(window);
    if (!window->IsMapped())
    {
        window->Map();
    }

    m_view->SetBackgroundColor(Quantity_NOC_GRAY30);
    m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08, V3d_ZBUFFER);
    m_view->MustBeResized();
    m_view->SetProj(V3d_XposYnegZpos);
    m_view->Redraw();
}

void NativeOccView::Resize(int, int)
{
    if (!m_view.IsNull())
    {
        m_view->MustBeResized();
        m_view->Redraw();
    }
}

void NativeOccView::Redraw()
{
    if (!m_view.IsNull())
    {
        m_view->Redraw();
    }
}

bool NativeOccView::LoadStep(const std::wstring& filePath, bool append)
{
    if (m_context.IsNull())
    {
        return false;
    }

    if (!append)
    {
        ClearScene();
    }

    std::string utf8Path = WideToUtf8(filePath);

    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(utf8Path.c_str());
    if (status != IFSelect_RetDone)
    {
        return false;
    }

    Standard_Integer roots = reader.TransferRoots();
    if (roots <= 0)
    {
        return false;
    }

    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull())
    {
        return false;
    }

    Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
    m_context->Display(aisShape, Standard_False);
    m_context->SetDisplayMode(aisShape, 1, Standard_False); // AIS_Shaded
    m_shapes.push_back(aisShape);
    FitAll();
    return true;
}

bool NativeOccView::LoadRobotArm(const std::vector<RobotPartDef>& parts,
                                 const std::vector<std::pair<int,int>>& axisToPartMap)
{
    if (m_context.IsNull())
        return false;

    ClearScene();

    m_partDefs = parts;
    m_axisToPartMap = axisToPartMap;
    m_jointAngles.assign(6, 0.0);

    int n = static_cast<int>(parts.size());

    // Load all STEP files and create AIS_Shape objects
    for (int i = 0; i < n; i++)
    {
        std::string utf8Path = WideToUtf8(parts[i].filePath);

        STEPControl_Reader reader;
        IFSelect_ReturnStatus status = reader.ReadFile(utf8Path.c_str());
        if (status != IFSelect_RetDone)
        {
            m_originalShapes.push_back(TopoDS_Shape());
            m_shapes.push_back(Handle(AIS_Shape)());
            continue;
        }

        Standard_Integer roots = reader.TransferRoots();
        if (roots <= 0)
        {
            m_originalShapes.push_back(TopoDS_Shape());
            m_shapes.push_back(Handle(AIS_Shape)());
            continue;
        }

        TopoDS_Shape shape = reader.OneShape();
        m_originalShapes.push_back(shape);

        Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
        Quantity_Color qColor(
            parts[i].colorR / 255.0,
            parts[i].colorG / 255.0,
            parts[i].colorB / 255.0,
            Quantity_TOC_sRGB);
        m_context->Display(aisShape, 1, -1, Standard_False);
        m_context->SetColor(aisShape, qColor, Standard_False);
        m_shapes.push_back(aisShape);
    }

    UpdateRobotTransforms();
    FitAll();
    return !m_shapes.empty();
}

void NativeOccView::SetJointAngle(int axisIndex, double angleDeg)
{
    if (axisIndex < 0 || axisIndex >= 6 || m_jointAngles.size() != 6)
        return;

    m_jointAngles[axisIndex] = angleDeg;
    UpdateRobotTransforms();
}

void NativeOccView::UpdateRobotTransforms()
{
    if (m_context.IsNull() || m_partDefs.empty())
        return;

    int n = static_cast<int>(m_partDefs.size());

    // Build joint angle delta per part
    std::vector<double> partJointDelta(n, 0.0);
    for (const auto& mapping : m_axisToPartMap)
    {
        int axisIdx = mapping.first;
        int partIdx = mapping.second;
        if (axisIdx >= 1 && axisIdx <= 6 && partIdx >= 0 && partIdx < n)
        {
            partJointDelta[partIdx] = m_jointAngles[axisIdx - 1];
        }
    }

    // Compute cumulative DH transforms
    std::vector<gp_Trsf> dhCumulative(n);
    for (int i = 0; i < n; i++)
    {
        double theta = m_partDefs[i].dh_theta + partJointDelta[i];
        gp_Trsf dhLocal = MakeDHTransform(
            m_partDefs[i].dh_a, m_partDefs[i].dh_alpha,
            m_partDefs[i].dh_d, theta);

        int parent = m_partDefs[i].parentIdx;
        if (parent >= 0 && parent < n)
        {
            dhCumulative[i] = dhCumulative[parent];
            dhCumulative[i].Multiply(dhLocal);
        }
        else
        {
            dhCumulative[i] = dhLocal;
        }
    }

    // Update each shape: swap underlying TopoDS_Shape with located version
    for (int i = 0; i < n; i++)
    {
        if (m_shapes[i].IsNull())
            continue;

        gp_Trsf offsetTrsf = MakeOffsetTransform(
            m_partDefs[i].offset[0], m_partDefs[i].offset[1], m_partDefs[i].offset[2],
            m_partDefs[i].offset[3], m_partDefs[i].offset[4], m_partDefs[i].offset[5]);

        gp_Trsf finalTrsf = dhCumulative[i];
        finalTrsf.Multiply(offsetTrsf);

        // Update the transformation on the AIS object directly
        m_shapes[i]->SetLocalTransformation(finalTrsf);
    }

    m_context->UpdateCurrentViewer();
}

void NativeOccView::ClearScene()
{
    if (m_context.IsNull())
    {
        return;
    }

    for (const auto& shape : m_shapes)
    {
        if (!shape.IsNull())
            m_context->Remove(shape, Standard_False);
    }
    m_shapes.clear();
    m_originalShapes.clear();
    m_partDefs.clear();
    m_axisToPartMap.clear();
    m_jointAngles.clear();
    m_context->UpdateCurrentViewer();
    Redraw();
}

void NativeOccView::FitAll()
{
    if (!m_view.IsNull())
    {
        m_view->FitAll();
        m_view->ZFitAll();
        m_view->Redraw();
    }
}

void NativeOccView::SetViewIso()
{
    if (!m_view.IsNull())
    {
        m_view->SetProj(V3d_XposYnegZpos);
        FitAll();
    }
}

void NativeOccView::SetViewTop()
{
    if (!m_view.IsNull())
    {
        m_view->SetProj(V3d_Zpos);
        FitAll();
    }
}

void NativeOccView::OnMouseDown(int x, int y, int button)
{
    m_lastX = x;
    m_lastY = y;

    if (button == 1048576) // Left
    {
        m_isRotating = true;
        if (!m_view.IsNull())
        {
            m_view->StartRotation(x, y);
        }
    }
    else if (button == 4194304) // Middle
    {
        m_isPanning = true;
    }
}

void NativeOccView::OnMouseMove(int x, int y, int)
{
    if (m_view.IsNull() || m_context.IsNull())
    {
        return;
    }

    if (m_isRotating)
    {
        m_view->Rotation(x, y);
    }
    else if (m_isPanning)
    {
        m_view->Pan(x - m_lastX, m_lastY - y);
        m_lastX = x;
        m_lastY = y;
    }
    else
    {
        m_context->MoveTo(x, y, m_view, Standard_True);
    }
}

void NativeOccView::OnMouseUp()
{
    m_isRotating = false;
    m_isPanning = false;
}

void NativeOccView::OnMouseWheel(int delta)
{
    if (m_view.IsNull())
        return;

    double factor = (delta > 0) ? 1.15 : (1.0 / 1.15);
    m_view->SetZoom(factor);
    m_view->Redraw();
}
