#include <msclr/marshal_cppstd.h>
#include <cstdint>
#include <string>
#include <cmath>

#include "NativeOccView.h"
#include <Quantity_Color.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kDegToRad = kPi / 180.0;
static constexpr double kEpsilon = 1e-10;
static constexpr double kZoomFactor = 1.15;

static gp_Trsf makeDhTransform(double a, double alphaDeg, double d, double thetaDeg)
{
	const double alphaRad = alphaDeg * kDegToRad;
	const double thetaRad = thetaDeg * kDegToRad;

	// Standard DH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
	gp_Trsf rz;
	if( std::abs(thetaRad) > kEpsilon ) {
		rz.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), thetaRad);
	}

	gp_Trsf tz;
	if( std::abs(d) > kEpsilon ) {
		tz.SetTranslation(gp_Vec(0, 0, d));
	}

	gp_Trsf tx;
	if( std::abs(a) > kEpsilon ) {
		tx.SetTranslation(gp_Vec(a, 0, 0));
	}

	gp_Trsf rx;
	if( std::abs(alphaRad) > kEpsilon ) {
		rx.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), alphaRad);
	}

	gp_Trsf result = rz;
	result.Multiply(tz);
	result.Multiply(tx);
	result.Multiply(rx);
	return result;
}

static gp_Trsf makeOffsetTransform(double tx, double ty, double tz,
								   double rxDeg, double ryDeg, double rzDeg)
{
	gp_Trsf rotX;
	if( std::abs(rxDeg) > kEpsilon ) {
		rotX.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), rxDeg * kDegToRad);
	}

	gp_Trsf rotY;
	if( std::abs(ryDeg) > kEpsilon ) {
		rotY.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), ryDeg * kDegToRad);
	}

	gp_Trsf rotZ;
	if( std::abs(rzDeg) > kEpsilon ) {
		rotZ.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), rzDeg * kDegToRad);
	}

	gp_Trsf trans;
	if( std::abs(tx) > kEpsilon || std::abs(ty) > kEpsilon || std::abs(tz) > kEpsilon ) {
		trans.SetTranslation(gp_Vec(tx, ty, tz));
	}

	// T * Rz * Ry * Rx: cancels DH rotation, placing CAD in original orientation
	gp_Trsf result = trans;
	result.Multiply(rotZ);
	result.Multiply(rotY);
	result.Multiply(rotX);
	return result;
}

static std::string wideToUtf8(const std::wstring& wide)
{
	const int32_t len = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
	std::string utf8(len - 1, '\0');
	WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, &utf8[0], len, nullptr, nullptr);
	return utf8;
}

NativeOccView::NativeOccView() = default;

NativeOccView::~NativeOccView()
{
	m_shapes.clear();
	m_context.Nullify();
	m_view.Nullify();
	m_viewer.Nullify();
	m_graphicDriver.Nullify();
	m_displayConnection.Nullify();
}

void NativeOccView::initialize(HWND hwnd)
{
	m_hwnd = hwnd;

	m_displayConnection = new Aspect_DisplayConnection();
	m_graphicDriver = new OpenGl_GraphicDriver(m_displayConnection);
	m_viewer = new V3d_Viewer(m_graphicDriver);
	m_viewer->SetDefaultLights();
	m_viewer->SetLightOn();
	m_context = new AIS_InteractiveContext(m_viewer);
	m_view = m_viewer->CreateView();

	Handle(WNT_Window) window = new WNT_Window(reinterpret_cast<Aspect_Handle>(m_hwnd));
	m_view->SetWindow(window);
	if( !window->IsMapped() ) {
		window->Map();
	}

	m_view->SetBackgroundColor(Quantity_NOC_GRAY30);
	m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08, V3d_ZBUFFER);
	m_view->MustBeResized();
	m_view->SetProj(V3d_XposYnegZpos);
	m_view->Redraw();
}

void NativeOccView::resize(int32_t /*width*/, int32_t /*height*/)
{
	if( !m_view.IsNull() ) {
		m_view->MustBeResized();
		m_view->Redraw();
	}
}

void NativeOccView::redraw()
{
	if( !m_view.IsNull() ) {
		m_view->Redraw();
	}
}

bool NativeOccView::loadStep(const std::wstring& filePath, bool append)
{
	if( m_context.IsNull() ) {
		return false;
	}

	if( !append ) {
		clearScene();
	}

	const std::string utf8Path = wideToUtf8(filePath);

	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile(utf8Path.c_str());
	if( status != IFSelect_RetDone ) {
		return false;
	}

	const Standard_Integer roots = reader.TransferRoots();
	if( roots <= 0 ) {
		return false;
	}

	TopoDS_Shape shape = reader.OneShape();
	if( shape.IsNull() ) {
		return false;
	}

	Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
	m_context->Display(aisShape, Standard_False);
	m_context->SetDisplayMode(aisShape, 1, Standard_False);
	m_shapes.push_back(aisShape);
	fitAll();
	return true;
}

bool NativeOccView::loadRobotArm(const std::vector<RobotPartDef>& parts,
								 const std::vector<std::pair<int32_t, int32_t>>& axisToPartMap)
{
	if( m_context.IsNull() ) {
		return false;
	}

	clearScene();

	m_partDefs = parts;
	m_axisToPartMap = axisToPartMap;
	m_jointAngles.assign(6, 0.0);

	const auto n = static_cast<int32_t>(parts.size());

	for (int32_t i = 0; i < n; ++i)
	{
		const std::string utf8Path = wideToUtf8(parts[i].filePath);

		STEPControl_Reader reader;
		const IFSelect_ReturnStatus status = reader.ReadFile(utf8Path.c_str());
		if( status != IFSelect_RetDone ) {
			m_originalShapes.emplace_back();
			m_shapes.push_back(Handle(AIS_Shape)());
			continue;
		}

		const Standard_Integer roots = reader.TransferRoots();
		if( roots <= 0 ) {
			m_originalShapes.emplace_back();
			m_shapes.push_back(Handle(AIS_Shape)());
			continue;
		}

		TopoDS_Shape shape = reader.OneShape();
		m_originalShapes.push_back(shape);

		Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
		const Quantity_Color qColor(
			parts[i].colorR / 255.0,
			parts[i].colorG / 255.0,
			parts[i].colorB / 255.0,
			Quantity_TOC_sRGB);
		m_context->Display(aisShape, 1, -1, Standard_False);
		m_context->SetColor(aisShape, qColor, Standard_False);
		m_shapes.push_back(aisShape);
	}

	updateRobotTransforms();
	fitAll();
	return !m_shapes.empty();
}

void NativeOccView::setJointAngle(int32_t axisIndex, double angleDeg)
{
	if( axisIndex < 0 || axisIndex >= 6 || m_jointAngles.size() != 6 ) {
		return;
	}

	m_jointAngles[axisIndex] = angleDeg;
	updateRobotTransforms();
}

void NativeOccView::updateRobotTransforms()
{
	if( m_context.IsNull() || m_partDefs.empty() ) {
		return;
	}

	const auto n = static_cast<int32_t>(m_partDefs.size());

	// Build joint angle delta per part
	std::vector<double> partJointDelta(n, 0.0);
	for (const auto& mapping : m_axisToPartMap)
	{
		const int32_t axisIdx = mapping.first;
		const int32_t partIdx = mapping.second;
		if( axisIdx >= 1 && axisIdx <= 6 && partIdx >= 0 && partIdx < n ) {
			partJointDelta[partIdx] = m_jointAngles[axisIdx - 1];
		}
	}

	// Compute cumulative DH transforms
	std::vector<gp_Trsf> dhCumulative(n);
	for (int32_t i = 0; i < n; ++i)
	{
		const double theta = m_partDefs[i].dhTheta + partJointDelta[i];
		gp_Trsf dhLocal = makeDhTransform(
			m_partDefs[i].dhA, m_partDefs[i].dhAlpha,
			m_partDefs[i].dhD, theta);

		const int32_t parent = m_partDefs[i].parentIdx;
		if( parent >= 0 && parent < n ) {
			dhCumulative[i] = dhCumulative[parent];
			dhCumulative[i].Multiply(dhLocal);
		} else {
			dhCumulative[i] = dhLocal;
		}
	}

	for (int32_t i = 0; i < n; ++i)
	{
		if( m_shapes[i].IsNull() ) {
			continue;
		}

		gp_Trsf offsetTrsf = makeOffsetTransform(
			m_partDefs[i].offset[0], m_partDefs[i].offset[1], m_partDefs[i].offset[2],
			m_partDefs[i].offset[3], m_partDefs[i].offset[4], m_partDefs[i].offset[5]);

		gp_Trsf finalTrsf = dhCumulative[i];
		finalTrsf.Multiply(offsetTrsf);

		m_shapes[i]->SetLocalTransformation(finalTrsf);
	}

	m_context->UpdateCurrentViewer();
}

void NativeOccView::clearScene()
{
	if( m_context.IsNull() ) {
		return;
	}

	for (const auto& shape : m_shapes)
	{
		if( !shape.IsNull() ) {
			m_context->Remove(shape, Standard_False);
		}
	}
	m_shapes.clear();
	m_originalShapes.clear();
	m_partDefs.clear();
	m_axisToPartMap.clear();
	m_jointAngles.clear();
	m_context->UpdateCurrentViewer();
	redraw();
}

void NativeOccView::fitAll()
{
	if( !m_view.IsNull() ) {
		m_view->FitAll();
		m_view->ZFitAll();
		m_view->Redraw();
	}
}

void NativeOccView::setViewIso()
{
	if( !m_view.IsNull() ) {
		m_view->SetProj(V3d_XposYnegZpos);
		fitAll();
	}
}

void NativeOccView::setViewTop()
{
	if( !m_view.IsNull() ) {
		m_view->SetProj(V3d_Zpos);
		fitAll();
	}
}

void NativeOccView::onMouseDown(int32_t x, int32_t y, int32_t button)
{
	m_lastX = x;
	m_lastY = y;

	if( button == static_cast<int32_t>(MouseButton::Left) ) {
		m_isRotating = true;
		if( !m_view.IsNull() ) {
			m_view->StartRotation(x, y);
		}
	} else if( button == static_cast<int32_t>(MouseButton::Middle) ) {
		m_isPanning = true;
	}
}

void NativeOccView::onMouseMove(int32_t x, int32_t y, int32_t /*buttonMask*/)
{
	if( m_view.IsNull() || m_context.IsNull() ) {
		return;
	}

	if( m_isRotating ) {
		m_view->Rotation(x, y);
	} else if( m_isPanning ) {
		m_view->Pan(x - m_lastX, m_lastY - y);
		m_lastX = x;
		m_lastY = y;
	} else {
		m_context->MoveTo(x, y, m_view, Standard_True);
	}
}

void NativeOccView::onMouseUp()
{
	m_isRotating = false;
	m_isPanning = false;
}

void NativeOccView::onMouseWheel(int32_t delta)
{
	if( m_view.IsNull() ) {
		return;
	}

	const double factor = (delta > 0) ? kZoomFactor : (1.0 / kZoomFactor);
	m_view->SetZoom(factor);
	m_view->Redraw();
}
