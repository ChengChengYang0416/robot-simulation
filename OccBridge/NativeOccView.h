#pragma once

#include <windows.h>
#include <cstdint>
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

enum class MouseButton : int32_t {
	Left   = 1048576,
	Middle = 4194304
};

struct RobotPartDef
{
	std::wstring filePath;
	double dhA     = 0.0;
	double dhAlpha  = 0.0;
	double dhD      = 0.0;
	double dhTheta  = 0.0;
	double offset[6] = {};   // tx, ty, tz, rx_deg, ry_deg, rz_deg
	int32_t parentIdx = -1;  // parent DH frame index (-1 for root)
	int32_t colorR = 200, colorG = 200, colorB = 200;
};

class NativeOccView
{
public:
	NativeOccView();
	~NativeOccView();

	void initialize(HWND hwnd);
	void resize(int32_t width, int32_t height);
	void redraw();

	[[nodiscard]] bool loadStep(const std::wstring& filePath, bool append);
	[[nodiscard]] bool loadRobotArm(const std::vector<RobotPartDef>& parts,
									const std::vector<std::pair<int32_t, int32_t>>& axisToPartMap);
	void setJointAngle(int32_t axisIndex, double angleDeg);
	void clearScene();
	void fitAll();
	void setViewIso();
	void setViewTop();

	void onMouseDown(int32_t x, int32_t y, int32_t button);
	void onMouseMove(int32_t x, int32_t y, int32_t buttonMask);
	void onMouseUp();
	void onMouseWheel(int32_t delta);

private:
	void updateRobotTransforms();

	HWND m_hwnd = nullptr;
	int32_t m_lastX = 0;
	int32_t m_lastY = 0;
	bool m_isRotating = false;
	bool m_isPanning  = false;

	Handle(Aspect_DisplayConnection) m_displayConnection;
	Handle(OpenGl_GraphicDriver) m_graphicDriver;
	Handle(V3d_Viewer) m_viewer;
	Handle(V3d_View) m_view;
	Handle(AIS_InteractiveContext) m_context;
	std::vector<Handle(AIS_Shape)> m_shapes;

	// Robot arm state
	std::vector<RobotPartDef> m_partDefs;
	std::vector<TopoDS_Shape> m_originalShapes;
	std::vector<std::pair<int32_t, int32_t>> m_axisToPartMap;
	std::vector<double> m_jointAngles;
};
