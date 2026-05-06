#pragma once

struct HWND__;
using HWND = HWND__*;
struct RobotPartDef;

class NativeOccView
{
public:
	NativeOccView();
	~NativeOccView();

	void initialize(HWND hwnd);
	void resize(int width, int height);
	void redraw();

	[[nodiscard]] bool loadStep(const wchar_t* filePath, bool append);
	[[nodiscard]] bool loadRobotArm(const RobotPartDef* parts, int partCount,
									const int* axisToPartMap, int mapCount);
	void setJointAngle(int axisIndex, double angleDeg);
	void clearScene();
	void fitAll();
	void setViewIso();
	void setViewTop();

	void onMouseDown(int x, int y, int button);
	void onMouseMove(int x, int y, int buttonMask);
	void onMouseUp();
	void onMouseWheel(int delta);

private:
	void updateRobotTransforms();

	struct Impl;
	Impl* m_impl;
};
