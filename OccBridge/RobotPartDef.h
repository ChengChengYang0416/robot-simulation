#pragma once

#include <string>

struct RobotPartDef
{
	std::wstring filePath;
	double dhA      = 0.0;
	double dhAlpha  = 0.0;
	double dhD      = 0.0;
	double dhTheta  = 0.0;
	double offset[6] = {};   // tx, ty, tz, rx_deg, ry_deg, rz_deg
	int parentIdx = -1;      // parent DH frame index (-1 for root)
	int colorR = 200, colorG = 200, colorB = 200;
};
