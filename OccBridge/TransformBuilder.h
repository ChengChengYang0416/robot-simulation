#pragma once

#include <gp_Trsf.hxx>

namespace OccBridge::Transform {

inline constexpr double kPi      = 3.14159265358979323846;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kEpsilon  = 1e-10;

[[nodiscard]] gp_Trsf makeDh( double a, double alphaDeg, double d, double thetaDeg );
// Builds a homogeneous DH transform using the standard convention:
//   T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
//   theta : joint rotation around previous Z axis
//   d     : link offset along previous Z axis
//   a     : link length along common normal (new X axis)
//   alpha : link twist around new X axis

[[nodiscard]] gp_Trsf makeOffset( double tx, double ty, double tz,
								  double rxDeg, double ryDeg, double rzDeg );
// Builds a rigid offset transform that places the CAD-authored frame into
// the DH joint frame. Rotation order is Rz * Ry * Rx, then translation.

}  // namespace OccBridge::Transform
