#pragma once

#include <array>
#include <gp_Trsf.hxx>

namespace OccBridge {

[[nodiscard]] std::array<double, 6> solveTcpPose( const gp_Trsf& tcp );
// Decomposes a TCP transform into [x, y, z, rx, ry, rz] in mm and degrees.
// Euler convention: R = Rz(rz) * Ry(ry) * Rx(rx) (ZYX intrinsic == XYZ extrinsic),
// matching the rotation order used by Transform::makeOffset.
// Handles gimbal lock (ry near +/- 90 deg) by collapsing rz to 0.

}  // namespace OccBridge
