#pragma once

#include <array>
#include <optional>
#include <utility>
#include <vector>
#include <gp_Trsf.hxx>
#include "RobotPartDef.h"

namespace OccBridge {

class RobotKinematics
{
public:
	void configure( std::vector<RobotPartDef> parts,
					std::vector<std::pair<int, int>> axisMap );
	// Stores part definitions and axis-to-part mapping, resets joint angles to zero,
	// and pre-allocates internal buffers so subsequent setJointAngle / computeCumulative
	// calls perform no heap allocation (Hot Path safe).

	void setJointAngle( int axisIdx, double deg );
	// Updates the joint angle (degrees) for axis [0..5]; out-of-range indices are ignored.

	[[nodiscard]] const std::vector<gp_Trsf>& computeCumulative();
	// Recomputes dhCumulative[i] = dhCumulative[parent] * dhLocal(i) into the pre-allocated
	// buffer and returns a const reference. Returns an empty vector if configure() has not
	// been called. Walks parts in array order, so parent indices must precede children.

	[[nodiscard]] gp_Trsf computeFinal( int partIdx ) const;
	// Returns dhCumulative[partIdx] * offset(partIdx). Caller must have invoked
	// computeCumulative() since the last setJointAngle(). Returns identity transform
	// when partIdx is out of range.

	[[nodiscard]] std::optional<gp_Trsf> tcpFrame() const;
	// Returns the last cumulative DH frame (assumed end-effector / TCP), or std::nullopt
	// if no parts are configured. Caller must have invoked computeCumulative() first.

	[[nodiscard]] const std::vector<RobotPartDef>& parts() const
	{
		return m_parts;
	}
	// Read-only access to the configured part list; useful for callers that need to apply
	// the computed transforms to external scene objects (e.g. AIS shapes).

private:
	std::vector<RobotPartDef> m_parts;
	std::vector<std::pair<int, int>> m_axisToPartMap;
	std::array<double, 6> m_jointAngles{};

	// Pre-allocated buffers reused by computeCumulative() to avoid heap allocation
	// on every setJointAngle() update. Sized once in configure().
	std::vector<double> m_partJointDelta;
	std::vector<gp_Trsf> m_dhCumulative;
};

}  // namespace OccBridge
