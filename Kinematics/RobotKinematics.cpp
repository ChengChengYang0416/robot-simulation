#include "RobotKinematics.h"
#include "TransformBuilder.h"
#include <algorithm>

namespace OccBridge {

void RobotKinematics::configure( std::vector<RobotPartDef> parts,
								 std::vector<std::pair<int, int>> axisMap )
// Moves the inputs into members and resizes the cumulative-transform and joint-delta
// buffers exactly once. After this call, computeCumulative() reuses the same storage.
{
	m_parts = std::move( parts );
	m_axisToPartMap = std::move( axisMap );
	m_jointAngles.fill( 0.0 );

	m_partJointDelta.assign( m_parts.size(), 0.0 );
	m_dhCumulative.assign( m_parts.size(), gp_Trsf() );
}

void RobotKinematics::setJointAngle( int axisIdx, double deg )
// Updates a single joint angle; the actual transform recomputation is deferred until
// computeCumulative() so callers can batch multiple updates before recomputing.
{
	if( axisIdx < 0 || axisIdx >= 6 ) {
		return;
	}
	m_jointAngles[ axisIdx ] = deg;
}

const std::vector<gp_Trsf>& RobotKinematics::computeCumulative()
// Two-pass forward kinematics:
//   1. Spread joint angles (axis 1..6) onto the part they drive; non-driven parts keep 0.
//   2. Walk parts in array order, accumulating each DH transform onto its parent's frame.
// All work is done in pre-allocated buffers; no heap allocation in the steady state.
{
	const int n = static_cast<int>( m_parts.size() );

	std::fill( m_partJointDelta.begin(), m_partJointDelta.end(), 0.0 );
	for( const auto& mapping : m_axisToPartMap ) {
		const int axisIdx = mapping.first;
		const int partIdx = mapping.second;
		if( axisIdx >= 1 && axisIdx <= 6 && partIdx >= 0 && partIdx < n ) {
			m_partJointDelta[ partIdx ] = m_jointAngles[ axisIdx - 1 ];
		}
	}

	for( int i = 0; i < n; ++i ) {
		const double theta = m_parts[ i ].dhTheta + m_partJointDelta[ i ];
		gp_Trsf dhLocal = Transform::makeDh(
			m_parts[ i ].dhA, m_parts[ i ].dhAlpha,
			m_parts[ i ].dhD, theta );

		const int parent = m_parts[ i ].parentIdx;
		if( parent >= 0 && parent < n ) {
			m_dhCumulative[ i ] = m_dhCumulative[ parent ];
			m_dhCumulative[ i ].Multiply( dhLocal );
		} else {
			m_dhCumulative[ i ] = dhLocal;
		}
	}

	return m_dhCumulative;
}

gp_Trsf RobotKinematics::computeFinal( int partIdx ) const
// Returns the world transform of a single part: cumulative DH frame composed with the
// CAD-authored offset that aligns the shape's authoring frame to the DH joint frame.
{
	if( partIdx < 0 || partIdx >= static_cast<int>( m_dhCumulative.size() ) ) {
		return {};
	}

	const auto& part = m_parts[ partIdx ];
	gp_Trsf offsetTrsf = Transform::makeOffset(
		part.offset[ 0 ], part.offset[ 1 ], part.offset[ 2 ],
		part.offset[ 3 ], part.offset[ 4 ], part.offset[ 5 ] );

	gp_Trsf finalTrsf = m_dhCumulative[ partIdx ];
	finalTrsf.Multiply( offsetTrsf );
	return finalTrsf;
}

std::optional<gp_Trsf> RobotKinematics::tcpFrame() const
// The TCP is conventionally the last DH frame in the chain; callers query this after
// computeCumulative() to update tool-center-point visualization or report pose.
{
	if( m_dhCumulative.empty() ) {
		return std::nullopt;
	}
	return m_dhCumulative.back();
}

}  // namespace OccBridge
