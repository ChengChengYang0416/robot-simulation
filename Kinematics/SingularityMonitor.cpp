#include "SingularityMonitor.h"

#include "Jacobian.h"
#include "RobotKinematics.h"
#include "RobotPartDef.h"

#include <algorithm>
#include <cmath>

namespace OccBridge {

namespace {

[[nodiscard]] double computeArmReferenceLength( const RobotKinematics& kin ) noexcept
// L_ref ~= a1 + a2 + sqrt(a3^2 + d4^2). Used to normalise the arm sub-determinant
// so the dimensionless ratio compares across robots of different sizes. Falls back
// to 1.0 when the part list is too short to be meaningful (e.g. a non-6DOF chain
// loaded by mistake) so the ratio degrades gracefully instead of dividing by zero.
{
	const auto& axisMap = kin.axisToPartMap();
	const auto& parts   = kin.parts();
	if( parts.empty() || axisMap.empty() ) {
		return 1.0;
	}

	// Resolve axes 1..4 to part indices; the arm subspace depends only on axes 1-3
	// (and the d4 offset of the link driven by axis 4 for the elbow-to-wrist arm).
	int axisPart[ 4 ] = { -1, -1, -1, -1 };
	for( const auto& m : axisMap ) {
		const int axisIdx = m.first;
		const int partIdx = m.second;
		if( axisIdx >= 1 && axisIdx <= 4
			&& partIdx >= 0 && partIdx < static_cast<int>( parts.size() ) ) {
			axisPart[ axisIdx - 1 ] = partIdx;
		}
	}
	if( axisPart[ 0 ] < 0 || axisPart[ 1 ] < 0
		|| axisPart[ 2 ] < 0 || axisPart[ 3 ] < 0 ) {
		return 1.0;
	}

	const double a1 = parts[ axisPart[ 0 ] ].dhA;
	const double a2 = parts[ axisPart[ 1 ] ].dhA;
	const double a3 = parts[ axisPart[ 2 ] ].dhA;
	const double d4 = parts[ axisPart[ 3 ] ].dhD;
	const double L3 = std::sqrt( a3 * a3 + d4 * d4 );

	const double ref = std::fabs( a1 ) + std::fabs( a2 ) + L3;
	return ( ref > 1.0e-6 ) ? ref : 1.0;
}

[[nodiscard]] SingularityLevel pickLevel( double ratio,
										  double warn,
										  double critical ) noexcept
{
	if( ratio < critical ) {
		return SingularityLevel::Critical;
	}
	if( ratio < warn ) {
		return SingularityLevel::Warning;
	}
	return SingularityLevel::Normal;
}

[[nodiscard]] SingularityKind classify( SingularityLevel wristLvl,
										SingularityLevel armLvl ) noexcept
// Combined when both subspaces are non-Normal; otherwise the worse of the two
// dictates the kind. The arm-vs-shoulder/elbow distinction is left to the caller
// since it depends on whether the wrist centre lies on the z0 axis (cheap to
// test but tangential to the rank loss itself).
{
	const bool wristBad = ( wristLvl != SingularityLevel::Normal );
	const bool armBad   = ( armLvl   != SingularityLevel::Normal );
	if( wristBad && armBad ) {
		return SingularityKind::Combined;
	}
	if( wristBad ) {
		return SingularityKind::Wrist;
	}
	if( armBad ) {
		// Default to Elbow; a more nuanced classifier could check the wrist-
		// centre radial distance to z0 and report Shoulder when small. Kept as
		// Elbow here because shoulder singularities are far rarer in practice
		// for arms whose first link offsets the wrist away from z0 (the LA
		// series have a1 = 0 or 30 mm, which still puts WC off-axis once joint
		// 2/3 have any deflection).
		return SingularityKind::Elbow;
	}
	return SingularityKind::None;
}

}  // namespace

SingularityReport evaluate( RobotKinematics&             kin,
							const SingularityThresholds& thresholds ) noexcept
{
	SingularityReport report;

	Jacobian::Matrix6x6 J;
	if( !Jacobian::build( kin, J ) ) {
		return report;  // valid = false
	}

	// Full Jacobian determinant: |det(J)| is Yoshikawa's manipulability index
	// for a square Jacobian. Useful as a single scalar summary but its absolute
	// value depends on robot size and unit choice; the sub-block ratios below
	// are what drive the kind / level decisions.
	report.manipulability = Jacobian::manipulability( J );

	// Wrist sub-block: rotational rows (3..5) coupled to wrist columns (3..5).
	// For a Pieper-spherical-wrist arm this is the angular Jacobian of joints
	// 4-6 expressed in the world; its determinant factors to (something) *
	// sin(q5), which is exactly the textbook wrist singularity criterion.
	report.wristManipulability = std::fabs( Jacobian::subDeterminant3x3( J, 3, 3 ) );

	// Arm sub-block: linear rows (0..2) coupled to arm columns (0..2). Goes to
	// zero when joints 2-3 collinearise (elbow stretched / folded) or the wrist
	// centre falls on the joint-1 axis (shoulder).
	report.armManipulability = std::fabs( Jacobian::subDeterminant3x3( J, 0, 0 ) );

	const double refLength = computeArmReferenceLength( kin );
	const double refCubed  = refLength * refLength * refLength;

	report.wristRatio = report.wristManipulability;                       // already dimensionless
	report.armRatio   = ( refCubed > 1.0e-6 )
		? report.armManipulability / refCubed
		: report.armManipulability;

	const SingularityLevel wristLvl = pickLevel( report.wristRatio,
												 thresholds.wristWarn,
												 thresholds.wristCritical );
	const SingularityLevel armLvl   = pickLevel( report.armRatio,
												 thresholds.armWarn,
												 thresholds.armCritical );

	// Overall level = worse of the two sub-spaces. Cast through int because
	// the enum is scoped (no implicit ordering operator).
	report.level = ( static_cast<int>( wristLvl ) >= static_cast<int>( armLvl ) )
		? wristLvl
		: armLvl;
	report.kind  = classify( wristLvl, armLvl );
	report.valid = true;
	return report;
}

}  // namespace OccBridge
