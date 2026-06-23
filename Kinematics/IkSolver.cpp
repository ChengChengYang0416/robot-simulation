#include "IkSolver.h"

#include "Jacobian.h"
#include "RobotKinematics.h"
#include "TransformBuilder.h"

#include <algorithm>
#include <cmath>

namespace OccBridge {

namespace {

constexpr int kDof = 6;

struct Vec3
{
	double x;
	double y;
	double z;
};

[[nodiscard]] Vec3 translationOf( const gp_Trsf& t ) noexcept
{
	return { t.Value( 1, 4 ), t.Value( 2, 4 ), t.Value( 3, 4 ) };
}

void orientationError( const gp_Trsf& target, const gp_Trsf& current, double out[ 3 ] ) noexcept
// Computes the axis-angle representation of R_err = R_target * R_current^T and writes
// (axis * angle) into out. This is the standard so(3) error vector used by velocity-level
// IK; for small angles it degenerates smoothly to 0.5 * (R_err - R_err^T) skew.
{
	// R_err = R_target * R_current^T. gp_Trsf::Inverted() on a rigid transform inverts both
	// rotation and translation; we only need the rotation block so the translation column
	// of the product is ignored below.
	gp_Trsf currentInv = current.Inverted();
	gp_Trsf err        = target;
	err.Multiply( currentInv );

	const double r11 = err.Value( 1, 1 );
	const double r22 = err.Value( 2, 2 );
	const double r33 = err.Value( 3, 3 );
	const double r32 = err.Value( 3, 2 );
	const double r23 = err.Value( 2, 3 );
	const double r13 = err.Value( 1, 3 );
	const double r31 = err.Value( 3, 1 );
	const double r21 = err.Value( 2, 1 );
	const double r12 = err.Value( 1, 2 );

	double cosTheta = ( r11 + r22 + r33 - 1.0 ) * 0.5;
	cosTheta        = std::clamp( cosTheta, -1.0, 1.0 );
	const double theta = std::acos( cosTheta );

	if( theta < 1.0e-9 ) {
		// Near-zero rotation: first-order skew-symmetric extraction is numerically stable.
		out[ 0 ] = 0.5 * ( r32 - r23 );
		out[ 1 ] = 0.5 * ( r13 - r31 );
		out[ 2 ] = 0.5 * ( r21 - r12 );
		return;
	}

	// Near +/- pi the (2 sin theta) denominator collapses; switch to the symmetric form
	// that recovers the axis from the diagonal. Threshold chosen to keep > ~6 digits of
	// precision in the divisor.
	const double sinTheta = std::sin( theta );
	if( sinTheta < 1.0e-6 ) {
		// theta ~= pi: axis^2 components come from the diagonal of (R + I) / 2.
		const double xx = std::max( 0.0, ( r11 + 1.0 ) * 0.5 );
		const double yy = std::max( 0.0, ( r22 + 1.0 ) * 0.5 );
		const double zz = std::max( 0.0, ( r33 + 1.0 ) * 0.5 );
		double ax       = std::sqrt( xx );
		double ay       = std::sqrt( yy );
		double az       = std::sqrt( zz );
		// Disambiguate signs via the off-diagonal terms; pick the dominant component as
		// the reference and propagate signs from there.
		if( ax >= ay && ax >= az ) {
			ay = std::copysign( ay, r21 + r12 );
			az = std::copysign( az, r13 + r31 );
		} else if( ay >= ax && ay >= az ) {
			ax = std::copysign( ax, r21 + r12 );
			az = std::copysign( az, r32 + r23 );
		} else {
			ax = std::copysign( ax, r13 + r31 );
			ay = std::copysign( ay, r32 + r23 );
		}
		out[ 0 ] = theta * ax;
		out[ 1 ] = theta * ay;
		out[ 2 ] = theta * az;
		return;
	}

	const double k = theta / ( 2.0 * sinTheta );
	out[ 0 ]       = k * ( r32 - r23 );
	out[ 1 ]       = k * ( r13 - r31 );
	out[ 2 ]       = k * ( r21 - r12 );
}

[[nodiscard]] bool solve6x6( double A[ 6 ][ 6 ], double b[ 6 ], double x[ 6 ] ) noexcept
// In-place Gaussian elimination with partial pivoting; A and b are overwritten. Returns
// false only on a numerically singular pivot (should not happen with DLS damping > 0).
{
	int    perm[ kDof ];
	for( int i = 0; i < kDof; ++i ) {
		perm[ i ] = i;
	}

	for( int k = 0; k < kDof; ++k ) {
		int    pivotRow = k;
		double pivotMag = std::fabs( A[ k ][ k ] );
		for( int r = k + 1; r < kDof; ++r ) {
			const double mag = std::fabs( A[ r ][ k ] );
			if( mag > pivotMag ) {
				pivotMag = mag;
				pivotRow = r;
			}
		}
		if( pivotMag < 1.0e-18 ) {
			return false;
		}
		if( pivotRow != k ) {
			for( int c = 0; c < kDof; ++c ) {
				std::swap( A[ k ][ c ], A[ pivotRow ][ c ] );
			}
			std::swap( b[ k ], b[ pivotRow ] );
			std::swap( perm[ k ], perm[ pivotRow ] );
		}

		const double invPivot = 1.0 / A[ k ][ k ];
		for( int r = k + 1; r < kDof; ++r ) {
			const double factor = A[ r ][ k ] * invPivot;
			if( factor == 0.0 ) {
				continue;
			}
			for( int c = k; c < kDof; ++c ) {
				A[ r ][ c ] -= factor * A[ k ][ c ];
			}
			b[ r ] -= factor * b[ k ];
		}
	}

	for( int i = kDof - 1; i >= 0; --i ) {
		double sum = b[ i ];
		for( int c = i + 1; c < kDof; ++c ) {
			sum -= A[ i ][ c ] * x[ c ];
		}
		x[ i ] = sum / A[ i ][ i ];
	}
	return true;
}

}  // namespace

IkResult solveIkDls( RobotKinematics&             kin,
					 const gp_Trsf&               targetTcp,
					 const std::array<double, 6>& seedAnglesDeg,
					 const IkOptions&             options )
{
	IkResult result;
	result.jointAnglesDeg = seedAnglesDeg;

	const auto& parts = kin.parts();
	const auto& axisMap = kin.axisToPartMap();
	if( parts.empty() || axisMap.empty() ) {
		return result;  // status defaults to InvalidConfiguration
	}

	const Vec3 pTarget = translationOf( targetTcp );
	const double lambdaSq = options.damping * options.damping;

	std::array<double, kDof> q = seedAnglesDeg;
	if( options.useJointLimits ) {
		for( int i = 0; i < kDof; ++i ) {
			q[ i ] = std::clamp( q[ i ], options.jointMinDeg[ i ], options.jointMaxDeg[ i ] );
		}
	}

	for( int iter = 0; iter < options.maxIterations; ++iter ) {
		// 1. Forward kinematics + Jacobian: push q into kin, then delegate the FK
		//    cache update and the 6x6 J construction to the shared Jacobian module.
		for( int i = 0; i < kDof; ++i ) {
			kin.setJointAngle( i, q[ i ] );
		}
		Jacobian::Matrix6x6 Jmat;
		if( !Jacobian::build( kin, Jmat ) ) {
			return result;  // configuration became invalid mid-flight
		}

		// FK output is needed once more for the pose error term; the Jacobian
		// builder has already populated the cumulative-frame cache via
		// computeCumulative(), so this is just a const re-fetch (no recompute).
		const auto& cum = kin.computeCumulative();
		const gp_Trsf& tcpTrsf = cum.back();
		const Vec3     pTcp    = translationOf( tcpTrsf );

		// 2. Pose error e (6) = [position err mm ; orientation err rad].
		double e[ kDof ] = { pTarget.x - pTcp.x,
							 pTarget.y - pTcp.y,
							 pTarget.z - pTcp.z,
							 0.0, 0.0, 0.0 };
		double ori[ 3 ];
		orientationError( targetTcp, tcpTrsf, ori );
		e[ 3 ] = ori[ 0 ];
		e[ 4 ] = ori[ 1 ];
		e[ 5 ] = ori[ 2 ];

		const double posErr = std::sqrt( e[ 0 ] * e[ 0 ] + e[ 1 ] * e[ 1 ] + e[ 2 ] * e[ 2 ] );
		const double oriErr = std::sqrt( e[ 3 ] * e[ 3 ] + e[ 4 ] * e[ 4 ] + e[ 5 ] * e[ 5 ] );

		result.iterations          = iter;
		result.positionErrorMm     = posErr;
		result.orientationErrorRad = oriErr;
		result.jointAnglesDeg      = q;

		if( posErr < options.positionToleranceMm && oriErr < options.orientationToleranceRad ) {
			result.status = IkStatus::Converged;
			return result;
		}

		// 3. A = J*J^T + lambda^2 * I (6x6 symmetric positive-definite for lambda > 0).
		double A[ kDof ][ kDof ];
		for( int r = 0; r < kDof; ++r ) {
			for( int c = 0; c < kDof; ++c ) {
				double sum = 0.0;
				for( int k = 0; k < kDof; ++k ) {
					sum += Jmat.m[ r ][ k ] * Jmat.m[ c ][ k ];
				}
				A[ r ][ c ] = sum;
			}
			A[ r ][ r ] += lambdaSq;
		}

		double rhs[ kDof ];
		double y[ kDof ];
		for( int i = 0; i < kDof; ++i ) {
			rhs[ i ] = e[ i ];
		}
		if( !solve6x6( A, rhs, y ) ) {
			result.status = IkStatus::MaxIterations;
			return result;
		}

		// 4. delta_q (rad) = J^T * y, clamp per-axis to maxStepDeg, accumulate in degrees.
		for( int i = 0; i < kDof; ++i ) {
			double dq = 0.0;
			for( int r = 0; r < kDof; ++r ) {
				dq += Jmat.m[ r ][ i ] * y[ r ];
			}
			double dqDeg = dq / Transform::kDegToRad;
			if( dqDeg > options.maxStepDeg ) {
				dqDeg = options.maxStepDeg;
			} else if( dqDeg < -options.maxStepDeg ) {
				dqDeg = -options.maxStepDeg;
			}
			q[ i ] += dqDeg;
			if( options.useJointLimits ) {
				q[ i ] = std::clamp( q[ i ], options.jointMinDeg[ i ], options.jointMaxDeg[ i ] );
			}
		}
	}

	// Loop fell through without satisfying tolerances; q still holds the best iterate.
	result.iterations     = options.maxIterations;
	result.jointAnglesDeg = q;
	result.status         = IkStatus::MaxIterations;
	return result;
}

}  // namespace OccBridge
