#include "Jacobian.h"

#include "RobotKinematics.h"

#include <algorithm>
#include <cmath>

namespace OccBridge::Jacobian {

namespace {

constexpr int kDof = 6;

struct Vec3
{
	double x;
	double y;
	double z;
};

[[nodiscard]] Vec3 cross( const Vec3& a, const Vec3& b ) noexcept
{
	return { a.y * b.z - a.z * b.y,
			 a.z * b.x - a.x * b.z,
			 a.x * b.y - a.y * b.x };
}

[[nodiscard]] Vec3 translationOf( const gp_Trsf& t ) noexcept
{
	return { t.Value( 1, 4 ), t.Value( 2, 4 ), t.Value( 3, 4 ) };
}

[[nodiscard]] Vec3 zAxisOf( const gp_Trsf& t ) noexcept
// Third column of the rotation block; for a DH joint this is the rotation axis.
{
	return { t.Value( 1, 3 ), t.Value( 2, 3 ), t.Value( 3, 3 ) };
}

}  // namespace

bool build( RobotKinematics& kin, Matrix6x6& outJ ) noexcept
{
	const auto& parts   = kin.parts();
	const auto& axisMap = kin.axisToPartMap();
	if( parts.empty() || axisMap.empty() ) {
		return false;
	}

	// Resolve 1-based axis index -> part index once; the rest of the routine
	// only does table look-ups so there is no heap allocation in the hot path.
	int axisPart[ kDof ];
	for( int i = 0; i < kDof; ++i ) {
		axisPart[ i ] = -1;
	}
	for( const auto& m : axisMap ) {
		const int axisIdx = m.first;
		const int partIdx = m.second;
		if( axisIdx >= 1 && axisIdx <= kDof
			&& partIdx >= 0 && partIdx < static_cast<int>( parts.size() ) ) {
			axisPart[ axisIdx - 1 ] = partIdx;
		}
	}
	for( int i = 0; i < kDof; ++i ) {
		if( axisPart[ i ] < 0 ) {
			return false;
		}
	}

	const auto& cum = kin.computeCumulative();
	if( cum.empty() ) {
		return false;
	}
	const Vec3 pTcp = translationOf( cum.back() );

	for( int i = 0; i < kDof; ++i ) {
		const int   partIdx   = axisPart[ i ];
		const int   parentIdx = parts[ partIdx ].parentIdx;
		gp_Trsf     jointFrame;  // identity when no parent (axis 1 at world origin)
		if( parentIdx >= 0 && parentIdx < static_cast<int>( cum.size() ) ) {
			jointFrame = cum[ parentIdx ];
		}
		const Vec3 zi = zAxisOf( jointFrame );
		const Vec3 pi = translationOf( jointFrame );
		const Vec3 r  = { pTcp.x - pi.x, pTcp.y - pi.y, pTcp.z - pi.z };
		const Vec3 jv = cross( zi, r );
		outJ.m[ 0 ][ i ] = jv.x;
		outJ.m[ 1 ][ i ] = jv.y;
		outJ.m[ 2 ][ i ] = jv.z;
		outJ.m[ 3 ][ i ] = zi.x;
		outJ.m[ 4 ][ i ] = zi.y;
		outJ.m[ 5 ][ i ] = zi.z;
	}
	return true;
}

double determinant( const Matrix6x6& J ) noexcept
// LU factorisation with partial pivoting in a local copy; det = sign * product
// of diagonal pivots. Cheaper than computing J*J^T then a 6x6 SPD det.
{
	double A[ kDof ][ kDof ];
	for( int r = 0; r < kDof; ++r ) {
		for( int c = 0; c < kDof; ++c ) {
			A[ r ][ c ] = J.m[ r ][ c ];
		}
	}

	double sign = 1.0;
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
			return 0.0;  // singular
		}
		if( pivotRow != k ) {
			for( int c = 0; c < kDof; ++c ) {
				std::swap( A[ k ][ c ], A[ pivotRow ][ c ] );
			}
			sign = -sign;
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
		}
	}

	double det = sign;
	for( int i = 0; i < kDof; ++i ) {
		det *= A[ i ][ i ];
	}
	return det;
}

double manipulability( const Matrix6x6& J ) noexcept
// For a square Jacobian sqrt(det(J*J^T)) = |det(J)|. Use the cheap form.
{
	return std::fabs( determinant( J ) );
}

double subDeterminant3x3( const Matrix6x6& J, int row0, int col0 ) noexcept
{
	if( row0 < 0 || col0 < 0 || row0 + 3 > kDof || col0 + 3 > kDof ) {
		return 0.0;
	}
	const double a = J.m[ row0     ][ col0     ];
	const double b = J.m[ row0     ][ col0 + 1 ];
	const double c = J.m[ row0     ][ col0 + 2 ];
	const double d = J.m[ row0 + 1 ][ col0     ];
	const double e = J.m[ row0 + 1 ][ col0 + 1 ];
	const double f = J.m[ row0 + 1 ][ col0 + 2 ];
	const double g = J.m[ row0 + 2 ][ col0     ];
	const double h = J.m[ row0 + 2 ][ col0 + 1 ];
	const double i = J.m[ row0 + 2 ][ col0 + 2 ];
	return a * ( e * i - f * h )
		 - b * ( d * i - f * g )
		 + c * ( d * h - e * g );
}

}  // namespace OccBridge::Jacobian
