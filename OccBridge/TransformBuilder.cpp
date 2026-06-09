#include "TransformBuilder.h"

#include <cmath>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace OccBridge::Transform {

gp_Trsf makeDh( double a, double alphaDeg, double d, double thetaDeg )
// Builds a homogeneous transformation from DH parameters,
// applying rotations in the order Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
{
	const double alphaRad = alphaDeg * kDegToRad;
	const double thetaRad = thetaDeg * kDegToRad;

	// Build each elementary transform separately; skipping near-zero ones avoids
	// accumulating floating-point noise in the final 4x4 matrix.
	gp_Trsf rz;
	if( std::abs( thetaRad ) > kEpsilon ) {
		rz.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ) ), thetaRad );
	}

	gp_Trsf tz;
	if( std::abs( d ) > kEpsilon ) {
		tz.SetTranslation( gp_Vec( 0, 0, d ) );
	}

	gp_Trsf tx;
	if( std::abs( a ) > kEpsilon ) {
		tx.SetTranslation( gp_Vec( a, 0, 0 ) );
	}

	gp_Trsf rx;
	if( std::abs( alphaRad ) > kEpsilon ) {
		rx.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 1, 0, 0 ) ), alphaRad );
	}

	// gp_Trsf::Multiply post-multiplies, so the call order below matches the
	// mathematical left-to-right product Rz * Tz * Tx * Rx.
	gp_Trsf result = rz;
	result.Multiply( tz );
	result.Multiply( tx );
	result.Multiply( rx );
	return result;
}

gp_Trsf makeOffset( double tx, double ty, double tz,
					double rxDeg, double ryDeg, double rzDeg )
// Builds a homogeneous transformation from offset parameters,
// applying rotations in the order Rx * Ry * Rz, then translation
{
	// Each CAD STEP file is authored in its own local frame which generally does
	// not coincide with the DH joint frame. The offset parameters describe the
	// fixed rigid transform that places the CAD geometry inside the joint frame.
	gp_Trsf rotX;
	if( std::abs( rxDeg ) > kEpsilon ) {
		rotX.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 1, 0, 0 ) ), rxDeg * kDegToRad );
	}

	gp_Trsf rotY;
	if( std::abs( ryDeg ) > kEpsilon ) {
		rotY.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 1, 0 ) ), ryDeg * kDegToRad );
	}

	gp_Trsf rotZ;
	if( std::abs( rzDeg ) > kEpsilon ) {
		rotZ.SetRotation( gp_Ax1( gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ) ), rzDeg * kDegToRad );
	}

	gp_Trsf trans;
	if( std::abs( tx ) > kEpsilon || std::abs( ty ) > kEpsilon || std::abs( tz ) > kEpsilon ) {
		trans.SetTranslation( gp_Vec( tx, ty, tz ) );
	}

	// T * Rz * Ry * Rx: cancels DH rotation, placing CAD in original orientation
	gp_Trsf result = trans;
	result.Multiply( rotZ );
	result.Multiply( rotY );
	result.Multiply( rotX );
	return result;
}

}  // namespace OccBridge::Transform
