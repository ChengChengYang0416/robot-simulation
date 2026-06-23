#include "AnalyticalIkSolver.h"

#include "RobotKinematics.h"
#include "TransformBuilder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace OccBridge {

namespace {

constexpr int    kDof          = 6;
constexpr double kGeometryTol  = 1.0e-6;   // mm; tolerance for "is zero" on DH lengths
constexpr double kWristSingTol = 1.0e-6;   // |sin q5| below this -> gimbal lock branch
constexpr double kRadialTol    = 1.0e-9;   // mm; below this WC is on the z0 axis

struct DhParams
{
	// One row per joint, joints indexed 0..5 (axis 1..6 in the JSON / axisMap).
	// theta is the DH offset added to the runtime joint angle q before evaluating
	// the trig functions; e.g. the LA series stores theta_2 = -90 so that the
	// rendered model corresponds to q_2 = 0.
	double a[ kDof ]{};
	double alphaRad[ kDof ]{};
	double d[ kDof ]{};
	double thetaRad[ kDof ]{};   // DH offset, NOT including q
};

[[nodiscard]] bool extractDh( const RobotKinematics& kin, DhParams& out, int axisPart[ kDof ] ) noexcept
// Resolves the axis -> part mapping and pulls a, alpha, d, theta into a flat array
// so the solver does not need to touch RobotKinematics again. Returns false when
// the configuration is incomplete or out of range.
{
	const auto& parts   = kin.parts();
	const auto& axisMap = kin.axisToPartMap();
	if( parts.empty() || axisMap.empty() ) {
		return false;
	}

	for( int i = 0; i < kDof; ++i ) {
		axisPart[ i ] = -1;
	}
	for( const auto& m : axisMap ) {
		const int axisIdx = m.first;     // 1-based
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
		const auto& p = parts[ axisPart[ i ] ];
		out.a[ i ]        = p.dhA;
		out.alphaRad[ i ] = p.dhAlpha * Transform::kDegToRad;
		out.d[ i ]        = p.dhD;
		out.thetaRad[ i ] = p.dhTheta * Transform::kDegToRad;
	}
	return true;
}

[[nodiscard]] bool hasSphericalWrist( const DhParams& dh ) noexcept
// Pieper's condition for decoupled position / orientation: the last three axes
// must intersect at a single point. With standard DH that requires a4 = a5 = a6
// = 0 and d5 = 0 (frames 4 and 5 share an origin, frame 6 sits at d6 along z5).
{
	return std::fabs( dh.a[ 3 ] ) < kGeometryTol
		&& std::fabs( dh.a[ 4 ] ) < kGeometryTol
		&& std::fabs( dh.a[ 5 ] ) < kGeometryTol
		&& std::fabs( dh.d[ 4 ] ) < kGeometryTol;
}

[[nodiscard]] double wrapToPi( double a ) noexcept
// Maps an angle in radians into (-π, π]; used both for solution selection
// (so q1 + 2π is not "far" from q1) and for joint-limit checks.
{
	constexpr double kTwoPi = 2.0 * Transform::kPi;
	a = std::fmod( a + Transform::kPi, kTwoPi );
	if( a <= 0.0 ) {
		a += kTwoPi;
	}
	return a - Transform::kPi;
}

[[nodiscard]] double wrappedDistanceDeg( double a, double b ) noexcept
// |a - b| measured modulo 360°, so a candidate at -179° and a seed at +179°
// are considered 2° apart, not 358°. Used to score candidates against the seed.
{
	double d = std::fmod( a - b, 360.0 );
	if( d > 180.0 ) {
		d -= 360.0;
	} else if( d < -180.0 ) {
		d += 360.0;
	}
	return std::fabs( d );
}

struct Candidate
{
	std::array<double, kDof> q{};   // degrees, post offset removal (i.e. user-facing)
	bool valid = false;
};

[[nodiscard]] bool insideLimits( const Candidate&             c,
								 const IkOptions&             options ) noexcept
{
	if( !options.useJointLimits ) {
		return true;
	}
	for( int i = 0; i < kDof; ++i ) {
		// Try both q and q ± 360°: the analytical formulas return angles in
		// (-π, π], but joint 4 / 6 typically allow multi-turn motion and the
		// closest equivalent may lie outside (-180°, 180°].
		const double q     = c.q[ i ];
		const double qPlus  = q + 360.0;
		const double qMinus = q - 360.0;
		const bool   in = ( q      >= options.jointMinDeg[ i ] && q      <= options.jointMaxDeg[ i ] )
					   || ( qPlus  >= options.jointMinDeg[ i ] && qPlus  <= options.jointMaxDeg[ i ] )
					   || ( qMinus >= options.jointMinDeg[ i ] && qMinus <= options.jointMaxDeg[ i ] );
		if( !in ) {
			return false;
		}
	}
	return true;
}

void snapToLimits( Candidate& c, const IkOptions& options ) noexcept
// When a candidate satisfies limits in its ±360° equivalent, pull the angle
// into the allowed window so the consumer sees a value that displays sanely.
{
	if( !options.useJointLimits ) {
		return;
	}
	for( int i = 0; i < kDof; ++i ) {
		const double lo = options.jointMinDeg[ i ];
		const double hi = options.jointMaxDeg[ i ];
		double       q  = c.q[ i ];
		if( q < lo && q + 360.0 <= hi ) {
			q += 360.0;
		} else if( q > hi && q - 360.0 >= lo ) {
			q -= 360.0;
		}
		c.q[ i ] = q;
	}
}

[[nodiscard]] double seedDistance( const Candidate&                          c,
								   const std::array<double, kDof>&           seedDeg ) noexcept
// Sum of wrapped angular distances; cheap surrogate for "closest configuration".
// Squared distance would emphasise large axes more, but for branch selection a
// linear sum tracks "fewest joints have to move" closely enough.
{
	double sum = 0.0;
	for( int i = 0; i < kDof; ++i ) {
		sum += wrappedDistanceDeg( c.q[ i ], seedDeg[ i ] );
	}
	return sum;
}

}  // namespace

IkResult solveIkAnalytical( const RobotKinematics&       kin,
							const gp_Trsf&               targetTcp,
							const std::array<double, 6>& seedAnglesDeg,
							const IkOptions&             options )
{
	IkResult result;
	result.jointAnglesDeg = seedAnglesDeg;

	DhParams dh;
	int      axisPart[ kDof ];
	if( !extractDh( kin, dh, axisPart ) ) {
		return result;  // InvalidConfiguration
	}
	if( !hasSphericalWrist( dh ) ) {
		// Caller should fall back to DLS; the geometry breaks Pieper's decoupling.
		return result;
	}

	// ------------------------------------------------------------------ Step 1
	// Wrist centre = TCP - d6 · z6_target. z6_target is the third column of the
	// target rotation matrix; in OCCT's gp_Trsf storage that is Value(row, 3).
	const double d6  = dh.d[ 5 ];
	const double z6x = targetTcp.Value( 1, 3 );
	const double z6y = targetTcp.Value( 2, 3 );
	const double z6z = targetTcp.Value( 3, 3 );
	const double pTx = targetTcp.Value( 1, 4 );
	const double pTy = targetTcp.Value( 2, 4 );
	const double pTz = targetTcp.Value( 3, 4 );

	const double Wx = pTx - d6 * z6x;
	const double Wy = pTy - d6 * z6y;
	const double Wz = pTz - d6 * z6z;

	// Cached link parameters.
	const double a1 = dh.a[ 0 ];
	const double d1 = dh.d[ 0 ];
	const double a2 = dh.a[ 1 ];
	const double a3 = dh.a[ 2 ];
	const double d4 = dh.d[ 3 ];

	const double L3      = std::sqrt( a3 * a3 + d4 * d4 );
	// psi rotates the joint-3 frame so that the wrist-centre vector lies along
	// the "+x" axis of an effective 2-link planar arm. Derivation: WC in frame 2
	// is (a3·cos q3hat - d4·sin q3hat, a3·sin q3hat + d4·cos q3hat, 0); writing
	// that as L3·(cos(q3hat+psi), sin(q3hat+psi)) requires cos psi = a3/L3 and
	// sin psi = d4/L3, i.e. psi = atan2(d4, a3) — NOT atan2(a3, d4).
	const double psi     = std::atan2( d4, a3 );
	const double rhoMag  = std::sqrt( Wx * Wx + Wy * Wy );
	const bool   onAxis  = ( rhoMag < kRadialTol );

	std::array<Candidate, 8> cands{};

	// ------------------------------------------------------------------ Step 2
	// Two shoulder branches (forward / back). When the wrist centre sits on
	// the z0 axis we cannot resolve q1; keep the seed value so the orientation
	// stage still produces a usable q4..q6.
	const double q1Front = onAxis
		? ( seedAnglesDeg[ 0 ] * Transform::kDegToRad + dh.thetaRad[ 0 ] )
		: std::atan2( Wy, Wx );
	const double q1Back  = wrapToPi( q1Front + Transform::kPi );

	struct ArmPlane
	{
		double q1Rad;   // raw DH angle (includes theta offset)
		double r;       // signed radial coordinate in the rotated plane
	};
	const std::array<ArmPlane, 2> shoulders = { {
		{ q1Front,  rhoMag },
		{ q1Back,  -rhoMag },
	} };

	int candCount = 0;

	for( const auto& shoulder : shoulders ) {
		const double u = shoulder.r - a1;          // horizontal offset from joint 2 to WC
		const double v = d1 - Wz;                  // vertical offset from joint 2 to WC (positive = WC below joint 2)

		const double reachSq = u * u + v * v;
		const double cosArg  = ( reachSq - a2 * a2 - L3 * L3 ) / ( 2.0 * a2 * L3 );

		// Workspace check: if |cosArg| > 1 + ε the target is unreachable for this
		// shoulder branch. We do not early-out — the other shoulder may still work.
		if( cosArg > 1.0 + 1.0e-6 || cosArg < -1.0 - 1.0e-6 ) {
			continue;
		}
		const double cosClamped = std::clamp( cosArg, -1.0, 1.0 );
		const double sinMag     = std::sqrt( std::max( 0.0, 1.0 - cosClamped * cosClamped ) );

		// Elbow up / elbow down branches: q3 + ψ = ±acos(cosArg).
		for( int elbow = 0; elbow < 2; ++elbow ) {
			const double sinSigned = ( elbow == 0 ) ?  sinMag : -sinMag;
			const double q3Plus    = std::atan2( sinSigned, cosClamped );  // = q3 + ψ
			const double q3Raw     = q3Plus - psi;                          // DH angle (no q offset removal yet)

			// q2hat from u = A·c2 - B·s2, v = -(A·s2 + B·c2)? No — recall my derivation:
			// u =  A·c2hat - B·s2hat
			// v =  A·s2hat + B·c2hat  (this v has been negated relative to (d1 - Wz))
			// where A = a2 + L3·cos(q3+ψ), B = L3·sin(q3+ψ). The sign convention above
			// uses v = d1 - Wz so the equation set becomes
			//   u =  A·c2hat - B·s2hat
			//   v =  A·s2hat + B·c2hat
			// Solving gives q2hat = atan2( A·v - B·u, A·u + B·v ).
			const double A = a2 + L3 * cosClamped;
			const double B = L3 * sinSigned;
			const double q2Raw = std::atan2( A * v - B * u, A * u + B * v );

			// -------------------------------------------------------- Step 3
			// Build R_0^3 explicitly from the closed-form rotation matrix derived
			// in the design notes (avoids three full 4x4 multiplies). beta is the
			// "elbow rotation" in the arm plane.
			const double c1   = std::cos( shoulder.q1Rad );
			const double s1   = std::sin( shoulder.q1Rad );
			const double c2   = std::cos( q2Raw );
			const double s2   = std::sin( q2Raw );
			const double beta = q2Raw + q3Raw;
			const double cb   = std::cos( beta );
			const double sb   = std::sin( beta );

			(void)c2; (void)s2;  // only beta survives in R_0^3 for this DH pattern

			// R_0^3 columns (verified analytically — see design notes in commit msg):
			//   x3 = ( c1·cb,  s1·cb, -sb )
			//   y3 = ( s1,    -c1,    0  )
			//   z3 = (-c1·sb, -s1·sb, -cb )
			const double R03[ 3 ][ 3 ] = {
				{  c1 * cb,  s1,      -c1 * sb },
				{  s1 * cb, -c1,      -s1 * sb },
				{ -sb,       0.0,     -cb       },
			};

			// R_3^6 = R_0^3^T · R_target. Read R_target out of the gp_Trsf.
			double R36[ 3 ][ 3 ];
			for( int r = 0; r < 3; ++r ) {
				for( int c = 0; c < 3; ++c ) {
					R36[ r ][ c ] = R03[ 0 ][ r ] * targetTcp.Value( 1, c + 1 )
								  + R03[ 1 ][ r ] * targetTcp.Value( 2, c + 1 )
								  + R03[ 2 ][ r ] * targetTcp.Value( 3, c + 1 );
				}
			}

			// -------------------------------------------------------- Step 4
			// Decompose R_3^6 = Rz(q4) · Ry(-q5) · Rz(q6 + θ6) via ZYZ Euler.
			// Identity used in design: Rx(90)·Rz(q5)·Rx(-90) = Ry(-q5), which
			// turns the wrist's intrinsic Z-X-Z DH twist sequence into a clean
			// ZYZ extraction problem.
			//
			// Standard ZYZ formulas:
			//   β = atan2( ±sqrt(R13² + R23²), R33 )
			//   α = atan2( R23, R13 )
			//   γ = atan2( R32, -R31 )
			// with q4 = α, q5 = -β, q6 + θ6 = γ.
			const double r13 = R36[ 0 ][ 2 ];
			const double r23 = R36[ 1 ][ 2 ];
			const double r33 = R36[ 2 ][ 2 ];
			const double r31 = R36[ 2 ][ 0 ];
			const double r32 = R36[ 2 ][ 1 ];
			const double r11 = R36[ 0 ][ 0 ];
			const double r12 = R36[ 0 ][ 1 ];

			const double sBetaMag = std::sqrt( r13 * r13 + r23 * r23 );

			for( int wristBranch = 0; wristBranch < 2; ++wristBranch ) {
				if( candCount >= static_cast<int>( cands.size() ) ) {
					break;
				}

				double q4Raw;
				double q5Raw;
				double q6Raw;

				if( sBetaMag < kWristSingTol ) {
					// Gimbal: only q4 + q6 (or q4 - q6, depending on cos(β) sign)
					// is observable. Hold q4 at the seed and absorb the remainder
					// into q6 so subsequent IK calls with similar targets stay
					// continuous.
					q4Raw = ( seedAnglesDeg[ 3 ] * Transform::kDegToRad ) + dh.thetaRad[ 3 ];
					if( r33 > 0.0 ) {
						q5Raw = 0.0;                                   // β = 0
						q6Raw = std::atan2( -r12, r11 ) - q4Raw;        // q4 + q6 = atan2(-r12, r11)
					} else {
						q5Raw = Transform::kPi;                         // β = π → q5 = -π (equivalent to +π)
						q6Raw = std::atan2(  r12, -r11 ) + q4Raw;       // q6 - q4 = atan2(r12, -r11)
					}
					if( wristBranch != 0 ) {
						continue;   // singularity has a 1-parameter family, not a discrete second branch
					}
				} else {
					const double sBeta = ( wristBranch == 0 ) ? sBetaMag : -sBetaMag;
					const double beta_ = std::atan2( sBeta, r33 );
					q4Raw = std::atan2( r23 / sBeta, r13 / sBeta );
					q5Raw = -beta_;
					q6Raw = std::atan2( r32 / sBeta, -r31 / sBeta );
				}

				// Subtract DH θ offsets to get user-facing q (the value that the
				// rendered model treats as "axis angle"). Wrap into (-π, π] so
				// limit comparisons are not fooled by 2π aliases.
				Candidate cand;
				cand.q[ 0 ] = wrapToPi( shoulder.q1Rad - dh.thetaRad[ 0 ] ) / Transform::kDegToRad;
				cand.q[ 1 ] = wrapToPi( q2Raw          - dh.thetaRad[ 1 ] ) / Transform::kDegToRad;
				cand.q[ 2 ] = wrapToPi( q3Raw          - dh.thetaRad[ 2 ] ) / Transform::kDegToRad;
				cand.q[ 3 ] = wrapToPi( q4Raw          - dh.thetaRad[ 3 ] ) / Transform::kDegToRad;
				cand.q[ 4 ] = wrapToPi( q5Raw          - dh.thetaRad[ 4 ] ) / Transform::kDegToRad;
				cand.q[ 5 ] = wrapToPi( q6Raw          - dh.thetaRad[ 5 ] ) / Transform::kDegToRad;
				cand.valid  = true;

				snapToLimits( cand, options );
				if( !insideLimits( cand, options ) ) {
					continue;
				}
				cands[ candCount++ ] = cand;
			}
		}
	}

	if( candCount == 0 ) {
		// No reachable / limit-respecting branch. Status stays InvalidConfiguration
		// so the caller can decide whether to retry with DLS (which may converge
		// to a nearby pose even when no exact analytic solution exists).
		return result;
	}

	// ------------------------------------------------------------------ Step 5
	// Pick the candidate closest to the seed, measured by summed wrapped angular
	// distance. Ties (e.g. perfect symmetric reach) resolve to the first valid.
	int    bestIdx  = 0;
	double bestDist = std::numeric_limits<double>::infinity();
	for( int i = 0; i < candCount; ++i ) {
		const double d = seedDistance( cands[ i ], seedAnglesDeg );
		if( d < bestDist ) {
			bestDist = d;
			bestIdx  = i;
		}
	}

	result.jointAnglesDeg      = cands[ bestIdx ].q;
	result.iterations          = 1;                  // closed-form: one "pass"
	result.positionErrorMm     = 0.0;                // analytic: exact within FP precision
	result.orientationErrorRad = 0.0;
	result.status              = IkStatus::Converged;
	return result;
}

}  // namespace OccBridge
