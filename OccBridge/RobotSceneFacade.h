#pragma once

#include "IRobotScene.h"

class gp_Trsf;
class gp_Pnt;
class gp_Dir;

namespace OccBridge {

class RobotSceneFacade : public IRobotScene
{
// Concrete IRobotScene that composes the small libraries refactored out in
// phases R1..R5 (Viewer / Scene / Interaction / Kinematics). All OCCT state
// lives behind the PIMPL Impl struct so this header stays free of OCCT
// includes — keeping compile times bounded and shielding managed callers.
public:
	RobotSceneFacade();
	// Allocates the PIMPL Impl; OCCT handles are default-constructed (null) inside.

	~RobotSceneFacade() override;
	// Releases the PIMPL Impl; OCCT handles drop their ref-count automatically.

	void initialize( HWND hwnd ) override;
	void resize( int width, int height ) override;
	void redraw() override;

	[[nodiscard]] bool loadStep( const wchar_t* filePath, bool append ) override;

	[[nodiscard]] bool beginRobotArm( const RobotPartDef* parts, int partCount,
									  const int* axisToPartMap, int mapCount ) override;
	[[nodiscard]] bool loadRobotPart( int index ) override;
	void endRobotArm() override;

	void setJointAngle( int axisIndex, double angleDeg ) override;
	void setJointAngles( const double anglesDeg[ 6 ] ) override;
	[[nodiscard]] bool getJointAngles( double outAnglesDeg[ 6 ] ) const override;
	[[nodiscard]] bool getTcpPose( double out[ 6 ] ) const override;

	[[nodiscard]] int solveTcpIk( const double targetXyzRpy[ 6 ],
								  const double jointMinDeg[ 6 ],
								  const double jointMaxDeg[ 6 ],
								  double       outAnglesDeg[ 6 ] ) override;

	[[nodiscard]] bool getManipulability( double outMetrics[ 5 ],
										  int*   outKind,
										  int*   outLevel ) const override;

	void setJointLimits( const double jointMinDeg[ 6 ],
						 const double jointMaxDeg[ 6 ] ) override;

	void setDragEnabled( bool enabled ) override;

	void setGizmoMode( int mode ) override;

	void clearScene() override;

	void fitAll() override;
	void setViewIso() override;
	void setViewTop() override;

	void setTcpTrailMode( int mode ) override;
	void clearTcpTrail() override;
	void setTcpTrailMaxPoints( int maxPoints ) override;
	void setTcpTrailFrameStride( int stride ) override;
	void setTcpTrailColor( int r, int g, int b ) override;

	void onMouseDown( int x, int y, int button ) override;
	void onMouseMove( int x, int y, int buttonMask ) override;
	void onMouseUp() override;
	void onMouseWheel( int delta ) override;

	[[nodiscard]] bool saveScreenshot( const wchar_t* filePath ) override;

private:
	void updateRobotTransforms();
	// Recomputes cumulative DH transforms for every part and pushes them into the
	// SceneRepository via the partToSlot map. Also refreshes the TCP trihedron pose.

	void applyDragTarget( const gp_Trsf& targetWorld );
	// Drag-mode IK callback: decomposes the manipulator's target world pose, runs
	// analytical-first / DLS-fallback IK using the cached per-axis joint limits, and
	// commits the solution via setJointAngles() on convergence. The gizmo's screen
	// position remains where the operator dragged it; the FK pass triggered by the
	// joint commit moves the trihedron to match.

	void applyJointDragDelta( int axisIdxZeroBased, double deltaDeg );
	// Joint-drag callback (Phase 3.3): applies an incremental angle to a single axis,
	// clamps to the cached joint limits, and refreshes the scene through the standard
	// FK path. Called from the JointDragController via a std::function closure wired
	// up in initialize().

	void syncDragControllers();
	// Reconciles the (dragEnabled, gizmoMode) pair into concrete enable/disable calls
	// on the manipulator gizmo (TCP Translate/Rotate) and the joint-drag picker (per-
	// link Joint). Invoked by setDragEnabled and setGizmoMode so the two setters do
	// not need to know each other's state to keep the controllers consistent.

	bool axisFrameForIndex( int axisOneBased, gp_Pnt& origin, gp_Dir& dir ) const;
	// Resolves the world-space pivot and rotation axis for driven axis 1..6 by
	// walking the DH cumulative chain to the joint's parent frame (origin +
	// Z-axis). Extracted so both the joint-drag picker's frame callback and the
	// ring-placement pass share one implementation.

	void rebuildJointRings();
	// Discards any existing rings and builds one fresh AIS_Circle per driven axis
	// with a per-axis radius scaled off the arm's DH reach. Called from
	// endRobotArm() after the shape lookup is populated so ring pointers can be
	// merged into the same axisLookup handed to the joint-drag picker.

	void refreshJointRingPoses();
	// Repositions every ring to match its axis's current parent frame. Runs on
	// every FK update while joint-mode is active so rings track the arm as
	// upstream joints move; a no-op when the rings list is empty.

	void setJointRingsVisible( bool visible );
	// Displays or erases every ring in a single AIS context call and toggles the
	// cached visibility flag. Also drives whether refreshJointRingPoses runs on
	// FK updates — hidden rings do not need re-posing.

	void pushAxisLookupToJointDrag();
	// Merges shapeAxisLookup with the ring pointers and hands the union to the
	// joint-drag picker. Called whenever either half of the lookup changes
	// (endRobotArm rebuilds shapes, rebuildJointRings rebuilds rings).

	struct Impl;
	// PIMPL: defined in the .cpp so OCCT headers (V3d_View, AIS_Shape, ...) do not
	// leak through this header.

	Impl* m_impl;
};

}  // namespace OccBridge
