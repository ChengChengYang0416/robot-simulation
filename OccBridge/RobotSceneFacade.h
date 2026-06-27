#pragma once

#include "IRobotScene.h"

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
	[[nodiscard]] bool getTcpPose( double out[ 6 ] ) const override;

	[[nodiscard]] int solveTcpIk( const double targetXyzRpy[ 6 ],
								  const double jointMinDeg[ 6 ],
								  const double jointMaxDeg[ 6 ],
								  double       outAnglesDeg[ 6 ] ) override;

	[[nodiscard]] bool getManipulability( double outMetrics[ 5 ],
										  int*   outKind,
										  int*   outLevel ) const override;

	void clearScene() override;

	void fitAll() override;
	void setViewIso() override;
	void setViewTop() override;

	void setTcpTrailMode( int mode ) override;
	void clearTcpTrail() override;

	void onMouseDown( int x, int y, int button ) override;
	void onMouseMove( int x, int y, int buttonMask ) override;
	void onMouseUp() override;
	void onMouseWheel( int delta ) override;

	[[nodiscard]] bool saveScreenshot( const wchar_t* filePath ) override;

private:
	void updateRobotTransforms();
	// Recomputes cumulative DH transforms for every part and pushes them into the
	// SceneRepository via the partToSlot map. Also refreshes the TCP trihedron pose.

	struct Impl;
	// PIMPL: defined in the .cpp so OCCT headers (V3d_View, AIS_Shape, ...) do not
	// leak through this header.

	Impl* m_impl;
};

}  // namespace OccBridge
