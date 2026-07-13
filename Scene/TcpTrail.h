#pragma once

#include <deque>
#include <vector>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>
#include <gp_Trsf.hxx>

namespace Scene {

class TcpTrail
{
// Visualises the TCP's recent travel path as either a polyline (mode Polyline) or
// a polyline plus sub-sampled frame markers along it (mode PolylineWithFrames).
// Owned by RobotSceneFacade as a sibling of SceneRepository — same OCCT context,
// distinct lifecycle. Keeping the trail in its own class instead of bloating
// SceneRepository preserves SRP: SceneRepository registers robot geometry, this
// type owns "TCP history visualisation". The two have independent clear / mode
// semantics that would tangle if collapsed into one class.
//
// Hot-path contract: pushPose() is called from updateRobotTransforms() on every
// joint change (slider, jog, MoveL/MoveJ tick at 3 ms cadence). When mode is Off
// the call is a single comparison + return; when active, the polyline is rebuilt
// via BRepBuilderAPI_MakePolygon. The ring buffer caps memory and rebuild cost.
public:
	enum class Mode : int
	{
		Off                = 0,  // no display, no sampling (cheapest)
		Polyline           = 1,  // TCP point trail only
		PolylineWithFrames = 2,  // TCP trail + sub-sampled mini trihedrons
	};

	void attach( const Handle( AIS_InteractiveContext ) & ctx );
	// Stores the AIS context handle so subsequent display calls route through it.
	// Re-attaching after points were displayed clears the existing visuals first.

	void setMode( Mode mode );
	// Switches visualisation mode. Off removes all displayed AIS objects and
	// resets the buffer; Polyline / PolylineWithFrames re-display from current
	// buffer contents (which is normally empty after Off, so the trail starts
	// fresh on re-enable).

	[[nodiscard]] Mode mode() const noexcept { return m_mode; }

	void pushPose( const gp_Trsf& tcp );
	// Records the TCP pose at the tail of the ring buffer and refreshes the
	// polyline / frame markers if the mode is active. No-op when mode == Off,
	// keeping the hot path cheap when the operator has the trail disabled.

	void clear();
	// Empties the ring buffer and removes any displayed AIS objects from the
	// context. Mode is preserved so the next pushPose() rebuilds in the same
	// visualisation style.

	void setMaxPoints( int maxPoints );
	// Updates the ring-buffer cap. Values are clamped to [2, kAbsoluteTrailMaxPoints]
	// so the polyline always has at least two points to draw and cannot grow
	// unbounded from a bad HMI push. When the new cap is smaller than the current
	// buffer size, oldest samples are dropped from the front and the polyline /
	// frames are rebuilt to match. No-op when the value is unchanged.

	void setFrameStride( int stride );
	// Updates the sub-sampling stride used by PolylineWithFrames mode. Clamped
	// to [1, kAbsoluteTrailMaxStride]. Rebuilds frames immediately when the mode
	// is PolylineWithFrames; otherwise stored for the next mode switch.

	void setColor( int r, int g, int b );
	// Sets the polyline colour. Component values are clamped to [0, 255] and
	// re-applied to the currently displayed polyline (if any). The frame
	// trihedrons keep the X = blue, Y = green, Z = red convention because
	// their per-axis colouring conveys orientation independently of the trail
	// colour.

	[[nodiscard]] int maxPoints() const noexcept { return m_maxPoints; }
	[[nodiscard]] int frameStride() const noexcept { return m_frameStride; }

	[[nodiscard]] int pointCount() const noexcept
	{
		return static_cast<int>( m_points.size() );
	}

private:
	void rebuildPolyline();
	// Rebuilds the displayed polyline from m_points. Two-point minimum (a single
	// point has no edge to draw). Replaces any existing m_polyline handle in the
	// context. Caller must batch a context UpdateCurrentViewer().

	void rebuildFrames();
	// Releases all existing frame trihedrons and re-displays one every
	// kFrameStride points (and one at the tail). Cheap because the sub-sample
	// count is bounded by kMaxPoints / kFrameStride.

	void removeAllAisObjects();
	// Internal helper: removes the polyline and every frame from the context
	// (Standard_False so the caller batches the redraw). Called by setMode(Off),
	// clear(), and the rebuild paths before re-displaying.

	Handle( AIS_InteractiveContext ) m_context;
	std::deque<gp_Trsf>              m_points;
	Handle( AIS_Shape )              m_polyline;
	std::vector<Handle( AIS_Trihedron )> m_frames;
	Mode                             m_mode = Mode::Off;
	int                              m_maxPoints    = 2000;
	int                              m_frameStride  = 50;
	Quantity_Color                   m_lineColor{ Quantity_NOC_YELLOW };
};

// Default buffer / visual tuning. Kept as `inline constexpr` so callers (e.g.
// HMI wire values) can reference the same numbers used to seed a fresh trail.
// Per-instance overrides go through setMaxPoints / setFrameStride / setColor.
inline constexpr int    kDefaultTcpTrailMaxPoints   = 2000;
inline constexpr int    kDefaultTcpTrailFrameStride = 50;
inline constexpr double kTcpTrailLineWidth          = 2.0;
inline constexpr double kTcpTrailFrameSize          = 20.0;
// Hard safety caps so a bad HMI push cannot degenerate the visualisation.
// kAbsoluteTrailMaxPoints keeps memory + per-tick polyline rebuild bounded;
// kAbsoluteTrailMaxStride is loose (a very high stride just hides every marker
// bar the tail one, which is a valid "polyline-only-ish" degenerate).
inline constexpr int kAbsoluteTrailMaxPoints = 20000;
inline constexpr int kAbsoluteTrailMaxStride = 2000;

}  // namespace Scene
