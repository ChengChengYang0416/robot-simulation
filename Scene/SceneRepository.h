#pragma once

#include <optional>
#include <vector>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

namespace Scene {

class SceneRepository
{
public:
	struct SlotResult
	{
		int slotId = -1;
		bool isValid = false;
	};
	// Returned by addShape / addColoredShape. Callers must check isValid before using slotId.
	// slotId is a stable, monotonically increasing handle for use with setTransform();
	// it is not a part index, so failed additions never shift other slots.

	void attach( const Handle( AIS_InteractiveContext ) & ctx );
	// Stores the AIS context that owns subsequent shapes; must be called once after the
	// context is constructed. The repository does not create or destroy the context.

	[[nodiscard]] SlotResult addShape( const TopoDS_Shape& shape );
	// Displays a plain shape in shaded mode (Display + SetDisplayMode(1)). Used by
	// callers that do not need per-part color (e.g. ad-hoc STEP loading).

	[[nodiscard]] SlotResult addColoredShape( const TopoDS_Shape& shape,
											  const Quantity_Color& color );
	// Displays a shape directly in shaded mode (Display(..., 1, -1, ...)) with the
	// given color. Used by robot part loading where each part has its own color.

	void setTransform( int slotId, const gp_Trsf& trsf );
	// Applies a local transformation to the slot's AIS_Shape in-place. Out-of-range
	// or invalidated slots are silently ignored; geometry is not re-tessellated.

	void ensureTcpTrihedron();
	// Creates the TCP trihedron once and displays it. Safe to call multiple times;
	// subsequent calls are no-ops as long as the trihedron is alive.

	void setTcpTransform( const gp_Trsf& trsf );
	// Updates the TCP trihedron's local transformation and caches the pose for tcpFrame().

	[[nodiscard]] std::optional<gp_Trsf> tcpFrame() const;
	// Returns the last TCP transform set via setTcpTransform, or std::nullopt if the
	// trihedron has not been created or no transform has been applied yet.

	void clear();
	// Removes all shapes and the TCP trihedron from the context, then drops all internal
	// Handles. The context itself remains attached; callers may reuse the repository.

	void updateViewer();
	// Forwards to AIS_InteractiveContext::UpdateCurrentViewer; batches drawing updates
	// so callers can perform many adds/transforms before triggering one redraw.

	[[nodiscard]] int slotCount() const
	{
		return static_cast<int>( m_shapes.size() );
	}
	// Number of slots ever allocated. Includes slots that may have been individually
	// removed in the future; currently slots are monotonic and only freed via clear().

private:
	Handle( AIS_InteractiveContext ) m_context;
	std::vector<Handle( AIS_Shape )> m_shapes;
	Handle( AIS_Trihedron ) m_tcpTrihedron;
	std::optional<gp_Trsf> m_tcpTrsf;
};

}  // namespace Scene
