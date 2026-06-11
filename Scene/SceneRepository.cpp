#include "SceneRepository.h"
#include <Geom_Axis2Placement.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

namespace Scene {

void SceneRepository::attach( const Handle( AIS_InteractiveContext ) & ctx )
// Stores the AIS context handle so subsequent operations can route through it.
// Re-attaching to a different context after shapes have been added is not supported.
{
	m_context = ctx;
}

SceneRepository::SlotResult SceneRepository::addShape( const TopoDS_Shape& shape )
// Mirrors the legacy loadStep flow: Display in default mode, then SetDisplayMode(1)
// to switch to shaded. Both calls pass Standard_False so the viewer is not redrawn
// per-shape; callers batch with updateViewer().
{
	if( m_context.IsNull() ) {
		return {};
	}

	Handle( AIS_Shape ) aisShape = new AIS_Shape( shape );
	m_context->Display( aisShape, Standard_False );
	m_context->SetDisplayMode( aisShape, 1, Standard_False );

	const int slotId = static_cast<int>( m_shapes.size() );
	m_shapes.push_back( aisShape );
	return { slotId, true };
}

SceneRepository::SlotResult SceneRepository::addColoredShape( const TopoDS_Shape& shape,
															  const Quantity_Color& color )
// Mirrors the legacy loadRobotPart flow: Display directly in shaded mode (1) with a
// per-shape color. SetColor must be called separately because Display does not take
// a color parameter in this overload.
{
	if( m_context.IsNull() ) {
		return {};
	}

	Handle( AIS_Shape ) aisShape = new AIS_Shape( shape );
	m_context->Display( aisShape, 1, -1, Standard_False );
	m_context->SetColor( aisShape, color, Standard_False );

	const int slotId = static_cast<int>( m_shapes.size() );
	m_shapes.push_back( aisShape );
	return { slotId, true };
}

void SceneRepository::setTransform( int slotId, const gp_Trsf& trsf )
// Cheap update path: SetLocalTransformation does not re-tessellate, so this is safe
// to call on every slider change. Null slots (cleared shapes) are skipped.
{
	if( slotId < 0 || slotId >= static_cast<int>( m_shapes.size() ) ) {
		return;
	}
	if( m_shapes[ slotId ].IsNull() ) {
		return;
	}
	m_shapes[ slotId ]->SetLocalTransformation( trsf );
}

void SceneRepository::ensureTcpTrihedron()
// Creates the TCP trihedron once with the conventional axis colors (X=blue, Y=green,
// Z=red) matching the corner trihedron set up by NativeOccView::initialize.
{
	if( m_context.IsNull() || !m_tcpTrihedron.IsNull() ) {
		return;
	}

	Handle( Geom_Axis2Placement ) axis = new Geom_Axis2Placement(
		gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ), gp_Dir( 1, 0, 0 ) );
	m_tcpTrihedron = new AIS_Trihedron( axis );
	m_tcpTrihedron->SetSize( 80.0 );
	m_tcpTrihedron->SetDatumPartColor( Prs3d_DatumParts_XAxis, Quantity_Color( Quantity_NOC_BLUE ) );
	m_tcpTrihedron->SetDatumPartColor( Prs3d_DatumParts_YAxis, Quantity_Color( Quantity_NOC_GREEN ) );
	m_tcpTrihedron->SetDatumPartColor( Prs3d_DatumParts_ZAxis, Quantity_Color( Quantity_NOC_RED ) );
	m_context->Display( m_tcpTrihedron, Standard_False );
}

void SceneRepository::setTcpTransform( const gp_Trsf& trsf )
// Applies the transform to the trihedron and caches it so tcpFrame() can return the
// last known pose without re-querying OCCT. No-op if the trihedron is not created.
{
	if( m_tcpTrihedron.IsNull() ) {
		return;
	}
	m_tcpTrihedron->SetLocalTransformation( trsf );
	m_tcpTrsf = trsf;
}

std::optional<gp_Trsf> SceneRepository::tcpFrame() const
// Returns std::nullopt until both ensureTcpTrihedron() and setTcpTransform() have run.
{
	return m_tcpTrsf;
}

void SceneRepository::clear()
// Removes every AIS object the repository displayed and drops its Handles. Reuses the
// existing m_context attachment so callers do not need to re-attach after clearing.
{
	if( m_context.IsNull() ) {
		m_shapes.clear();
		m_tcpTrihedron.Nullify();
		m_tcpTrsf.reset();
		return;
	}

	for( const auto& shape : m_shapes ) {
		if( !shape.IsNull() ) {
			m_context->Remove( shape, Standard_False );
		}
	}
	m_shapes.clear();

	if( !m_tcpTrihedron.IsNull() ) {
		m_context->Remove( m_tcpTrihedron, Standard_False );
		m_tcpTrihedron.Nullify();
	}
	m_tcpTrsf.reset();
}

void SceneRepository::updateViewer()
// Single batched redraw entry point. Callers do many adds / setTransforms with
// Standard_False, then call this once at the end of the operation.
{
	if( !m_context.IsNull() ) {
		m_context->UpdateCurrentViewer();
	}
}

}  // namespace Scene
