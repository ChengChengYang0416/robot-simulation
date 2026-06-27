#include "TcpTrail.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_Drawer.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

namespace Scene {

void TcpTrail::attach( const Handle( AIS_InteractiveContext ) & ctx )
{
	if( m_context == ctx ) {
		return;
	}
	// Re-attaching to a different context: drop the visuals owned by the old one
	// so we never carry stale Handles into the new context. The point buffer is
	// preserved — a subsequent setMode() or pushPose() will repopulate display.
	removeAllAisObjects();
	m_context = ctx;
}

void TcpTrail::setMode( Mode mode )
{
	if( mode == m_mode ) {
		return;
	}
	m_mode = mode;

	if( mode == Mode::Off ) {
		// Mode Off is the "no overhead" state: drop displayed visuals and the
		// recorded points so we don't pay buffer growth while disabled, and so
		// re-enabling starts a fresh trail at the operator's current pose.
		removeAllAisObjects();
		m_points.clear();
		if( !m_context.IsNull() ) {
			m_context->UpdateCurrentViewer();
		}
		return;
	}

	// Switching between Polyline and PolylineWithFrames just toggles whether the
	// frame trihedrons are drawn. The polyline itself stays as-is.
	rebuildPolyline();
	if( mode == Mode::PolylineWithFrames ) {
		rebuildFrames();
	} else {
		// Polyline-only mode: drop any frames that the previous mode displayed.
		if( !m_context.IsNull() ) {
			for( const auto& frame : m_frames ) {
				if( !frame.IsNull() ) {
					m_context->Remove( frame, Standard_False );
				}
			}
		}
		m_frames.clear();
	}
	if( !m_context.IsNull() ) {
		m_context->UpdateCurrentViewer();
	}
}

void TcpTrail::pushPose( const gp_Trsf& tcp )
{
	if( m_mode == Mode::Off ) {
		return;  // hot-path fast exit when disabled
	}

	// Ring buffer behaviour: cap at kTcpTrailMaxPoints so memory + per-tick
	// polyline rebuild stay bounded during long jogs / MoveL playback.
	m_points.push_back( tcp );
	if( static_cast<int>( m_points.size() ) > kTcpTrailMaxPoints ) {
		m_points.pop_front();
	}

	if( m_points.size() < 2 ) {
		return;  // need at least two points to draw an edge
	}

	rebuildPolyline();
	if( m_mode == Mode::PolylineWithFrames ) {
		rebuildFrames();
	}
	// Intentionally no UpdateCurrentViewer() here — the surrounding tick
	// (RobotSceneFacade::updateRobotTransforms) already triggers a single batched
	// redraw via SceneRepository::updateViewer(), so an extra refresh would just
	// double-redraw at the WPF dispatcher rate.
}

void TcpTrail::clear()
{
	m_points.clear();
	removeAllAisObjects();
	if( !m_context.IsNull() ) {
		m_context->UpdateCurrentViewer();
	}
}

void TcpTrail::rebuildPolyline()
{
	if( m_context.IsNull() || m_points.size() < 2 ) {
		return;
	}

	// Build a TopoDS_Wire by chaining edges between consecutive sampled points.
	// MakeWire-from-edges is preferable to MakePolygon here because MakePolygon
	// is slightly more allocation-heavy for short edges and offers no display
	// advantage for an AIS_Shape wireframe.
	BRepBuilderAPI_MakeWire wireBuilder;
	gp_Pnt previous = m_points.front().TranslationPart();
	for( auto it = std::next( m_points.begin() ); it != m_points.end(); ++it ) {
		const gp_Pnt current = it->TranslationPart();
		BRepBuilderAPI_MakeEdge edgeBuilder( previous, current );
		if( edgeBuilder.IsDone() ) {
			wireBuilder.Add( edgeBuilder.Edge() );
		}
		previous = current;
	}
	if( !wireBuilder.IsDone() ) {
		return;
	}
	const TopoDS_Wire wire = wireBuilder.Wire();

	// Replace the displayed polyline. Reusing the AIS_Shape handle with SetShape()
	// is possible but the recompute cost is similar — full replace keeps the
	// logic simple and matches the lifecycle of frame markers below.
	if( !m_polyline.IsNull() ) {
		m_context->Remove( m_polyline, Standard_False );
	}

	m_polyline = new AIS_Shape( wire );
	// Display mode 0 = wireframe (AIS_WireFrame). Wires have no faces to shade,
	// so mode 1 would silently fall back to wireframe anyway.
	m_context->Display( m_polyline, 0, -1, Standard_False );
	m_context->SetColor( m_polyline, Quantity_Color( Quantity_NOC_YELLOW ), Standard_False );
	m_context->SetWidth( m_polyline, kTcpTrailLineWidth, Standard_False );
}

void TcpTrail::rebuildFrames()
{
	if( m_context.IsNull() ) {
		return;
	}

	// Drop the existing trihedrons. Pooling + setLocalTransformation would avoid
	// the AIS allocation, but the count is bounded (≤ kTcpTrailMaxPoints /
	// kTcpTrailFrameStride ≈ 40) and the rebuild is gated by pushPose() being
	// triggered, so simplicity wins over micro-optimisation here.
	for( const auto& frame : m_frames ) {
		if( !frame.IsNull() ) {
			m_context->Remove( frame, Standard_False );
		}
	}
	m_frames.clear();

	if( m_points.empty() ) {
		return;
	}

	const int n = static_cast<int>( m_points.size() );
	for( int i = 0; i < n; i += kTcpTrailFrameStride ) {
		Handle( Geom_Axis2Placement ) axis = new Geom_Axis2Placement(
			gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ), gp_Dir( 1, 0, 0 ) );
		Handle( AIS_Trihedron ) tri = new AIS_Trihedron( axis );
		tri->SetSize( kTcpTrailFrameSize );
		tri->SetDatumPartColor( Prs3d_DatumParts_XAxis, Quantity_Color( Quantity_NOC_BLUE ) );
		tri->SetDatumPartColor( Prs3d_DatumParts_YAxis, Quantity_Color( Quantity_NOC_GREEN ) );
		tri->SetDatumPartColor( Prs3d_DatumParts_ZAxis, Quantity_Color( Quantity_NOC_RED ) );
		tri->SetLocalTransformation( m_points[ i ] );
		m_context->Display( tri, Standard_False );
		m_frames.push_back( tri );
	}
	// Always include the most recent sample even when (n - 1) is not on a
	// stride boundary, so the head of the trail shows the current orientation.
	const int tail = n - 1;
	if( tail > 0 && ( tail % kTcpTrailFrameStride ) != 0 ) {
		Handle( Geom_Axis2Placement ) axis = new Geom_Axis2Placement(
			gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ), gp_Dir( 1, 0, 0 ) );
		Handle( AIS_Trihedron ) tri = new AIS_Trihedron( axis );
		tri->SetSize( kTcpTrailFrameSize );
		tri->SetDatumPartColor( Prs3d_DatumParts_XAxis, Quantity_Color( Quantity_NOC_BLUE ) );
		tri->SetDatumPartColor( Prs3d_DatumParts_YAxis, Quantity_Color( Quantity_NOC_GREEN ) );
		tri->SetDatumPartColor( Prs3d_DatumParts_ZAxis, Quantity_Color( Quantity_NOC_RED ) );
		tri->SetLocalTransformation( m_points[ tail ] );
		m_context->Display( tri, Standard_False );
		m_frames.push_back( tri );
	}
}

void TcpTrail::removeAllAisObjects()
{
	if( m_context.IsNull() ) {
		m_polyline.Nullify();
		m_frames.clear();
		return;
	}
	if( !m_polyline.IsNull() ) {
		m_context->Remove( m_polyline, Standard_False );
		m_polyline.Nullify();
	}
	for( const auto& frame : m_frames ) {
		if( !frame.IsNull() ) {
			m_context->Remove( frame, Standard_False );
		}
	}
	m_frames.clear();
}

}  // namespace Scene
