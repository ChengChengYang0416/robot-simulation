#include "TcpTrail.h"
#include <algorithm>
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

	// Ring buffer behaviour: cap at m_maxPoints so memory + per-tick
	// polyline rebuild stay bounded during long jogs / MoveL playback.
	m_points.push_back( tcp );
	while( static_cast<int>( m_points.size() ) > m_maxPoints ) {
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
	m_context->SetColor( m_polyline, m_lineColor, Standard_False );
	m_context->SetWidth( m_polyline, kTcpTrailLineWidth, Standard_False );
}

void TcpTrail::rebuildFrames()
{
	if( m_context.IsNull() ) {
		return;
	}

	// Drop the existing trihedrons. Pooling + setLocalTransformation would avoid
	// the AIS allocation, but the count is bounded (≤ m_maxPoints /
	// m_frameStride, default ≈ 40) and the rebuild is gated by pushPose() being
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

	// Local helper: build + display a labelless trihedron at the given pose. The
	// "X / Y / Z" text labels are suppressed because at trail densities the
	// repeated text triplets fight the polyline for screen real estate and
	// drop legibility; the per-axis colours (X=blue, Y=green, Z=red — matched
	// to the TCP trihedron) already convey orientation unambiguously.
	auto displayFrame = [ this ]( const gp_Trsf& pose ) {
		Handle( Geom_Axis2Placement ) axis = new Geom_Axis2Placement(
			gp_Pnt( 0, 0, 0 ), gp_Dir( 0, 0, 1 ), gp_Dir( 1, 0, 0 ) );
		Handle( AIS_Trihedron ) tri = new AIS_Trihedron( axis );
		tri->SetSize( kTcpTrailFrameSize );
		tri->SetDatumPartColor( Prs3d_DatumParts_XAxis, Quantity_Color( Quantity_NOC_BLUE ) );
		tri->SetDatumPartColor( Prs3d_DatumParts_YAxis, Quantity_Color( Quantity_NOC_GREEN ) );
		tri->SetDatumPartColor( Prs3d_DatumParts_ZAxis, Quantity_Color( Quantity_NOC_RED ) );
		// Suppress the "X / Y / Z" text labels. SetDrawLabels lives on
		// Prs3d_DatumAspect which AIS_Trihedron exposes through its Attributes()
		// drawer. Must be set before Display() so the first render already
		// reflects the choice.
		const Handle( Prs3d_DatumAspect ) datum = tri->Attributes()->DatumAspect();
		datum->SetDrawLabels( false );
		tri->SetLocalTransformation( pose );
		m_context->Display( tri, Standard_False );
		m_frames.push_back( tri );
	};

	const int n = static_cast<int>( m_points.size() );
	for( int i = 0; i < n; i += m_frameStride ) {
		displayFrame( m_points[ i ] );
	}
	// Always include the most recent sample even when (n - 1) is not on a
	// stride boundary, so the head of the trail shows the current orientation.
	const int tail = n - 1;
	if( tail > 0 && ( tail % m_frameStride ) != 0 ) {
		displayFrame( m_points[ tail ] );
	}
}

void TcpTrail::setMaxPoints( int maxPoints )
{
	// Hard clamp to sane bounds. Minimum 2 keeps the polyline drawable; upper
	// bound bounds per-tick rebuild work and RAM usage.
	const int clamped = std::clamp( maxPoints, 2, kAbsoluteTrailMaxPoints );
	if( clamped == m_maxPoints ) {
		return;
	}
	m_maxPoints = clamped;

	// Shrink existing buffer if the new cap is tighter, then re-render so the
	// visible trail matches the new bound immediately (rather than waiting for
	// the next pushPose to notice the overflow).
	bool shrunk = false;
	while( static_cast<int>( m_points.size() ) > m_maxPoints ) {
		m_points.pop_front();
		shrunk = true;
	}
	if( shrunk && m_mode != Mode::Off ) {
		rebuildPolyline();
		if( m_mode == Mode::PolylineWithFrames ) {
			rebuildFrames();
		}
		if( !m_context.IsNull() ) {
			m_context->UpdateCurrentViewer();
		}
	}
}

void TcpTrail::setFrameStride( int stride )
{
	const int clamped = std::clamp( stride, 1, kAbsoluteTrailMaxStride );
	if( clamped == m_frameStride ) {
		return;
	}
	m_frameStride = clamped;

	// Only PolylineWithFrames consumes the stride; other modes will pick it up
	// on the next mode switch.
	if( m_mode == Mode::PolylineWithFrames ) {
		rebuildFrames();
		if( !m_context.IsNull() ) {
			m_context->UpdateCurrentViewer();
		}
	}
}

void TcpTrail::setColor( int r, int g, int b )
{
	const int rc = std::clamp( r, 0, 255 );
	const int gc = std::clamp( g, 0, 255 );
	const int bc = std::clamp( b, 0, 255 );
	// Quantity_Color RGB constructor takes doubles in [0, 1]; sRGB is the
	// default colour space and matches OCCT's other named-colour presets so
	// yellow-in stays yellow-out on screen.
	m_lineColor = Quantity_Color(
		rc / 255.0, gc / 255.0, bc / 255.0, Quantity_TOC_RGB );

	// Re-tint the currently displayed polyline without touching the buffer or
	// rebuilding geometry — colour is a pure presentation property.
	if( !m_context.IsNull() && !m_polyline.IsNull() ) {
		m_context->SetColor( m_polyline, m_lineColor, Standard_False );
		m_context->UpdateCurrentViewer();
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
