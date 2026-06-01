#include <msclr/marshal_cppstd.h>
#include "OccViewerControl.h"
#include "NativeOccView.h"
#include "RobotPartDef.h"
#include <vector>
#include <string>

namespace OccBridge {

	OccViewerControl::OccViewerControl( void )
		: m_pNative( new NativeOccView() ), _initialized( false )
	// Creates the native viewer and sets default control appearance: dark background, fill parent, no double-buffering
	{
		// DoubleBuffered must be off: WinForms' double buffering would draw on top
		// of OCCT's OpenGL output and erase it. ControlStyles::Selectable lets the
		// control receive mouse-wheel events after a click-to-focus.
		this->Dock = DockStyle::Fill;
		this->DoubleBuffered = false;
		this->SetStyle( ControlStyles::Selectable, true );
	}

	OccViewerControl::~OccViewerControl( void )
	// Delegates to the finalizer to release unmanaged resources
	{
		// Standard managed/native cleanup pattern: the destructor (called by
		// IDisposable::Dispose) invokes the !finalizer which actually frees the
		// native pointer. This guarantees deterministic cleanup even if the GC
		// finalizer thread runs the destructor first.
		this->!OccViewerControl();
	}

	OccViewerControl::!OccViewerControl( void )
	// Deletes the native viewer object to prevent memory leaks
	{
		if( m_pNative != nullptr ) {
			delete m_pNative;
			m_pNative = nullptr;
		}
	}

	void OccViewerControl::OnHandleCreated( EventArgs^ e )
	// Initializes OCCT the first time the window handle is ready; skips if already initialized
	{
		UserControl::OnHandleCreated( e );
		// OCCT needs a valid HWND to bind its OpenGL surface, so initialization is
		// deferred until WinForms creates the underlying window. The _initialized
		// guard prevents re-initialization if the handle is recreated.
		if( !_initialized && this->Handle != IntPtr::Zero ) {
			m_pNative->initialize( static_cast<HWND>( this->Handle.ToPointer() ) );
			_initialized = true;
		}
	}

	void OccViewerControl::OnResize( EventArgs^ e )
	// Notifies OCCT to adjust the viewport when the window is resized
	{
		UserControl::OnResize( e );
		if( _initialized ) {
			m_pNative->resize( this->Width, this->Height );
		}
	}

	void OccViewerControl::OnPaint( PaintEventArgs^ e )
	// Delegates WinForms repaint to OCCT redraw to prevent blank flicker
	{
		UserControl::OnPaint( e );
		if( _initialized ) {
			m_pNative->redraw();
		}
	}

	bool OccViewerControl::LoadStep( String^ path, bool append )
	// Marshals the managed string to a native wide string and forwards to the native loader
	{
		if( !_initialized ) {
			return false;
		}

		std::wstring nativePath = msclr::interop::marshal_as<std::wstring>( path );
		return m_pNative->loadStep( nativePath.c_str(), append );
	}

	void OccViewerControl::ClearScene( void )
	// Forwards to the native clear, releasing all AIS objects and axis state
	{
		if( _initialized ) {
			m_pNative->clearScene();
		}
	}

	bool OccViewerControl::LoadRobotArm( cli::array<RobotPartInfo^>^ parts,
								 cli::array<cli::array<int>^>^ axisToPartMap,
								 Action<int, int>^ progress )
	// Converts the managed arrays to native structs and forwards to the native loader
	{
		if( !_initialized ) {
			return false;
		}

		// Step 1: Marshal each managed RobotPartInfo into a native RobotPartDef.
		// std::wstring assignment copies the marshalled buffer, so the managed
		// String^ does not need to remain pinned afterwards.
		std::vector<RobotPartDef> nativeParts( parts->Length );
		for( int i = 0; i < parts->Length; i++ ) {
			System::String^ fp = parts[ i ]->FilePath;
			nativeParts[ i ].filePath = msclr::interop::marshal_as<std::wstring>( fp );
			nativeParts[ i ].dhA = parts[ i ]->DH_a;
			nativeParts[ i ].dhAlpha = parts[ i ]->DH_alpha;
			nativeParts[ i ].dhD = parts[ i ]->DH_d;
			nativeParts[ i ].dhTheta = parts[ i ]->DH_theta;
			for( int j = 0; j < 6; j++ ) {
				nativeParts[ i ].offset[ j ] = parts[ i ]->Offset[ j ];
			}
			nativeParts[ i ].parentIdx = parts[ i ]->ParentIdx;
			nativeParts[ i ].colorR = parts[ i ]->ColorR;
			nativeParts[ i ].colorG = parts[ i ]->ColorG;
			nativeParts[ i ].colorB = parts[ i ]->ColorB;
		}

		// Step 2: Flatten the [[axis, partIdx], ...] jagged array into a contiguous
		// int[] (pairs) that crosses the managed/native boundary without nested
		// allocations on the native side.
		std::vector<int> nativeMap;
		if( axisToPartMap != nullptr ) {
			for( int i = 0; i < axisToPartMap->Length; i++ ) {
				nativeMap.push_back( axisToPartMap[ i ][ 0 ] );
				nativeMap.push_back( axisToPartMap[ i ][ 1 ] );
			}
		}

		// Step 3: Three-phase native load: beginRobotArm stores the metadata,
		// loadRobotPart streams in one STEP file per call (so the UI can report
		// progress between calls), and endRobotArm finalizes transforms + TCP.
		const int n = parts->Length;
		if( !m_pNative->beginRobotArm( nativeParts.data(), n,
									 nativeMap.data(), static_cast<int>( nativeMap.size() ) ) ) {
			return false;
		}

		for( int i = 0; i < n; i++ ) {
			if( progress != nullptr ) {
				progress->Invoke( i + 1, n );
			}
			// loadRobotPart is [[nodiscard]]; the cast to void is intentional because
			// failures push a null placeholder and we still want to continue with
			// remaining parts rather than abort the whole load.
			(void)m_pNative->loadRobotPart( i );
		}

		m_pNative->endRobotArm();
		return true;
	}

	void OccViewerControl::SetJointAngle( int axisIndex, double angleDeg )
	// Forwards the joint angle to the native viewer
	{
		if( _initialized ) {
			m_pNative->setJointAngle( axisIndex, angleDeg );
		}
	}

	cli::array<double>^ OccViewerControl::GetTcpPose( void )
	// Retrieves the TCP pose from the native viewer and marshals it into a managed array
	{
		if( !_initialized ) {
			return nullptr;
		}
		double buf[ 6 ] = { 0, 0, 0, 0, 0, 0 };
		if( !m_pNative->getTcpPose( buf ) ) {
			return nullptr;
		}
		auto result = gcnew cli::array<double>( 6 );
		for( int i = 0; i < 6; ++i ) {
			result[ i ] = buf[ i ];
		}
		return result;
	}

	void OccViewerControl::FitAllView( void )
	// Forwards fit-all to the native viewer
	{
		if( _initialized ) {
			m_pNative->fitAll();
		}
	}

	void OccViewerControl::SetViewIso( void )
	// Forwards isometric view switch to the native viewer
	{
		if( _initialized ) {
			m_pNative->setViewIso();
		}
	}

	void OccViewerControl::SetViewTop( void )
	// Forwards top view switch to the native viewer
	{
		if( _initialized ) {
			m_pNative->setViewTop();
		}
	}

	void OccViewerControl::OnMouseDown( MouseEventArgs^ e )
	// Ensures the control has keyboard focus, then forwards mouse-down to the native viewer
	{
		UserControl::OnMouseDown( e );
		// WinForms does not auto-focus a control on mouse click unless it is the
		// active tab stop. Forcing focus here makes the mouse wheel work right
		// after the user clicks into the viewer.
		if( !this->Focused ) {
			this->Focus();
		}
		if( _initialized ) {
			m_pNative->onMouseDown( e->X, e->Y, static_cast<int>( e->Button ) );
		}
	}

	void OccViewerControl::OnMouseMove( MouseEventArgs^ e )
	// Forwards mouse-move to the native viewer for rotation, pan, or highlight update
	{
		UserControl::OnMouseMove( e );
		if( _initialized ) {
			m_pNative->onMouseMove( e->X, e->Y, static_cast<int>( e->Button ) );
		}
	}

	void OccViewerControl::OnMouseUp( MouseEventArgs^ e )
	// Forwards mouse-up to the native viewer to stop rotation and pan
	{
		UserControl::OnMouseUp( e );
		if( _initialized ) {
			m_pNative->onMouseUp( );
		}
	}

	void OccViewerControl::OnMouseWheel( MouseEventArgs^ e )
	// Forwards mouse-wheel to the native viewer to zoom the 3D scene
	{
		UserControl::OnMouseWheel( e );
		if( _initialized ) {
			m_pNative->onMouseWheel( e->Delta );
		}
	}

}
