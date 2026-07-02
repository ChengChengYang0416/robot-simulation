#include <msclr/marshal_cppstd.h>
#include "OccViewerControl.h"
#include "IRobotScene.h"
#include "../Kinematics/RobotPartDef.h"
#include <vector>
#include <string>

namespace OccBridge {

	OccViewerControl::OccViewerControl( void )
		: m_pNative( createRobotScene() ), m_bInitialized( false )
	// Creates the native scene via the IRobotScene factory and sets default control appearance:
	// dark background, fill parent, no double-buffering.
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
		// deferred until WinForms creates the underlying window. The m_bInitialized
		// guard prevents re-initialization if the handle is recreated.
		if( !m_bInitialized && this->Handle != IntPtr::Zero ) {
			m_pNative->initialize( static_cast<HWND>( this->Handle.ToPointer() ) );
			m_bInitialized = true;
		}
	}

	void OccViewerControl::OnResize( EventArgs^ e )
	// Notifies OCCT to adjust the viewport when the window is resized
	{
		UserControl::OnResize( e );
		if( m_bInitialized ) {
			m_pNative->resize( this->Width, this->Height );
		}
	}

	void OccViewerControl::OnPaint( PaintEventArgs^ e )
	// Delegates WinForms repaint to OCCT redraw to prevent blank flicker
	{
		UserControl::OnPaint( e );
		if( m_bInitialized ) {
			m_pNative->redraw();
		}
	}

	bool OccViewerControl::LoadStep( String^ path, bool append )
	// Marshals the managed string to a native wide string and forwards to the native loader
	{
		if( !m_bInitialized ) {
			return false;
		}

		std::wstring nativePath = msclr::interop::marshal_as<std::wstring>( path );
		return m_pNative->loadStep( nativePath.c_str(), append );
	}

	void OccViewerControl::ClearScene( void )
	// Forwards to the native clear, releasing all AIS objects and axis state
	{
		if( m_bInitialized ) {
			m_pNative->clearScene();
		}
	}

	bool OccViewerControl::LoadRobotArm( cli::array<RobotPartInfo^>^ parts,
								 cli::array<cli::array<int>^>^ axisToPartMap,
								 Action<int, int>^ progress )
	// Converts the managed arrays to native structs and forwards to the native loader
	{
		if( !m_bInitialized ) {
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
		if( m_bInitialized ) {
			m_pNative->setJointAngle( axisIndex, angleDeg );
		}
	}

	void OccViewerControl::SetJointAngles( cli::array<double>^ anglesDeg )
	// Marshals all six joint angles in one shot to the native batch entry point so a
	// MoveL / MoveJ tick redraws the scene only once per frame.
	{
		if( !m_bInitialized || anglesDeg == nullptr || anglesDeg->Length < 6 ) {
			return;
		}
		double buf[ 6 ]{};
		for( int i = 0; i < 6; ++i ) {
			buf[ i ] = anglesDeg[ i ];
		}
		m_pNative->setJointAngles( buf );
	}

	cli::array<double>^ OccViewerControl::GetTcpPose( void )
	// Retrieves the TCP pose from the native viewer and marshals it into a managed array
	{
		if( !m_bInitialized ) {
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

	cli::array<double>^ OccViewerControl::GetJointAngles( void )
	// Reads the kinematics core's current joint vector out to a managed array.
	// Returns nullptr when no robot is loaded so subscribers of JointsChanged can
	// early-out without a per-axis loop over uninitialized data.
	{
		if( !m_bInitialized ) {
			return nullptr;
		}
		double buf[ 6 ] = { 0, 0, 0, 0, 0, 0 };
		if( !m_pNative->getJointAngles( buf ) ) {
			return nullptr;
		}
		auto result = gcnew cli::array<double>( 6 );
		for( int i = 0; i < 6; ++i ) {
			result[ i ] = buf[ i ];
		}
		return result;
	}

	int OccViewerControl::SolveTcpIk( cli::array<double>^ targetXyzRpy,
									  cli::array<double>^ jointMinDeg,
									  cli::array<double>^ jointMaxDeg,
									  cli::array<double>^ outAnglesDeg )
	// Marshals the managed arrays into native buffers and forwards to the native IK
	// entry point. Validates lengths up-front so a malformed call returns InvalidConfig
	// rather than corrupting memory.
	{
		const int expected = 6;
		if( !m_bInitialized
			|| targetXyzRpy == nullptr || targetXyzRpy->Length < expected
			|| outAnglesDeg == nullptr || outAnglesDeg->Length < expected ) {
			return static_cast<int>( IRobotScene::IkSolveStatus::InvalidConfig );
		}

		double tgt[ 6 ]{};
		double lo[ 6 ]{};
		double hi[ 6 ]{};
		double out[ 6 ]{};
		for( int i = 0; i < expected; ++i ) {
			tgt[ i ] = targetXyzRpy[ i ];
		}

		const bool haveLimits = ( jointMinDeg != nullptr && jointMinDeg->Length >= expected
								  && jointMaxDeg != nullptr && jointMaxDeg->Length >= expected );
		if( haveLimits ) {
			for( int i = 0; i < expected; ++i ) {
				lo[ i ] = jointMinDeg[ i ];
				hi[ i ] = jointMaxDeg[ i ];
			}
		}

		const int status = m_pNative->solveTcpIk( tgt,
												  haveLimits ? lo : nullptr,
												  haveLimits ? hi : nullptr,
												  out );
		for( int i = 0; i < expected; ++i ) {
			outAnglesDeg[ i ] = out[ i ];
		}
		return status;
	}

	bool OccViewerControl::GetManipulability( cli::array<double>^ outMetrics,
											  int% outKind,
											  int% outLevel )
	// Marshals the singularity report from the native facade. outKind / outLevel are
	// always written (defaulting to 0 = None / Normal) so caller code can read them
	// unconditionally even when the call returns false.
	{
		outKind  = 0;
		outLevel = 0;
		if( !m_bInitialized || outMetrics == nullptr || outMetrics->Length < 5 ) {
			return false;
		}

		double metrics[ 5 ]{};
		int    kind  = 0;
		int    level = 0;
		if( !m_pNative->getManipulability( metrics, &kind, &level ) ) {
			return false;
		}
		for( int i = 0; i < 5; ++i ) {
			outMetrics[ i ] = metrics[ i ];
		}
		outKind  = kind;
		outLevel = level;
		return true;
	}

	void OccViewerControl::FitAllView( void )
	// Forwards fit-all to the native viewer
	{
		if( m_bInitialized ) {
			m_pNative->fitAll();
		}
	}

	void OccViewerControl::SetViewIso( void )
	// Forwards isometric view switch to the native viewer
	{
		if( m_bInitialized ) {
			m_pNative->setViewIso();
		}
	}

	void OccViewerControl::SetViewTop( void )
	// Forwards top view switch to the native viewer
	{
		if( m_bInitialized ) {
			m_pNative->setViewTop();
		}
	}

	void OccViewerControl::SetTcpTrailMode( int mode )
	// Forwards the trail visualisation mode to the native scene. Safe to call
	// before initialisation: the early-return mirrors the other view-control
	// methods so menu wiring during XAML construction is harmless.
	{
		if( m_bInitialized ) {
			m_pNative->setTcpTrailMode( mode );
		}
	}

	void OccViewerControl::ClearTcpTrail( void )
	// Clears the displayed TCP trail without disturbing the current mode.
	{
		if( m_bInitialized ) {
			m_pNative->clearTcpTrail();
		}
	}

	bool OccViewerControl::SaveScreenshot( String^ path )
	// Marshals the managed path to a native wide string and forwards to the native dumper
	{
		if( !m_bInitialized || path == nullptr ) {
			return false;
		}

		std::wstring nativePath = msclr::interop::marshal_as<std::wstring>( path );
		return m_pNative->saveScreenshot( nativePath.c_str() );
	}

	void OccViewerControl::SetJointLimits( cli::array<double>^ jointMinDeg,
										   cli::array<double>^ jointMaxDeg )
	// Marshals the managed limit arrays to native double[6] buffers and forwards to
	// the facade. Passing a nullptr or wrong-length array clears the cached limits
	// (the native side falls back to unbounded IK for drag mode).
	{
		if( !m_bInitialized ) {
			return;
		}
		if( jointMinDeg == nullptr || jointMaxDeg == nullptr
		 	|| jointMinDeg->Length < 6 || jointMaxDeg->Length < 6 ) {
			m_pNative->setJointLimits( nullptr, nullptr );
			return;
		}
		double minNative[ 6 ];
		double maxNative[ 6 ];
		for( int i = 0; i < 6; ++i ) {
			minNative[ i ] = jointMinDeg[ i ];
			maxNative[ i ] = jointMaxDeg[ i ];
		}
		m_pNative->setJointLimits( minNative, maxNative );
	}

	void OccViewerControl::SetDragEnabled( bool enabled )
	// Forwards the drag-mode toggle to the native facade. Pre-initialization calls
	// are swallowed so the HMI toggle can be wired up during XAML construction
	// without risking a crash before OnHandleCreated runs.
	{
		if( m_bInitialized ) {
			m_pNative->setDragEnabled( enabled );
		}
	}

	void OccViewerControl::SetGizmoMode( int mode )
	// Forwards the gizmo mode selection. Same pre-initialization guard as
	// SetDragEnabled so radio-button Checked handlers can fire during XAML load.
	{
		if( m_bInitialized ) {
			m_pNative->setGizmoMode( mode );
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
		if( m_bInitialized ) {
			m_pNative->onMouseDown( e->X, e->Y, static_cast<int>( e->Button ) );
		}
	}

	void OccViewerControl::OnMouseMove( MouseEventArgs^ e )
	// Forwards mouse-move to the native viewer for rotation, pan, or highlight update
	{
		UserControl::OnMouseMove( e );
		if( m_bInitialized ) {
			m_pNative->onMouseMove( e->X, e->Y, static_cast<int>( e->Button ) );
		}
	}

	void OccViewerControl::OnMouseUp( MouseEventArgs^ e )
	// Forwards mouse-up to the native viewer, then re-publishes the joint vector.
	// Drag mode commits IK results synchronously inside the native onMouseUp path,
	// so by the time we call GetJointAngles the kinematics core already holds the
	// post-drag joint state. Firing on every mouse-up (not just drag) keeps the
	// control ignorant of whether a drag was in progress and costs one native
	// call + 6 marshalled doubles per click.
	{
		UserControl::OnMouseUp( e );
		if( m_bInitialized ) {
			m_pNative->onMouseUp( );
			JointsChanged( GetJointAngles() );
		}
	}

	void OccViewerControl::OnMouseWheel( MouseEventArgs^ e )
	// Forwards mouse-wheel to the native viewer to zoom the 3D scene
	{
		UserControl::OnMouseWheel( e );
		if( m_bInitialized ) {
			m_pNative->onMouseWheel( e->Delta );
		}
	}

}
