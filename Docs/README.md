# RobotSimulation

A 6-DOF industrial robot arm simulator built with **WPF + C++/CLI + OpenCASCADE Technology (OCCT) 7.9.3**. Loads STEP CAD parts described by **Denavit–Hartenberg (DH) parameters** in a JSON configuration file, renders them in an interactive 3D scene, and drives the arm through forward kinematics, closed-form / iterative inverse kinematics, jerk-limited MoveL/MoveJ motion, a 6-DoF drag gizmo, singularity monitoring, and a teach-pendant waypoint player.

![Platform](https://img.shields.io/badge/platform-Windows%20x64-blue)
![.NET](https://img.shields.io/badge/.NET-Framework%204.8-512BD4)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C)
![OCCT](https://img.shields.io/badge/OCCT-7.9.3-orange)

---

## Features

**Modeling & rendering**
- **Import a robot model** from a folder containing `*.step` parts and one `*.json` config (DH params, axis limits, color, parent-child links).
- **Forward kinematics** with standard DH convention and parent-child cumulative transforms.
- **Per-axis colored trihedrons** — base frame in the corner (X=blue, Y=green, Z=red) and a TCP (Tool Center Point) trihedron that tracks the end-effector in real time.
- **TCP trail** — polyline travel history, optionally with sub-sampled mini-trihedrons; configurable length, stride, and color.
- **3D interaction** — left-button rotate, middle-button pan, mouse-wheel zoom, ISO/Top presets, Fit All.
- **Loading progress** displayed part-by-part on a colored status bar; **auto-resume** of the last imported folder from the registry; **screenshot** capture (format inferred from extension).

**Kinematics & motion**
- **Six joint sliders** plus press-and-hold **JOG** with axis limits read from JSON; live degree readout per joint.
- **Cartesian World JOG** — X/Y/Z/Rx/Ry/Rz nudges in the world frame (quaternion composition, no Euler drift).
- **Inverse kinematics** — closed-form **Pieper** analytical solver (spherical wrist) with a **Damped Least Squares (DLS)** fallback; joint-limit aware.
- **MoveL** (linear TCP path, per-frame IK) and **MoveJ** (joint-space) with **trapezoidal** and **7-segment S-curve** (jerk-limited) velocity profiles and **quaternion slerp** orientation blending.
- **Singularity monitor** — Jacobian-based arm/wrist/elbow/shoulder classification with Normal/Warning/Critical levels; automatic **speed scaling** near singularities.

**Interaction & teaching**
- **Drag gizmo** — AIS_Manipulator with Translate / Rotate modes drives IK on the TCP; **link drag** rotates a picked joint via screen-space tangent projection.
- **Teach pendant** — capture, edit, save/load (JSON) waypoints; **sequence player** (Play/Pause/Step/Loop/Stop) executes MoveL/MoveJ segments.
- **Status dashboard** — live joint angles, TCP pose, and a singularity badge.

---

## Project Layout

```
robot-simulation/
├── RobotSimulation.sln
├── Hmi/                       C# WPF front-end (.NET Framework 4.8, x64)
│   ├── App.xaml(.cs)
│   ├── MainWindow.xaml(.cs)   Tabs (Manual/Auto/Drag/Teach), JOG, MoveL/MoveJ, IK, JSON, registry
│   ├── Controls/              StatusDashboard (pose + singularity badge), WaypointGrid (teach)
│   ├── Models/                Waypoint, WaypointStore, WaypointFileFormat (JSON persistence)
│   ├── Motion/                IWaypointSegmentExecutor, WaypointSequencePlayer (state machine)
│   └── Hmi.csproj
├── OccBridge/                 C++/CLI bridge DLL (mixed managed + native facade)
│   ├── OccViewerControl.h/.cpp    ref class : WinForms UserControl (managed)
│   ├── IRobotScene.h              Pure-virtual facade interface (30 methods) + createRobotScene()
│   ├── RobotSceneFacade.h/.cpp    Concrete IRobotScene composing the helper libs (PIMPL)
│   ├── MotionProfileBridge.h/.cpp Managed wrapper over Motion:: profiles for the HMI
│   └── OccBridge.vcxproj
├── Viewer/                    Static lib — OCCT rendering stack (ViewportContext)
├── Scene/                     Static lib — STEP load (StepLoader), AIS slot mgmt (SceneRepository), TcpTrail
├── Interaction/               Static lib — MouseInteractor, CameraController,
│                              ManipulatorController (drag gizmo), JointDragController (link drag)
├── Kinematics/                Static lib — DH math (TransformBuilder), TCP pose (TcpPoseSolver),
│                              FK (RobotKinematics), IK (AnalyticalIkSolver + IkSolver/DLS),
│                              Jacobian, SingularityMonitor, Quaternion, PoseInterpolator,
│                              CartesianJogStepper, RobotPartDef POD
├── Motion/                    Static lib — MotionProfile base, Linear/Trapezoidal/SCurve profiles,
│                              SingularitySpeedScaler
├── Utility/                   Static lib — StringUtil (wideToUtf8)
├── Props/Local.Occt.props     MSBuild props (OcctIncludeDir, OcctLibDir, OcctRoot, OcctThirdPartyRoot)
├── Occt/                      Vendored OCCT 7.9.3 binaries (gitignored)
├── Data/CadFiles/             Sample robot models (R-LA906-7, R-LA580-4)
├── Data/Waypoints/            Saved teach-pendant waypoint sequences (JSON)
├── Figures/                   App icon (.avif source, .png embedded)
├── Docs/                      Architecture docs, Roadmap.md, RefactoringRoadmap.md
└── bin/                       Build output
```

---

## Architecture

Four layers, with the native side split into a thin facade plus six single-responsibility static libraries. The managed wrapper depends only on the abstract `IRobotScene` interface (DIP).

```mermaid
flowchart TB
    subgraph Managed[Managed - .NET Framework 4.8]
        UI[Hmi WPF<br/>MainWindow + Teach + Motion players]
        Bridge[OccBridge.dll C++/CLI<br/>OccViewerControl : UserControl<br/>MotionProfileBridge]
    end
    subgraph Facade[Native facade - OccBridge.dll]
        IRS[IRobotScene<br/>30 pure-virtual + createRobotScene]
        RSF[RobotSceneFacade<br/>PIMPL composes the libs]
    end
    subgraph Libs[Native static libs - C++17]
        VW[Viewer::ViewportContext]
        SC[Scene::StepLoader<br/>SceneRepository / TcpTrail]
        IN[Interaction::MouseInteractor / CameraController<br/>ManipulatorController / JointDragController]
        KN[Kinematics::RobotKinematics / Transform<br/>IkSolver / AnalyticalIkSolver / Jacobian<br/>SingularityMonitor / Quaternion / PoseInterpolator]
        MO[Motion::Linear / Trapezoidal / SCurve<br/>SingularitySpeedScaler]
        UT[Utility::wideToUtf8]
    end
    OCCT[OpenCASCADE 7.9.3<br/>TKV3d, TKOpenGl, TKG3d, TKDESTEP, ...]

    UI -- WindowsFormsHost --> Bridge
    Bridge -- IRobotScene* --> IRS
    IRS -. implemented by .-> RSF
    RSF --> VW
    RSF --> SC
    RSF --> IN
    RSF --> KN
    Bridge --> MO
    SC --> UT
    VW --> OCCT
    SC --> OCCT
    IN --> OCCT
    KN --> OCCT
```

| Layer | Responsibility |
|---|---|
| **Hmi (C#/WPF)** | UI, tabs, sliders, JOG, MoveL/MoveJ, IK requests, teach pendant, JSON parsing, registry persistence, folder dialog. No OCCT types. |
| **OccBridge (C++/CLI)** | Marshalling boundary. `OccViewerControl` holds `IRobotScene*` from `createRobotScene()` — never names the concrete facade. `MotionProfileBridge` exposes `Motion::` profiles to managed code. |
| **RobotSceneFacade (native)** | Implements `IRobotScene`; orchestrates the helper libraries via PIMPL. Every override is a one-to-three-line forward. |
| **Viewer / Scene / Interaction / Kinematics / Motion / Utility** | Single-purpose static libs. Each owns its slice of OCCT or pure math; tested independently. |

---

## Class Model

### `Hmi.MainWindow`
- `_viewer : OccViewerControl` — the hosted C++/CLI control
- `_robotLoaded`, `_jointLabels`, `_jogTimer` — manual JOG / slider UI state
- `_cartesianJog` — world-frame Cartesian JOG stepper wrapper
- `_moveLTimer` / `_moveJTimer` with their `Profile`, `PoseInterpolator`, and `SingularitySpeedScaler` — drive MoveL / MoveJ virtual-clock ticks
- `_waypointStore`, `_sequencePlayer` — teach-pendant model + sequence state machine (`MainWindow` implements `IWaypointSegmentExecutor`)
- `LoadRobotFromFolder(path)` — parses JSON, builds `RobotPartInfo[]`, calls `_viewer.LoadRobotArm`
- `JogTimer_Tick` — polls `_viewer.GetTcpPose()` / `GetManipulability()` while jogging and pushes the result to `StatusDashboard`
- `StartMoveJToJointTarget` / MoveL handlers — plan a profile then step joints per tick; reused by "Reset to Home"
- `ShowFolderDialog(title)` — Vista `IFileOpenDialog` COM interop (Windows auto-remembers last folder per-app)
- `GetLastModelFolder / SetLastModelFolder` — registry persistence at `HKCU\SOFTWARE\RobotSimulation\LastModelFolder`

### `Hmi` teach-pendant types (`Models/`, `Motion/`, `Controls/`)
- `Waypoint` — POCO: `Name`, `Pose[6]`, `MotionType` (`L`/`J`), `VRatio`, `ARatio`
- `WaypointStore` — `ObservableCollection<Waypoint>` with CRUD, `CaptureFromPose`, `NextAutoName`
- `WaypointFileFormat` — JSON DTO for save/load under `Data/Waypoints/`
- `IWaypointSegmentExecutor` / `WaypointSequencePlayer` — executor interface + Idle/Running/Paused/Completed/Aborted state machine
- `StatusDashboard` (control) — live joint angles, TCP pose, singularity badge; `WaypointGrid` (control) — teach DataGrid CRUD

### `OccBridge::OccViewerControl` (ref class : `UserControl`)
- `IRobotScene* m_pNative` — obtained from `createRobotScene()`; released in finalizer `!OccViewerControl`. Wrapper never names `RobotSceneFacade` directly.
- `m_bInitialized` — OCCT init is deferred until `OnHandleCreated` (HWND must exist)
- Public methods: `LoadStep`, `LoadRobotArm`, `SetJointAngle(s)`, `GetJointAngles`, `GetTcpPose`, `SolveTcpIk`, `SetJointLimits`, `GetManipulability`, `SetDragEnabled`, `SetGizmoMode`, `SetTcpTrailMode`/`ClearTcpTrail`/`SetTcpTrailMaxPoints`/`SetTcpTrailFrameStride`/`SetTcpTrailColor`, `ClearScene`, `FitAllView`, `SetViewIso`, `SetViewTop`, `SaveScreenshot`
- `event JointsChanged` — raised after mouse-up so the HMI can resync its cached joint vector when a drag/IK path commits angles outside `SetJointAngle`
- Overrides: `OnHandleCreated`, `OnResize`, `OnPaint`, `OnMouseDown/Move/Up/Wheel`

### `OccBridge::RobotPartInfo` (managed ref class)
Carries JSON-parsed fields across the managed/native boundary: `FilePath`, `DH_a/alpha/d/theta`, `Offset[6]`, `ParentIdx`, `ColorR/G/B`. Marshalled into the native `RobotPartDef` POD in `LoadRobotArm`.

### `OccBridge::MotionProfileBridge` (managed ref class)
Thin C++/CLI wrapper exposing the native `Motion::` velocity profiles (Linear / Trapezoidal / S-curve) to the WPF layer so MoveL/MoveJ tick handlers can sample a normalized arc-length `s(t)` without owning OCCT or native math types.

### `OccBridge::IRobotScene` (abstract interface)
Forward-declares `HWND` and `RobotPartDef` only; no OCCT headers. Defines **30 pure-virtual methods** mirroring the facade's public surface — lifecycle, geometry loading, FK (`setJointAngle(s)`, `getJointAngles`, `getTcpPose`), IK (`solveTcpIk`, `setJointLimits`), singularity (`getManipulability`), drag (`setDragEnabled`, `setGizmoMode`), camera, and TCP trail — plus the nested enums `IkSolveStatus`, `GizmoMode`, `TcpTrailMode` and a free-function factory:

```cpp
[[nodiscard]] IRobotScene* createRobotScene();
```

Returns a raw pointer rather than `unique_ptr` because C++/CLI ref classes cannot hold `std::unique_ptr` member fields.

### `OccBridge::RobotSceneFacade` (concrete `IRobotScene`, PIMPL)
Public API hides OCCT entirely; the implementation composes the native helpers and a `partToSlot` map (part index → `SceneRepository` slot; `-1` = load failure):

```cpp
struct RobotSceneFacade::Impl {
    Viewer::ViewportContext         viewport;
    Scene::SceneRepository          repo;
    Scene::TcpTrail                 trail;         // TCP travel-history display
    Interaction::MouseInteractor    mouse;
    Interaction::CameraController   camera;
    Interaction::ManipulatorController dragGizmo;  // 6-DoF TCP drag (Translate/Rotate)
    Interaction::JointDragController   jointDrag;  // click-a-link joint drag
    RobotKinematics                 kin;           // FK + IK seed / limits
    std::vector<int>                partToSlot;
};
```

Every `override` is a one-to-three-line forward to one of the helpers; `updateRobotTransforms` (which also repositions the drag gizmo and pushes a TCP-trail sample) is the only non-trivial private method.

### `Viewer::ViewportContext`
Owns the OCCT rendering stack (`Aspect_DisplayConnection`, `OpenGl_GraphicDriver`, `V3d_Viewer`, `V3d_View`, `AIS_InteractiveContext`, `HWND`). Methods: `initialize(HWND)`, `resize`, `redraw`, `view()`, `context()`. Sets up default lights, gray30 background, lower-left corner trihedron, and the initial isometric projection.

### `Scene::StepLoader`
Single method `[[nodiscard]] std::optional<TopoDS_Shape> read(std::wstring_view path)`. Uses `Utility::wideToUtf8` then `STEPControl_Reader`; returns `std::nullopt` on any failure (no exceptions).

### `Scene::SceneRepository`
Owns `vector<Handle(AIS_Shape)>` plus an optional TCP `AIS_Trihedron`. Returns a `SlotResult { int slotId; bool isValid; }` from `addShape` / `addColoredShape` so failures never shift other slots. Methods: `attach(ctx)`, `addShape`, `addColoredShape`, `setTransform(slotId, trsf)`, `ensureTcpTrihedron()`, `setTcpTransform`, `tcpFrame()`, `clear`, `updateViewer`.

### `Scene::TcpTrail`
Ring-buffered TCP travel history with `enum class Mode { Off, Polyline, PolylineWithFrames }`. `pushPose()` is cheap (single comparison) while `Off`; otherwise it appends to the buffer and rebuilds the polyline (and sub-sampled mini-trihedrons at `frameStride`). Configurable `maxPoints`, `frameStride`, and polyline color. Methods: `attach(ctx)`, `setMode`, `pushPose`, `clear`, plus setters for the tunables.

### `Interaction::MouseInteractor`
State machine for rotate / pan / wheel. `enum class MouseButton { Left = 1048576, Middle = 4194304 };` matches the WinForms bitmask values forwarded by `OccViewerControl`. Methods: `attach(view, context)`, `onMouseDown/Move/Up/Wheel`.

### `Interaction::CameraController`
`attach(view)`, `setViewIso()` (V3d_XposYnegZpos + fitAll), `setViewTop()` (V3d_Zpos + fitAll), `fitAll()` (FitAll + ZFitAll + Redraw).

### `Interaction::ManipulatorController`
Wraps an `AIS_Manipulator` 6-DoF gizmo anchored to the TCP trihedron. `enum class GizmoMode { Translate, Rotate }` toggles the visible handle set. On each drag it reads the proposed world transform via `ObjectTransformation` (non-applying) and fires a callback so the facade can run IK; the gizmo re-syncs to the committed TCP frame. Methods: `attach`, `setAnchor`, `setEnabled`, `setGizmoMode`, `onMouseDown/Move/Up`, `syncToPose`.

### `Interaction::JointDragController`
Joint (link) drag mode: on mouse-down it resolves the picked `AIS_Shape` to its driving axis, projects the axis's world direction into screen space, and converts subsequent mouse motion into a joint-angle delta reported through a callback. Methods: `attach`, `setEnabled`, `onMouseDown/Move/Up`.

### `OccBridge::RobotKinematics`
Forward-kinematics solver (namespace `OccBridge`, in `Kinematics/`). Pre-allocates `m_partJointDelta`, `m_dhCumulative`, and `std::array<double, 6> m_jointAngles` once in `configure()` so `setJointAngle` → `computeCumulative` stays heap-allocation-free in the steady state. Methods: `configure`, `setJointAngle`, `setJointAngles`, `computeCumulative`, `computeFinal`, `tcpFrame`, `parts()`, `axisToPartMap()`, `jointAngles()`.

### Inverse kinematics — `solveIkAnalytical` / `solveIkDls`
Two solvers in `Kinematics/AnalyticalIkSolver.h` and `Kinematics/IkSolver.h`, sharing `IkOptions` / `IkResult` / `enum class IkStatus`.
- **`solveIkAnalytical`** — closed-form **Pieper** solution for a spherical wrist (O(1)): solves the wrist-center geometry then ZYZ Euler on the wrist, generates up to 8 candidate branches, and filters by joint limits.
- **`solveIkDls`** — **Damped Least Squares** iterative solver (Jacobian transpose with damping, per-iteration step clamp, position/orientation tolerances). Used as the fallback when the analytical geometry is invalid, and directly during drag.

### `Jacobian` (namespace)
`Kinematics/Jacobian.h`: `build(kin, Matrix6x6&)`, `determinant(J)`, `manipulability(J)` = `|det(J)|`, and `subDeterminant3x3(J, row0, col0)` for the arm/wrist sub-blocks used by the singularity monitor.

### `SingularityMonitor` — `evaluate`
`Kinematics/SingularityMonitor.h`: `evaluate(kin, thresholds) → SingularityReport`. Classifies the current pose via arm vs wrist Jacobian sub-manipulability into `enum class SingularityKind { None, Wrist, Elbow, Shoulder, Combined }` and `SingularityLevel { Normal, Warning, Critical }` against dimensionless `SingularityThresholds`.

### `Quaternion` / `PoseInterpolator` / `CartesianJogStepper`
- **`Quaternion`** (`Kinematics/Quaternion.h`) — `fromZyxDeg` / `toZyxDeg`, `normalise`, `dot`, `slerp` (shortest-arc, near-parallel fallback), `fromAxisAngleRad`, `multiply`. Used for drift-free orientation interpolation and world-frame composition.
- **`PoseInterpolator`** (`Kinematics/PoseInterpolator.h`) — stateful segment slerp: `begin(start, target)` caches the endpoint quaternions + geodesic angle once, then `sample(s)` returns the blended `[x,y,z,rx,ry,rz]` pose; `totalAngleRad()` is O(1).
- **`CartesianJogStepper`** (`Kinematics/CartesianJogStepper.h`) — world-frame open-loop jog: `step(currentPose, axis, dir, dt)` bumps position axes in Cartesian space and orientation axes via world-frame quaternion composition.

### `OccBridge::Transform` / `OccBridge::solveTcpPose`
Pure functions in `Kinematics/TransformBuilder.h` and `Kinematics/TcpPoseSolver.h`. `Transform::makeDh(a, alpha, d, theta)` and `Transform::makeOffset(tx, ty, tz, rx, ry, rz)` return `gp_Trsf` by value. `solveTcpPose(gp_Trsf)` decomposes to `std::array<double, 6>` `[x, y, z, rx, ry, rz]` in mm + degrees (ZYX intrinsic Euler, gimbal-lock safe).

### Motion profiles — `Motion::Profile` and subclasses
`Motion/MotionProfile.h` defines the abstract `Profile` (`durationSec()`, `sample(elapsedSec) → s ∈ [0,1]` normalized arc-length). Subclasses differ only in the shape of `s(t)`:
- **`LinearProfile`** — constant velocity, `s(t) = t/T` (baseline MoveL).
- **`TrapezoidalProfile`** — ramp-up / cruise / ramp-down; static `plan(distance, vMax, aMax)` auto-degrades to triangular.
- **`SCurveProfile`** — 7-segment jerk-limited; static `plan(distance, vMax, aMax, jMax)` handles no-cruise / no-const-accel degeneracies analytically.
- **`SingularitySpeedScaler`** — `scale(ratio) → factor ∈ [0,1]` piecewise-linear tempo scaling as the pose nears a singularity, with a one-shot `shouldAnnounceCritical()` latch. MoveL ticks multiply their virtual clock by this factor.

### `RobotPartDef` (native POD struct, in `Kinematics/`)
```cpp
struct RobotPartDef {
    std::wstring filePath;
    double dhA = 0.0, dhAlpha = 0.0, dhD = 0.0, dhTheta = 0.0;
    double offset[6] = {};    // tx, ty, tz, rx_deg, ry_deg, rz_deg
    int parentIdx = -1;       // -1 for root
    int colorR = 200, colorG = 200, colorB = 200;
};
```

### `Utility::wideToUtf8`
`std::string wideToUtf8(const wchar_t*)` and a `std::wstring_view` overload. Uses Win32 `WideCharToMultiByte`. Called only on STEP load (cold path).

---

## Kinematics

For each part $i$, the local DH transform follows the standard convention:

$$
T_i^{\text{local}} = R_z(\theta_i + \Delta\theta_i)\, T_z(d_i)\, T_x(a_i)\, R_x(\alpha_i)
$$

where $\Delta\theta_i$ is the slider-driven joint angle for the axis mapped to part $i$ (0 if the part is not joint-driven).

Cumulative transform along the parent chain:

$$
T_i^{\text{cum}} = T_{\text{parent}(i)}^{\text{cum}} \cdot T_i^{\text{local}}
$$

A fixed **offset transform** brings the CAD shape's authored frame into the DH joint frame:

$$
T_i^{\text{offset}} = T(t_x, t_y, t_z) \cdot R_z(r_z) \cdot R_y(r_y) \cdot R_x(r_x)
$$

Final world placement applied via `AIS_Shape::SetLocalTransformation`:

$$
T_i^{\text{world}} = T_i^{\text{cum}} \cdot T_i^{\text{offset}}
$$

The TCP trihedron is placed at $T_{n-1}^{\text{cum}}$ (the last DH frame, conventionally the end-effector).

> **Note:** OCCT's `gp_Trsf::Multiply(other)` post-multiplies (`this = this * other`), so the order of `Multiply` calls in code reads left-to-right.

### Inverse kinematics

The primary solver is closed-form (**Pieper**) for a spherical wrist: the wrist center is $p_c = p_{\text{tcp}} - d_6\,R\,\hat z$, the first three axes are solved from $p_c$, and the last three from the ZYZ decomposition of $R_3^{-1} R$. When the geometry is invalid (or the arm is non-spherical) it falls back to **Damped Least Squares**, iterating

$$
\Delta q = J^{\mathsf T}\left(J J^{\mathsf T} + \lambda^2 I\right)^{-1} e
$$

where $e$ is the 6-vector pose error and $\lambda$ the damping factor, with each $\Delta q$ clamped per axis and joint limits enforced between iterations.

### Singularity metric

Singularity proximity uses the manipulability measure $w = \lvert\det J\rvert$, split into arm and wrist sub-blocks $\lvert\det J_{0:3,0:3}\rvert$ and $\lvert\det J_{3:6,3:6}\rvert$. The dimensionless ratios are compared against Warning / Critical thresholds to classify the singularity kind and drive the MoveL speed scaler.

---

## Key Sequences

### Startup with auto-load

```mermaid
sequenceDiagram
    participant W as MainWindow
    participant V as OccViewerControl
    participant F as RobotSceneFacade<br/>(via IRobotScene*)
    participant R as SceneRepository
    participant K as RobotKinematics

    W->>W: ctor reads HKCU LastModelFolder
    W->>V: new OccViewerControl
    V->>F: createRobotScene()
    Note over W: WPF Loaded event fires
    W->>W: LoadRobotFromFolder
    W->>V: LoadRobotArm parts, map, progress
    V->>F: beginRobotArm
    F->>K: configure(parts, axisMap)
    loop for each part i
        V->>W: progress i+1 of n
        Note over W: SetStatus Loading orange
        V->>F: loadRobotPart i
        F->>R: addColoredShape -> SlotResult
    end
    V->>F: endRobotArm
    F->>R: ensureTcpTrihedron
    F->>F: updateRobotTransforms + fitAll
    Note over W: SetStatus Loaded green
    W->>W: SetLastModelFolder
```

### Joint slider movement (hot path)

```mermaid
sequenceDiagram
    participant S as Slider J3
    participant W as MainWindow
    participant V as OccViewerControl
    participant F as RobotSceneFacade
    participant K as RobotKinematics
    participant R as SceneRepository

    S->>W: ValueChanged angle, Tag=2
    W->>V: SetJointAngle 2, angle
    V->>F: virtual setJointAngle 2, angle
    F->>K: setJointAngle 2, angle
    Note over K: write m_jointAngles[2] (no alloc)
    F->>F: updateRobotTransforms
    F->>K: computeCumulative
    Note over K: std::fill + index writes on pre-sized buffers
    loop for each part i
        F->>K: computeFinal i
        F->>R: setTransform slotId, trsf
        Note over R: AIS_Shape::SetLocalTransformation
    end
    F->>R: setTcpTransform last DH frame
    F->>R: updateViewer
```

---

## JSON Model Format

The model folder must contain exactly one `*.json` file plus the referenced `*.step` files (resolved by **filename only** relative to the JSON folder):

```json
{
  "Name": "R-LA906-7",
  "AxisNum": 6,
  "AxisLimits": "[[-170,170],[-96,130],[-195,65],[-170,170],[-120,120],[-360,360]]",
  "AxisToPartMap": "[[1,1],[2,2],[3,3],[4,4],[5,5],[6,6]]",
  "PartInfos": [
    {
      "CadFilePath": "...\\R-LA906-7_BASE.step",
      "CadColor": "[42,41,42]",
      "Offset": "[0,0,0,0,0,0]",
      "ParentDHIdx": -1,
      "a": 0, "alpha": 0, "d": 0, "theta": 0
    },
    {
      "CadFilePath": "...\\R-LA906-7_1.step",
      "CadColor": "[241,240,234]",
      "Offset": "[0,0,0,90,0,0]",
      "ParentDHIdx": 0,
      "a": 30, "alpha": -90, "d": 380, "theta": 0
    }
  ]
}
```

| Field | Meaning |
|---|---|
| `AxisLimits` | Per-axis `[min_deg, max_deg]`; applied to sliders on load. |
| `AxisToPartMap` | `[axis_index_1_based, part_index_0_based]` pairs. |
| `Offset` | `[tx, ty, tz, rx_deg, ry_deg, rz_deg]` from CAD authored frame to DH joint frame. |
| `ParentDHIdx` | Parent part index; `-1` marks root. Parts must be listed in topological order. |
| `a, alpha, d, theta` | Standard DH parameters (distances in mm, angles in degrees). |

Sample models in `Data/CadFiles/`: `R-LA906-7` and `R-LA580-4`.

---

## Build

### Prerequisites

- **Visual Studio 2022 Professional** (toolset v143) with:
  - .NET Framework 4.8 SDK
  - Desktop development with C++
  - C++/CLI support
- **OCCT 7.9.3 vc14 x64** binary distribution placed at `Occt/opencascade-7.9.3-vc14-64/` (path configured in `Props/Local.Occt.props`)

### Steps

1. Open `RobotSimulation.sln` in Visual Studio.
2. Select **Debug | x64** or **Release | x64** (32-bit is unsupported).
3. Build → `OccBridge.dll` then `Hmi.exe` land in `bin/Debug/` (or `bin/Release/`). A post-build step copies all OCCT and third-party runtime DLLs (FreeType, TBB, FreeImage, FFmpeg, jemalloc, OpenVR) into the output folder.
4. Run `Hmi.exe`. Use **File → Import Model** to pick `Data/CadFiles/R-LA906-7/` for a quick test.

### Build Notes

- The solution has **eight** projects: `Hmi` (C# WPF), `OccBridge` (C++/CLI DLL), and six native C++17 static libs (`Viewer`, `Scene`, `Interaction`, `Kinematics`, `Motion`, `Utility`). `OccBridge` references them via `<ProjectReference>`.
- `OccBridge.vcxproj` is mixed-mode. The whole project compiles with `/clr`, **except** `RobotSceneFacade.cpp` which has a per-file override:
  ```xml
  <CompileAsManaged>false</CompileAsManaged>
  <ExceptionHandling>Sync</ExceptionHandling>
  ```
  This is required because OCCT classes such as `Geom_Axis2Placement` export native-mangled symbols that cannot be resolved by `/clr` translation units (which add a `$$F` to the mangle, producing `LNK2028`).
- Common native flags: `/std:c++17 /utf-8 /FS`. Warning `4996` suppressed in projects that transitively include `V3d_View` (OCCT 7.9 deprecates `Handle_Graphic3d_CLight`).
- OCCT include / lib paths come from `Props/Local.Occt.props` (`$(OcctIncludeDir)`, `$(OcctLibDir)`, `$(OcctRoot)`, `$(OcctThirdPartyRoot)`); each lib imports the props file once.
- OCCT libraries linked by `OccBridge`: `TKernel, TKMath, TKG3d, TKService, TKV3d, TKOpenGl, TKBRep, TKTopAlgo, TKDE, TKDESTEP, TKXSBase`. Note that `Geom_*` classes belong to **TKG3d**, not `TKGeomBase`.
- Static libs land in `bin/$(Configuration)/*.lib`; intermediates go to `x64/$(Configuration)/$(ProjectName)/`.
- Hmi window icon comes from `Figures/robot-icon.png` (embedded as `<Resource>`, set in code-behind — an ICO file triggered `XamlParseException`).

---

## Threading & UI Responsiveness

All OCCT calls run on the WPF UI thread. To avoid freezing during large STEP loads, the bridge splits `LoadRobotArm` into three phases:

1. `beginRobotArm` — stores metadata
2. Loop calling `loadRobotPart(i)` — loads one STEP file per call, invoking the progress callback between parts
3. `endRobotArm` — finalizes transforms + creates TCP trihedron

The progress callback uses a `DispatcherFrame` to pump pending paint messages so the status bar visibly updates between parts:

```csharp
var frame = new DispatcherFrame();
Dispatcher.CurrentDispatcher.BeginInvoke(DispatcherPriority.Background,
    new Action(() => frame.Continue = false));
Dispatcher.PushFrame(frame);
```

A plain `Dispatcher.Invoke` is **not** sufficient — it queues without flushing the render queue.

---

## Hot Path — Allocation Budget

`OccViewerControl::SetJointAngle` is the steady-state hot path (driven by the WPF `_jogTimer` and slider events). Verified zero native heap allocations end-to-end (see `Docs/RefactoringRoadmap.md` §R7):

| Step | Behavior | Native alloc |
|---|---|---|
| `RobotKinematics::setJointAngle` | one `double` write into `m_jointAngles` | 0 |
| `RobotKinematics::computeCumulative` | `std::fill` + index writes on pre-sized `m_partJointDelta` / `m_dhCumulative` | 0 |
| `RobotKinematics::computeFinal` | `gp_Trsf` stack values via `Transform::makeDh` / `makeOffset` | 0 |
| `SceneRepository::setTransform` | `AIS_Shape::SetLocalTransformation` — no re-tessellation | 0 |
| `SceneRepository::setTcpTransform` | `SetLocalTransformation` + cache to `std::optional` | 0 |
| `SceneRepository::updateViewer` | OCCT redraw scheduling | OCCT internal |

`push_back` / `emplace_back` / `new` are confined to cold paths (`addShape`, `beginRobotArm`, `createRobotScene`, `Impl` ctor). No `std::string` / `std::wstring` / `to_string` usage in any control-path translation unit; `Utility::wideToUtf8` only runs inside `Scene::StepLoader::read`.

---

## Coding Conventions

- **C++17**, brace style `if( cond ) {` with spaces inside parens, `array[ i ]` with spaces inside brackets, `for( ... ) {` on one line, `( void )` for no-argument legacy signatures.
- **Embedded / real-time policy** (per `.github/copilot-instructions.md`): no exceptions, no Hot Path heap allocation, `[[nodiscard]]` on `bool` / `std::optional` / error returns, smart pointers for ownership, `enum class`, fixed-width integers (`int32_t` etc.), `std::string_view` for read-only string params.
- **`OccBridge/IRobotScene.h` is OCCT-free** — only forward-declares `HWND` / `RobotPartDef`. `RobotSceneFacade.h` includes `IRobotScene.h` and hides OCCT behind a PIMPL `struct Impl`. The native static-lib headers (`Viewer`, `Scene`, `Interaction`, `Kinematics`, `Motion`) freely include the OCCT headers they need — they are linked into `OccBridge.dll` directly.
- Inline summary comments live **between the function signature and the opening brace**, not before the signature.
- **C#** follows the same spacing rules; `void` parameter syntax is omitted (unsupported in C#).
- All UI strings in English.

---

## Extension Points

| To add ... | Touch ... |
|---|---|
| A new view preset | `Interaction::CameraController::setViewXxx` + add a pure-virtual to `IRobotScene` + override in `RobotSceneFacade` + `OccViewerControl::SetViewXxx` + a `MenuItem` in `MainWindow.xaml` |
| 7th axis | Bump `m_jointAngles` from `std::array<double, 6>` to 7 and widen the bound check in `RobotKinematics::setJointAngle`; add `SliderJ7` |
| Forward kinematics readout | `RobotKinematics::tcpFrame()` already returns the last DH frame; expose a richer struct via a new `IRobotScene` method |
| Inverse kinematics | New static lib consuming `RobotKinematics::parts()` + a target pose; call `setJointAngle` for each solved axis |
| Collision detection | `BRepExtrema_DistShapeShape` on pairs of shapes held by `Scene::SceneRepository` after applying their cumulative transform |
| Alternate scene back-end (testing) | Implement `IRobotScene` with a mock; inject via a future ctor overload on `OccViewerControl` that takes `IRobotScene*` directly |

---

## License

This repository contains a runtime copy of OpenCASCADE Technology (LGPL-2.1 with OCCT exception). See `Occt/opencascade-7.9.3-vc14-64/LICENSE_LGPL_21.txt` and `OCCT_LGPL_EXCEPTION.txt`.
