# Matrix Coordinate Lab · 3D Robotics Lab

**English** · [简体中文](README.md)

Connect matrix calculations with spatial motion by dragging coordinate frames and changing rotation and joint angles. The lab is built for people learning ROS2, TF, URDF, and robot control—not as a formula-only calculator.

[Open the lab](https://s1eep1y416-desig.github.io/matrix-coordinate-lab/) · [GitHub source](https://github.com/s1eep1y416-desig/matrix-coordinate-lab)

## What you can do

| Module | Implemented interactions |
| --- | --- |
| Frames and transforms | World / Frame A; add child frames at any depth; reparent, show, hide, and select frames; drag origins and axis tips; enter exact poses; lock and limit degrees of freedom; inspect relative, inverse, and step-by-step parent-child transform chains. |
| Point conversion | Keep Point P fixed in space, switch source and target frames, and update coordinates and transform matrices in real time. |
| Rotation matrices | Adjust Rx / Ry / Rz independently; compare both orders for XY / YZ / ZX; play, pause, and scrub through the first and final steps. |
| Active / passive rotation | Rotate the gold vector in active mode; keep the world vector fixed and inspect its rotating-frame coordinates in passive mode. |
| Rotation representations | Six Euler orders, ZYX / RPY, quaternion, and Axis-Angle; preserve either pose or angle values when changing order; detect gimbal lock. |
| 3-Link FK | Z-Y-Y teaching skeleton, end-effector position and orientation, and step-by-step matrix-chain playback. |
| Position IK and trajectories | Analytic position Jacobian, damped least squares, descent line search, periodic base-joint wrapping across ±180°, analytic-branch recovery, and full-workspace reachability; draggable target, quintic minimum-jerk trajectories, scrubbing, playback controls, and a glowing end-effector trail. |
| Pinocchio workflow | Explain URDF → Model → q → FK → Frame placements with the same teaching skeleton. This is not a live Pinocchio runtime. |

The interface uses a white-and-green control area and a black-and-gold 3D scene. Axes follow the standard colors: X red, Y green, Z blue. Middle-button dragging orbits the 3D view, and the scene provides fullscreen / exit-fullscreen controls. The top-right Chinese / English button switches navigation, controls, teaching text, validation results, and scene hints without changing mathematical notation or the current experiment state.

## Run locally

### Easiest: open the offline edition

Download and extract the repository ZIP, then double-click **`Matrix-Coordinate-Lab-offline.html`** in the repository root. It contains its JavaScript, styles, equation fonts, and icon, so it needs neither Node.js nor a network connection or local server. Use a current WebGL-capable version of Chrome, Edge, Safari, or Firefox.

### Run the development edition

Requires Node.js **20.19+ (20.x) or 22.12+**, with 22.12+ recommended.

1. Choose **Code → Download ZIP** on GitHub and extract it, or use `git clone`.
2. Install [Node.js 22](https://nodejs.org/) once.
3. Double-click `start.command` on macOS or `start.bat` on Windows. On Linux, run `./start.sh`.

The launcher installs dependencies on the first run and opens `http://127.0.0.1:5173/`. Keep the terminal window open while using the lab. If macOS blocks the first launch, right-click `start.command` and choose Open, or run `chmod +x start.command && ./start.command` in Terminal.

### Start from a terminal

```bash
npm ci
npm start
```

Open the local URL printed by the terminal, normally `http://127.0.0.1:5173/`.

```bash
npm test        # Math and state regression tests
npm run build   # TypeScript check + dist/ build
npm run build:offline  # Also regenerate the single-file offline edition
```

Stack: React 19, TypeScript, Three.js, React Three Fiber, Drei, Zustand, KaTeX, Vite, and Vitest.

## Controls

- **Middle-button drag:** orbit the camera. **Mouse wheel:** zoom. Use the upper-right scene button to reset the view.
- **Left-drag a child-frame origin:** translate it in the current view plane. Drag a colored axis tip to change orientation. Drag P or the IK Target to move the point.
- **Drag a rotation number left or right:** adjust the angle. Hold Shift for fine control. Click the field to enter an exact value, then press Enter or blur to commit.
- **Scroll the right control panel:** move through parameters. On narrow screens, controls are placed below the scene.
- **Rotation demo:** choose an axis pair, replay from the start, jump to Initial / Step 1 / Final, or scrub the timeline. Changing target angles or modules pauses playback.
- **Degrees of freedom and limits:** expand the panel to set translation / Euler ranges and locks independently for each frame.
- **Transform-chain playback:** select a child frame at any depth, then click World, A, B… in “Step through transform chain.” The gold path, local matrices, and accumulated matrix advance together.

Demo angles and scene state live only in page memory and reset on refresh.

## Mathematical conventions

### Frames, units, and notation

- Right-handed coordinates with world **Z up**; distances are in meters.
- UI angle inputs use **degrees**; trigonometric functions and Jacobian joint increments use **radians**.
- Column vectors are used, so the rightmost matrix acts first.
- `^A T_B` is the pose of Frame B relative to Frame A and maps point coordinates from B into A.
- The canonical pose state is `position: Vector3` plus a unit `quaternion: Quaternion`. Euler angles, rotation matrices, and Axis-Angle are derived from the quaternion rather than maintained as separate truth states.
- Quaternion order is `[x, y, z, w]`; `q` and `-q` represent the same orientation.
- Three.js stores matrices internally in column-major order. The UI displays the conventional row/column layout and does not treat storage order as a transpose.

```text
p_A = R_A_B · p_B + position_A_B
^A T_B = [ R_A_B  position_A_B ]
         [ 0 0 0       1      ]
^W T_B = ^W T_A · ^A T_B
^A T_W = inverse(^W T_A)
inverse([R p; 0 1]) = [Rᵀ  -Rᵀp; 0 1]
```

The inverse translation is **−Rᵀp**, not simply −p. Changing the frame used to describe Point P does not move P in world space.

### Signs and rotation order

Let `c = cos(θ)` and `s = sin(θ)`. Active rotation matrices are:

```text
Rx = [1  0  0]    Ry = [ c  0  s]    Rz = [c -s  0]
     [0  c -s]         [ 0  1  0]         [s  c  0]
     [0  s  c]         [-s  0  c]         [0  0  1]
```

Independent ±90° checks:

| Input vector | +90° active rotation | −90° active rotation |
| --- | --- | --- |
| Rx applied to +Y | +Z | −Z |
| Ry applied to +X | **−Z** | +Z |
| Rz applied to +X | +Y | −Y |

Positive angles follow the right-hand rule, not the apparent clockwise or counterclockwise direction on the current screen. In particular, a positive rotation about +Y moves +X toward −Z; this is not a sign error.

Active rotation uses `v′ = Rv`. Passive rotation turns the reference frame while the world vector stays fixed, giving `v_local = Rᵀv_world`. After the frame rotates +90° about Z, a fixed world +X vector therefore has coordinates `[0, -1, 0]` in the new frame.

On the comparison page, `RxRy` means rotate first about the fixed Y axis and then about the fixed X axis. The two displayed origins are offset only for visual comparison and are not included in the rotation calculation. Progress `0 → 1` applies the rightmost matrix; `1 → 2` applies the left matrix.

Euler angles follow the Three.js intrinsic-sequence convention. Inputs always correspond to X / Y / Z components. ZYX / RPY corresponds to `Rz(yaw) Ry(pitch) Rx(roll)` and is equivalent to fixed-axis rotations X → Y → Z. Do not confuse the sequence name, written matrix order, and fixed-axis execution order.

Euler representations are not unique. Near ±180° boundaries or gimbal lock, readouts can jump to an equivalent branch. Axis-Angle uses a 0–180° angle with a corresponding axis direction, so a negative angle may appear as a reversed axis plus a positive angle. Neither behavior means the spatial orientation has reversed.

### Teaching robot and IK

The current skeleton has Z-Y-Y revolute joints. Links extend along local +X with lengths `1.25 / 1.00 / 0.80 m`. Each transform rotates first and then translates along the rotated +X axis, illustrating generic recursive kinematics.

```text
T_base_tool = T_base_link1 · T_link1_link2 · T_link2_link3
```

At zero pose, the end effector is `[3.05, 0, 0]`. With only q2 at +90°, it is `[1.25, 0, -1.80]`; with q2 at −90°, it is `[1.25, 0, +1.80]`.

Position IK uses:

```text
e = p_target − p_current
J_i = axis_i × (p_end − p_joint_i)   # all expressed in base
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
q_next = q + Δq
```

`q1` is a periodic base joint and uses equivalent-angle wrapping across `+180° / −180°`. If local DLS still stalls near a singularity or boundary, the solver resumes from the closest analytic Z-Y-Y branch. Reachability uses the full 3-Link workspace rather than only the maximum-reach sphere.

Trajectory planning uses the current joint angles as the start, the IK solution as the goal, and the quintic minimum-jerk time law `s(u)=10u³−15u⁴+6u⁵`. Every sample is evaluated through FK, so the solid glow and fading trail show the end effector's actual spatial path rather than a decorative curve.

## Project structure

```text
src/
├── math/                       # Computation separated from React / 3D rendering
│   ├── rotation.ts             # Axis rotations, order comparison, active/passive vectors, validation
│   ├── euler.ts                # Six Euler orders
│   ├── quaternion.ts           # Normalization, Axis-Angle, orientation equivalence
│   ├── transform.ts            # Pose, composition, inverse transforms, point conversion
│   ├── kinematics.ts           # 3-Link FK
│   ├── inverseKinematics.ts    # Jacobian / DLS position IK
│   ├── trajectory.ts           # Minimum-jerk joint trajectory and FK path samples
│   ├── math.test.ts
│   ├── trajectory.test.ts
│   └── signs.test.ts           # Independent analytic sign checks
├── components/
│   ├── CoordinateFrame.tsx     # RGB axes, transparent labels, dragging
│   ├── RotationScene.tsx       # A/B rotation and vector comparison
│   ├── RotationPlayback.tsx    # Play, pause, steps, and scrub control
│   ├── Robot.tsx               # Teaching robot skeleton
│   ├── Scene.tsx               # Camera, points, and scene composition
│   ├── MathView.tsx            # Matrices, formulas, and numeric readouts
│   └── NumberField.tsx         # Exact input and drag-to-adjust values
├── stores/
│   ├── labStore.ts             # Poses, hierarchy, constraints, interaction state
│   ├── labStore.test.ts
│   └── localeStore.ts          # Chinese / English UI state and copy selection
├── App.tsx
└── styles.css
```

## Validation and current limits

`npm test` covers six Euler round trips, quaternion / matrix round trips, `q ≡ −q`, `T · inverse(T) ≈ I`, multi-level transform chains, point conversion, FK, analytic Jacobians against finite differences, IK error reduction, minimum-jerk endpoints and FK samples, constraints, and playback state. Sign tests use explicit sin / cos matrices and known endpoints rather than relying only on self-consistent round trips.

The page checks `RᵀR ≈ I`, `det(R) ≈ 1`, unit quaternion norm, and the homogeneous final row in real time, using floating-point tolerances instead of strict equality. A typical validity tolerance is `1e-8`. Display values are rounded; calculations retain full precision.

Current limits:

- There is no live URDF import, mesh loader, or Pinocchio Python/WASM runtime. The Pinocchio page teaches the workflow using this project’s TypeScript model.
- IK solves only 3D position, not a full 6D end-effector pose. It is a local iterative method affected by initial state, singularities, limits, and unreachable targets; convergence is not guaranteed. The maximum-reach shell is only an outer bound.
- Frame rotation limits constrain components of the current Euler representation and are not equivalent to independent single-axis URDF joint limits. Euler singularities and equivalent branches still apply.
- Gimbal Lock is currently a numeric warning rather than a separate axis-alignment animation. SLERP comparison and tool/world motion comparison are also not implemented.
- Collision detection, dynamics, and real robot control are not implemented. Do not send teaching values directly to hardware.

## Publishing and privacy

The site is hosted with Sites and configured through `.openai/hosting.json`; static output is built into `dist/`. Site access and GitHub repository visibility are separate settings, and the GitHub repository is currently public. After downloading the source, use one of the launchers in the repository root to run it locally. Only production build artifacts are published; temporary screenshots, credentials, and local environment files are excluded.
