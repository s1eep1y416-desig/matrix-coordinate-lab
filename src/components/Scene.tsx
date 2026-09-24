import { useEffect, useRef } from 'react';
import { Canvas, useThree, type ThreeEvent } from '@react-three/fiber';
import { Line, OrbitControls } from '@react-three/drei';
import { MOUSE, Plane, Quaternion, Vector3 } from 'three';
import { compareRotationOrder } from '../math/rotation';
import { forwardKinematics } from '../math/kinematics';
import { makePose } from '../math/transform';
import { useLabStore, worldPoseFor } from '../stores/labStore';
import { CoordinateFrame, SceneLabel } from './CoordinateFrame';
import { Robot } from './Robot';

function PointMarker({ point, onMove, label = 'Point P', color = '#b9ee72' }: { point: Vector3; onMove: (point: Vector3) => void; label?: string; color?: string }) {
  const drag = useRef<{ pointerId: number; plane: Plane; startHit: Vector3; start: Vector3 } | null>(null);
  const down = (event: ThreeEvent<PointerEvent>) => {
    if (event.button !== 0) return;
    event.stopPropagation();
    const normal = new Vector3(); event.camera.getWorldDirection(normal);
    const plane = new Plane().setFromNormalAndCoplanarPoint(normal, point);
    const startHit = event.ray.intersectPlane(plane, new Vector3());
    if (!startHit) return;
    drag.current = { pointerId: event.pointerId, plane, startHit, start: point.clone() };
    (event.target as Element).setPointerCapture(event.pointerId);
  };
  const move = (event: ThreeEvent<PointerEvent>) => {
    const current = drag.current;
    if (!current || current.pointerId !== event.pointerId) return;
    event.stopPropagation();
    const hit = event.ray.intersectPlane(current.plane, new Vector3());
    if (hit) onMove(current.start.clone().add(hit.sub(current.startHit)));
  };
  const up = (event: ThreeEvent<PointerEvent>) => {
    if (drag.current?.pointerId !== event.pointerId) return;
    event.stopPropagation(); drag.current = null;
    (event.target as Element).releasePointerCapture(event.pointerId);
  };
  return <group position={point.toArray()}>
    <mesh onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}>
      <sphereGeometry args={[0.16, 16, 12]} /><meshBasicMaterial transparent opacity={0} depthWrite={false} />
    </mesh>
    <mesh><sphereGeometry args={[0.055, 16, 12]} /><meshBasicMaterial color={color} /></mesh>
    <SceneLabel label={label} position={[0, 0, 0.17]} color="#d4f6a9" fontSize={0.14} />
  </group>;
}

function FrameScene() {
  const { frames, selectedFrameId, pointWorld, selectFrame, moveFrameWorld, setPointWorld, setFrameQuaternion } = useLabStore();
  const visibleFrames = frames.filter((frame) => frame.visible);
  const sourceWorld = worldPoseFor(frames, selectedFrameId);
  return <>
    {visibleFrames.filter((frame) => frame.id !== 'world').map((frame) => {
      const parent = worldPoseFor(frames, frame.parentId ?? 'world').position;
      const child = worldPoseFor(frames, frame.id).position;
      return <Line key={`edge-${frame.id}`} points={[parent.toArray(), child.toArray()]} color="#d9b75f" opacity={0.4} transparent dashed dashSize={0.07} gapSize={0.07} lineWidth={1.5} />;
    })}
    {visibleFrames.map((frame) => <CoordinateFrame
      key={frame.id} pose={worldPoseFor(frames, frame.id)} label={frame.name} selected={frame.id === selectedFrameId}
      accent={frame.id === 'world' ? '#d7dbe3' : ['#b18cff', '#ffb55d', '#5fd4cf', '#f27caa', '#d2ee62'][(Math.max(0, frame.id.charCodeAt(0) - 65)) % 5]}
      length={frame.id === 'world' ? 1.6 : 1.02} subtle={frame.id === 'world'} draggable={frame.id !== 'world'}
      onSelect={() => selectFrame(frame.id)} onMoveWorld={(point) => moveFrameWorld(frame.id, point)}
      onRotateWorld={(quaternion) => {
        const parent = worldPoseFor(useLabStore.getState().frames, frame.parentId ?? 'world');
        setFrameQuaternion(frame.id, parent.quaternion.clone().invert().multiply(quaternion).normalize());
      }}
    />)}
    <Line points={[sourceWorld.position.toArray(), pointWorld.toArray()]} color="#b9ee72" lineWidth={2.5} />
    <PointMarker point={pointWorld} onMove={setPointWorld} />
  </>;
}

function RotationComparisonFrame({ position, orientation, label }: { position: [number, number, number]; orientation: Quaternion; label: string }) {
  const origin = new Vector3(...position);
  const vectorTip = origin.clone().add(new Vector3(0.62, 0.28, 0.2).applyQuaternion(orientation));
  return <>
    {([['#ff655c', 1, 0, 0], ['#69dd82', 0, 1, 0], ['#6b9cff', 0, 0, 1]] as const).map(([color, x, y, z]) =>
      <Line key={color} points={[origin.toArray(), origin.clone().add(new Vector3(x, y, z).multiplyScalar(0.8)).toArray()]} color={color} lineWidth={1.5} transparent opacity={0.3} />)}
    <CoordinateFrame pose={makePose(origin, orientation)} label={label} accent="#e8d08a" length={0.85} selected />
    <Line points={[origin.toArray(), vectorTip.toArray()]} color="#d9b75f" lineWidth={2.5} />
    <mesh position={vectorTip.toArray()}><sphereGeometry args={[0.035, 10, 8]} /><meshBasicMaterial color="#d9b75f" /></mesh>
  </>;
}

function RotationScene() {
  const { rotationX, rotationY, rotationZ, rotationPair, rotationSense } = useLabStore();
  const { firstAxis, secondAxis, forward, reverse } = compareRotationOrder({ X: rotationX, Y: rotationY, Z: rotationZ }, rotationPair);
  const vector = new Vector3(1, 0.35, 0.4);
  const worldVector = rotationSense === 'active' ? vector.clone().applyQuaternion(forward) : vector;
  return <>
    <RotationComparisonFrame position={[-1.75, 0.55, 0.25]} orientation={forward} label={`R${firstAxis.toLowerCase()}R${secondAxis.toLowerCase()}`} />
    <RotationComparisonFrame position={[1.75, 0.55, 0.25]} orientation={reverse} label={`R${secondAxis.toLowerCase()}R${firstAxis.toLowerCase()}`} />
    <CoordinateFrame pose={makePose(new Vector3(), new Quaternion())} label="World" length={0.65} subtle />
    <Line points={[[0, -1.25, 0], worldVector.clone().add(new Vector3(0, -1.25, 0)).toArray()]} color="#b9ee72" lineWidth={4} />
    <mesh position={worldVector.clone().add(new Vector3(0, -1.25, 0)).toArray()}><sphereGeometry args={[0.045, 16, 12]} /><meshBasicMaterial color="#b9ee72" /></mesh>
    <SceneLabel label={rotationSense === 'active' ? 'Active vector' : 'Passive frame'} position={[0, -1.25, 0.7]} color="#d4f6a9" />
    {rotationSense === 'passive' && <CoordinateFrame pose={makePose(new Vector3(0, -1.25, 0), forward)} label="观察坐标系" length={0.55} />}
  </>;
}

function IKScene() {
  const { robotAngles, ikTarget, setIkTarget } = useLabStore();
  const end = forwardKinematics(robotAngles).T_base_tool.position;
  return <>
    <Robot angles={robotAngles} step={3} />
    <Line points={[end.toArray(), ikTarget.toArray()]} color="#b9ee72" lineWidth={2.5} dashed dashSize={.08} gapSize={.05} />
    <PointMarker point={ikTarget} onMove={setIkTarget} label="IK Target" />
    <mesh position={[0, 0, 0]}><sphereGeometry args={[3.05, 32, 20]} /><meshBasicMaterial color="#89a66b" wireframe transparent opacity={.035} depthWrite={false} /></mesh>
  </>;
}

function WorldGrid() {
  return <>
    <gridHelper args={[14, 28, '#4c4939', '#252720']} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, -0.005]} />
    <ambientLight intensity={1.15} /><directionalLight position={[4, -5, 8]} intensity={2.3} />
  </>;
}

function SceneCamera({ mode, resetKey }: { mode: 'frames' | 'rotation' | 'chain' | 'fk' | 'ik' | 'pinocchio'; resetKey: number }) {
  const { camera, controls, invalidate } = useThree();
  useEffect(() => {
    const robotMode = mode === 'fk' || mode === 'ik' || mode === 'pinocchio';
    const position: [number, number, number] = robotMode ? [2.5, -3.4, 2.6] : mode === 'rotation' ? [3.2, -5, 3.7] : [3.1, -4.2, 3.0];
    const target: [number, number, number] = robotMode ? [1.35, 0.55, 0.25] : mode === 'rotation' ? [0, 0, 0.45] : [0.9, 0.5, 0.45];
    camera.position.set(...position);
    const orbit = controls as { target?: Vector3; update?: () => void } | null;
    if (orbit?.target) { orbit.target.set(...target); orbit.update?.(); }
    else camera.lookAt(...target);
    invalidate();
  }, [camera, controls, invalidate, mode, resetKey]);
  return null;
}

export function Scene() {
  const mode = useLabStore((state) => state.mode);
  const robotAngles = useLabStore((state) => state.robotAngles);
  const fkStep = useLabStore((state) => state.fkStep);
  const pinStep = useLabStore((state) => state.pinStep);
  const cameraResetKey = useLabStore((state) => state.cameraResetKey);
  const resetCamera = useLabStore((state) => state.resetCamera);
  const sourceId = useLabStore((state) => state.sourceId);
  const targetId = useLabStore((state) => state.targetId);
  const frames = useLabStore((state) => state.frames);
  const frameName = (id: string) => frames.find((frame) => frame.id === id)?.name ?? id;
  return <div className="scene-shell" aria-label="三维机器人学场景">
    <Canvas camera={{ position: [3.1, -4.2, 3.0], up: [0, 0, 1], fov: 42, near: 0.1, far: 100 }} dpr={[1, 2]} fallback={<div className="webgl-fallback">此浏览器无法启动 3D 场景，数值和矩阵仍可使用。</div>}>
      <color attach="background" args={['#0b0c09']} />
      <WorldGrid />
      {mode === 'fk' ? <Robot angles={robotAngles} step={fkStep} /> : mode === 'ik' ? <IKScene /> : mode === 'pinocchio' ? <Robot angles={robotAngles} step={pinStep < 3 ? 0 : pinStep < 5 ? 2 : 3} /> : mode === 'rotation' ? <RotationScene /> : <FrameScene />}
      <OrbitControls makeDefault target={[0.9, 0.5, 0.45]} enablePan={false} minDistance={2.5} maxDistance={18} mouseButtons={{ LEFT: MOUSE.PAN, MIDDLE: MOUSE.ROTATE, RIGHT: MOUSE.DOLLY }} />
      <SceneCamera mode={mode} resetKey={cameraResetKey} />
    </Canvas>
    {(mode === 'frames' || mode === 'chain') && <>
      <div className="scene-axis-legend" aria-label="坐标轴颜色：X 红、Y 绿、Z 蓝"><span className="x">X</span><span className="y">Y</span><span className="z">Z</span></div>
      <div className="scene-route"><span>源</span><strong>{frameName(sourceId)}</strong><b>→</b><span>目标</span><strong>{frameName(targetId)}</strong></div>
    </>}
    <button className="scene-reset-view" onClick={resetCamera}>重置视角</button>
    <div className="scene-overlay"><span className="scene-live">● 实时同步</span><span>{mode === 'frames' || mode === 'chain' ? '中键转视角 · 左键拖动原点 / 彩色轴端 / 点 P · 滚轮缩放' : mode === 'ik' ? '中键转视角 · 左键拖动 Target · 滚轮缩放' : '中键转视角 · 右侧调节参数 · 滚轮缩放'}</span></div>
  </div>;
}
