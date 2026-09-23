import { useEffect, useRef } from 'react';
import { Canvas, useFrame, useThree, type ThreeEvent } from '@react-three/fiber';
import { Line, OrbitControls } from '@react-three/drei';
import { Group, MOUSE, Plane, Quaternion, Vector3 } from 'three';
import { rotationAbout, rotationProduct } from '../math/rotation';
import { identityPose, makePose } from '../math/transform';
import { useLabStore, worldPoseFor } from '../stores/labStore';
import { CoordinateFrame, SceneLabel } from './CoordinateFrame';
import { Robot } from './Robot';

function PointMarker({ point, onMove }: { point: Vector3; onMove: (point: Vector3) => void }) {
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
      <sphereGeometry args={[0.16, 24, 18]} /><meshStandardMaterial color="#b9ee72" emissive="#58852a" emissiveIntensity={0.38} />
    </mesh>
    <SceneLabel label="Point P" position={[0, 0, 0.25]} color="#d4f6a9" />
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
      length={frame.id === 'world' ? 1.15 : 0.82} subtle={frame.id === 'world'} draggable={frame.id !== 'world'}
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

function AnimatedComparison({ position, target, label }: { position: [number, number, number]; target: Quaternion; label: string }) {
  const group = useRef<Group>(null);
  useFrame((_, delta) => {
    if (group.current) group.current.quaternion.slerp(target, 1 - Math.exp(-delta * 7));
  });
  return <group position={position} ref={group}>
    <CoordinateFrame pose={identityPose()} label={label} length={0.85} />
    <mesh position={[0.48, 0, 0]}><boxGeometry args={[0.95, 0.12, 0.16]} /><meshStandardMaterial color="#d9b75f" metalness={0.45} roughness={0.3} /></mesh>
  </group>;
}

function RotationScene() {
  const { rotationX, rotationY, rotationSense } = useLabStore();
  const Rx = rotationAbout('X', rotationX), Ry = rotationAbout('Y', rotationY);
  const RxRy = rotationProduct(Rx, Ry), RyRx = rotationProduct(Ry, Rx);
  const vector = new Vector3(1, 0.35, 0.4);
  const worldVector = rotationSense === 'active' ? vector.clone().applyQuaternion(RxRy) : vector;
  return <>
    <AnimatedComparison position={[-1.75, 0.55, 0.25]} target={RxRy} label="Rx · Ry" />
    <AnimatedComparison position={[1.75, 0.55, 0.25]} target={RyRx} label="Ry · Rx" />
    <CoordinateFrame pose={identityPose()} label="World" length={0.65} subtle />
    <Line points={[[0, -1.25, 0], worldVector.clone().add(new Vector3(0, -1.25, 0)).toArray()]} color="#b9ee72" lineWidth={4} />
    <mesh position={worldVector.clone().add(new Vector3(0, -1.25, 0)).toArray()}><sphereGeometry args={[0.09, 16, 12]} /><meshBasicMaterial color="#b9ee72" /></mesh>
    <SceneLabel label={rotationSense === 'active' ? 'Active vector' : 'Passive frame'} position={[0, -1.25, 0.7]} color="#d4f6a9" />
    {rotationSense === 'passive' && <CoordinateFrame pose={makePose(new Vector3(0, -1.25, 0), RxRy)} label="观察坐标系" length={0.55} />}
  </>;
}

function WorldGrid() {
  return <>
    <gridHelper args={[14, 28, '#4c4939', '#252720']} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, -0.005]} />
    <ambientLight intensity={1.15} /><directionalLight position={[4, -5, 8]} intensity={2.3} />
  </>;
}

function SceneCamera({ mode }: { mode: 'frames' | 'rotation' | 'chain' | 'fk' }) {
  const { camera, controls, invalidate } = useThree();
  useEffect(() => {
    const position: [number, number, number] = mode === 'fk' ? [2.5, -3.4, 2.6] : mode === 'rotation' ? [3.2, -5, 3.7] : [3.1, -4.2, 3.0];
    const target: [number, number, number] = mode === 'fk' ? [1.35, 0.55, 0.25] : mode === 'rotation' ? [0, 0, 0.45] : [0.9, 0.5, 0.45];
    camera.position.set(...position);
    const orbit = controls as { target?: Vector3; update?: () => void } | null;
    if (orbit?.target) { orbit.target.set(...target); orbit.update?.(); }
    else camera.lookAt(...target);
    invalidate();
  }, [camera, controls, invalidate, mode]);
  return null;
}

export function Scene() {
  const mode = useLabStore((state) => state.mode);
  const robotAngles = useLabStore((state) => state.robotAngles);
  const fkStep = useLabStore((state) => state.fkStep);
  return <div className="scene-shell" aria-label="三维机器人学场景">
    <Canvas camera={{ position: [3.1, -4.2, 3.0], up: [0, 0, 1], fov: 42, near: 0.1, far: 100 }} dpr={[1, 2]} fallback={<div className="webgl-fallback">此浏览器无法启动 3D 场景，数值和矩阵仍可使用。</div>}>
      <color attach="background" args={['#0b0c09']} />
      <WorldGrid />
      {mode === 'fk' ? <Robot angles={robotAngles} step={fkStep} /> : mode === 'rotation' ? <RotationScene /> : <FrameScene />}
      <OrbitControls makeDefault target={[0.9, 0.5, 0.45]} enablePan={false} minDistance={2.5} maxDistance={18} mouseButtons={{ LEFT: MOUSE.PAN, MIDDLE: MOUSE.ROTATE, RIGHT: MOUSE.DOLLY }} />
      <SceneCamera mode={mode} />
    </Canvas>
    <div className="scene-overlay"><span className="scene-live">● 实时同步</span><span>{mode === 'frames' || mode === 'chain' ? '中键转视角 · 左键拖动原点 / 彩色轴端 / 点 P · 滚轮缩放' : '中键转视角 · 右侧调节角度 · 滚轮缩放'}</span></div>
  </div>;
}
