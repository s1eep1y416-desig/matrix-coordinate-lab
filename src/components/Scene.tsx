import { useEffect, useRef, useState } from 'react';
import { Canvas, useThree, type ThreeEvent } from '@react-three/fiber';
import { Line, OrbitControls } from '@react-three/drei';
import { MOUSE, Plane, Vector3 } from 'three';
import { forwardKinematics } from '../math/kinematics';
import { framePathIds, useLabStore, worldPoseFor } from '../stores/labStore';
import { localize, useLocaleStore } from '../stores/localeStore';
import { CoordinateFrame, SceneLabel } from './CoordinateFrame';
import { Robot } from './Robot';
import { RotationScene } from './RotationScene';

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
  const { frames, selectedFrameId, pointWorld, chainStep, selectFrame, moveFrameWorld, setPointWorld, setFrameQuaternion } = useLabStore();
  const visibleFrames = frames.filter((frame) => frame.visible);
  const sourceWorld = worldPoseFor(frames, selectedFrameId);
  const chainPath = framePathIds(frames, selectedFrameId);
  const effectiveStep = Math.min(chainStep, Math.max(0, chainPath.length - 1));
  const highlightedId = chainPath[effectiveStep];
  return <>
    {visibleFrames.filter((frame) => frame.id !== 'world').map((frame) => {
      const parent = worldPoseFor(frames, frame.parentId ?? 'world').position;
      const child = worldPoseFor(frames, frame.id).position;
      const pathIndex = chainPath.indexOf(frame.id);
      const complete = pathIndex > 0 && pathIndex <= effectiveStep;
      const pending = pathIndex > effectiveStep;
      return <Line key={`edge-${frame.id}`} points={[parent.toArray(), child.toArray()]} color={complete ? '#f0cf72' : pending ? '#806e42' : '#55594d'} opacity={complete ? 0.95 : pending ? 0.35 : 0.22} transparent dashed={!complete} dashSize={0.07} gapSize={0.07} lineWidth={complete ? 3 : 1.5} />;
    })}
    {visibleFrames.map((frame) => <CoordinateFrame
      key={frame.id} pose={worldPoseFor(frames, frame.id)} label={frame.name} selected={frame.id === selectedFrameId} highlighted={frame.id === highlightedId}
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

function IKScene() {
  const { robotAngles, ikTarget, setIkTarget, ikTrajectory, ikTrajectoryProgress } = useLabStore();
  const end = forwardKinematics(robotAngles).T_base_tool.position;
  const completedIndex = Math.min(ikTrajectory.length - 1, Math.floor(ikTrajectoryProgress * Math.max(0, ikTrajectory.length - 1)));
  const travelled = ikTrajectory.slice(0, completedIndex + 1).map((sample) => sample.position);
  if (travelled.length && travelled.at(-1)!.distanceTo(end) > 1e-6) travelled.push(end);
  const trailStart = Math.max(0, travelled.length - 38);
  const trail = travelled.slice(trailStart);
  return <>
    <Robot angles={robotAngles} step={3} />
    {ikTrajectory.length > 1 && <>
      <Line points={ikTrajectory.map((sample) => sample.position)} color="#ffcc52" lineWidth={8} transparent opacity={0.1} depthWrite={false} />
      <Line points={ikTrajectory.map((sample) => sample.position)} color="#ffd96e" lineWidth={2.2} transparent opacity={0.72} depthWrite={false} />
    </>}
    {trail.slice(1).map((point, index) => {
      const strength = (index + 1) / Math.max(1, trail.length - 1);
      return <group key={`${trailStart}-${index}`}>
        <Line points={[trail[index], point]} color="#ffc83d" lineWidth={7 + strength * 5} transparent opacity={0.025 + strength * 0.12} depthWrite={false} />
        <Line points={[trail[index], point]} color="#fff0a6" lineWidth={1.4 + strength * 3.1} transparent opacity={0.08 + strength * 0.88} depthWrite={false} />
      </group>;
    })}
    {ikTrajectory.length > 1 && <group position={end.toArray()}>
      <mesh><sphereGeometry args={[0.095, 18, 12]} /><meshBasicMaterial color="#ffd76e" transparent opacity={0.2} depthWrite={false} /></mesh>
      <mesh><sphereGeometry args={[0.032, 16, 10]} /><meshBasicMaterial color="#fff1b2" /></mesh>
    </group>}
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

function SceneCamera({ mode, resetKey }: { mode: 'frames' | 'rotation' | 'fk' | 'ik' | 'pinocchio'; resetKey: number }) {
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
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const mode = useLabStore((state) => state.mode);
  const robotAngles = useLabStore((state) => state.robotAngles);
  const fkStep = useLabStore((state) => state.fkStep);
  const pinStep = useLabStore((state) => state.pinStep);
  const ikTrajectoryPlaying = useLabStore((state) => state.ikTrajectoryPlaying);
  const cameraResetKey = useLabStore((state) => state.cameraResetKey);
  const resetCamera = useLabStore((state) => state.resetCamera);
  const sourceId = useLabStore((state) => state.sourceId);
  const targetId = useLabStore((state) => state.targetId);
  const frames = useLabStore((state) => state.frames);
  const sceneShellRef = useRef<HTMLDivElement>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const frameName = (id: string) => frames.find((frame) => frame.id === id)?.name ?? id;
  useEffect(() => {
    if (mode !== 'ik' || !ikTrajectoryPlaying) return;
    let animationFrame = 0;
    let previous = performance.now();
    const advance = (now: number) => {
      const state = useLabStore.getState();
      state.advanceIKTrajectory(Math.min((now - previous) / 1000, 0.1));
      previous = now;
      if (useLabStore.getState().ikTrajectoryPlaying) animationFrame = requestAnimationFrame(advance);
    };
    animationFrame = requestAnimationFrame(advance);
    return () => cancelAnimationFrame(animationFrame);
  }, [ikTrajectoryPlaying, mode]);
  useEffect(() => {
    const updateFullscreen = () => setIsFullscreen(document.fullscreenElement === sceneShellRef.current);
    document.addEventListener('fullscreenchange', updateFullscreen);
    return () => document.removeEventListener('fullscreenchange', updateFullscreen);
  }, []);
  const toggleFullscreen = async () => {
    if (document.fullscreenElement) await document.exitFullscreen();
    else await sceneShellRef.current?.requestFullscreen();
  };
  const preventMiddleBrowserGesture = (event: React.MouseEvent<HTMLDivElement>) => {
    if (event.button === 1) event.preventDefault();
  };
  return <div ref={sceneShellRef} className="scene-shell" aria-label={l('三维机器人学场景', '3D robotics scene')} onMouseDownCapture={preventMiddleBrowserGesture} onAuxClick={preventMiddleBrowserGesture}>
    <Canvas camera={{ position: [3.1, -4.2, 3.0], up: [0, 0, 1], fov: 42, near: 0.1, far: 100 }} dpr={[1, 2]} fallback={<div className="webgl-fallback">{l('此浏览器无法启动 3D 场景，数值和矩阵仍可使用。', 'This browser cannot start the 3D scene. Numeric and matrix tools remain available.')}</div>}>
      <color attach="background" args={['#0b0c09']} />
      <WorldGrid />
      {mode === 'fk' ? <Robot angles={robotAngles} step={fkStep} /> : mode === 'ik' ? <IKScene /> : mode === 'pinocchio' ? <Robot angles={robotAngles} step={pinStep < 3 ? 0 : pinStep < 5 ? 2 : 3} /> : mode === 'rotation' ? <RotationScene /> : <FrameScene />}
      <OrbitControls makeDefault target={[0.9, 0.5, 0.45]} enablePan={false} minDistance={2.5} maxDistance={18} mouseButtons={{ LEFT: MOUSE.PAN, MIDDLE: MOUSE.ROTATE, RIGHT: MOUSE.DOLLY }} />
      <SceneCamera mode={mode} resetKey={cameraResetKey} />
    </Canvas>
    {mode === 'frames' && <>
      <div className="scene-axis-legend" aria-label={l('坐标轴颜色：X 红、Y 绿、Z 蓝', 'Axis colors: X red, Y green, Z blue')}><span className="x">X</span><span className="y">Y</span><span className="z">Z</span></div>
      <div className="scene-route"><span>{l('源', 'Source')}</span><strong>{frameName(sourceId)}</strong><b>→</b><span>{l('目标', 'Target')}</span><strong>{frameName(targetId)}</strong></div>
    </>}
    <div className="scene-view-actions">
      <button className="scene-reset-view" onClick={resetCamera}>{l('重置视角', 'Reset view')}</button>
      <button className="scene-fullscreen" onClick={toggleFullscreen}>{isFullscreen ? l('退出全屏', 'Exit fullscreen') : l('全屏', 'Fullscreen')}</button>
    </div>
    <div className="scene-overlay"><span className="scene-live">● {l('实时同步', 'Live sync')}</span><span>{mode === 'frames' ? l('中键转视角 · 左键拖动原点 / 彩色轴端 / 点 P · 滚轮缩放', 'Middle drag: orbit · Left drag: origin / axis tip / Point P · Wheel: zoom') : mode === 'ik' ? l('中键转视角 · 左键拖动 Target · 滚轮缩放', 'Middle drag: orbit · Left drag: Target · Wheel: zoom') : l('中键转视角 · 右侧调节参数 · 滚轮缩放', 'Middle drag: orbit · Adjust parameters on the right · Wheel: zoom')}</span></div>
  </div>;
}
