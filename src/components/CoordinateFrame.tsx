import { useRef } from 'react';
import { Billboard, Line, Text } from '@react-three/drei';
import type { ThreeEvent } from '@react-three/fiber';
import { Plane, Quaternion, Vector3 } from 'three';
import type { Pose } from '../math/transform';
import labelFont from 'katex/dist/fonts/KaTeX_Main-Regular.ttf?url';

const axes = [
  { color: '#ff655c', label: 'X', point: [1, 0, 0] as [number, number, number] },
  { color: '#69dd82', label: 'Y', point: [0, 1, 0] as [number, number, number] },
  { color: '#6b9cff', label: 'Z', point: [0, 0, 1] as [number, number, number] },
];

interface CoordinateFrameProps {
  pose: Pose;
  label: string;
  length?: number;
  selected?: boolean;
  draggable?: boolean;
  subtle?: boolean;
  onSelect?: () => void;
  onMoveWorld?: (point: Vector3) => void;
  onRotateWorld?: (quaternion: Quaternion) => void;
}

export function SceneLabel({ label, position, color = '#e6eadf', fontSize = 0.17 }: { label: string; position: [number, number, number]; color?: string; fontSize?: number }) {
  return <Billboard position={position} follow>
    <mesh position={[0, 0, -0.015]}><planeGeometry args={[Math.max(0.34, label.length * fontSize * 0.58 + 0.15), fontSize * 1.55]} /><meshBasicMaterial color="#10130f" transparent opacity={0.9} depthTest={false} /></mesh>
    <Text font={labelFont} fontSize={fontSize} color={color} anchorX="center" anchorY="middle" material-depthTest={false}>{label}</Text>
  </Billboard>;
}

export function CoordinateFrame({ pose, label, length = 0.85, selected, draggable, subtle, onSelect, onMoveWorld, onRotateWorld }: CoordinateFrameProps) {
  const drag = useRef<{ pointerId: number; plane: Plane; startHit: Vector3; startOrigin: Vector3 } | null>(null);
  const axisDrag = useRef<{ pointerId: number; plane: Plane; origin: Vector3; axis: Vector3; quaternion: Quaternion } | null>(null);
  const pointerDown = (event: ThreeEvent<PointerEvent>) => {
    if (!draggable || event.button !== 0) return;
    event.stopPropagation(); onSelect?.();
    const normal = new Vector3(); event.camera.getWorldDirection(normal);
    const plane = new Plane().setFromNormalAndCoplanarPoint(normal, pose.position);
    const startHit = event.ray.intersectPlane(plane, new Vector3());
    if (!startHit) return;
    drag.current = { pointerId: event.pointerId, plane, startHit, startOrigin: pose.position.clone() };
    (event.target as Element).setPointerCapture(event.pointerId);
  };
  const pointerMove = (event: ThreeEvent<PointerEvent>) => {
    const current = drag.current;
    if (!current || current.pointerId !== event.pointerId) return;
    event.stopPropagation();
    const hit = event.ray.intersectPlane(current.plane, new Vector3());
    if (hit) onMoveWorld?.(current.startOrigin.clone().add(hit.sub(current.startHit)));
  };
  const pointerEnd = (event: ThreeEvent<PointerEvent>) => {
    if (drag.current?.pointerId !== event.pointerId) return;
    event.stopPropagation(); drag.current = null;
    (event.target as Element).releasePointerCapture(event.pointerId);
  };
  const axisDown = (event: ThreeEvent<PointerEvent>, axis: [number, number, number]) => {
    if (!draggable || event.button !== 0) return;
    event.stopPropagation(); onSelect?.();
    const startAxis = new Vector3(...axis).applyQuaternion(pose.quaternion).normalize();
    const normal = new Vector3(); event.camera.getWorldDirection(normal);
    const plane = new Plane().setFromNormalAndCoplanarPoint(normal, pose.position.clone().addScaledVector(startAxis, length));
    axisDrag.current = { pointerId: event.pointerId, plane, origin: pose.position.clone(), axis: startAxis, quaternion: pose.quaternion.clone() };
    (event.target as Element).setPointerCapture(event.pointerId);
  };
  const axisMove = (event: ThreeEvent<PointerEvent>) => {
    const current = axisDrag.current;
    if (!current || current.pointerId !== event.pointerId) return;
    event.stopPropagation();
    const hit = event.ray.intersectPlane(current.plane, new Vector3());
    if (!hit || hit.distanceToSquared(current.origin) < 0.01) return;
    const desired = hit.sub(current.origin).normalize();
    const delta = new Quaternion().setFromUnitVectors(current.axis, desired);
    onRotateWorld?.(delta.multiply(current.quaternion).normalize());
  };
  const axisEnd = (event: ThreeEvent<PointerEvent>) => {
    if (axisDrag.current?.pointerId !== event.pointerId) return;
    event.stopPropagation(); axisDrag.current = null;
    (event.target as Element).releasePointerCapture(event.pointerId);
  };
  return <group position={pose.position.toArray()} quaternion={pose.quaternion.clone()}>
    {axes.map((axis) => <group key={axis.label}>
      <Line points={[[0, 0, 0], axis.point.map((number) => number * length) as [number, number, number]]} color={axis.color} lineWidth={subtle ? 1.6 : 3} transparent opacity={subtle ? 0.65 : 1} />
      <mesh position={axis.point.map((number) => number * length) as [number, number, number]}
        onPointerDown={(event) => axisDown(event, axis.point)} onPointerMove={axisMove} onPointerUp={axisEnd} onPointerCancel={axisEnd}>
        <sphereGeometry args={[draggable ? 0.095 : subtle ? 0.035 : 0.05, 16, 12]} /><meshBasicMaterial color={axis.color} />
      </mesh>
      {!subtle && <SceneLabel label={axis.label} position={axis.point.map((number) => number * (length + 0.13)) as [number, number, number]} color={axis.color} fontSize={0.18} />}
    </group>)}
    <mesh onPointerDown={pointerDown} onPointerMove={pointerMove} onPointerUp={pointerEnd} onPointerCancel={pointerEnd} onClick={(event) => { event.stopPropagation(); onSelect?.(); }}>
      <sphereGeometry args={[selected ? 0.17 : 0.14, 22, 16]} />
      <meshStandardMaterial color={selected ? '#d9b75f' : subtle ? '#a7afb2' : '#d7dde2'} emissive={selected ? '#8c6c20' : '#131715'} emissiveIntensity={selected ? 0.25 : 0.08} />
    </mesh>
    <SceneLabel label={label} position={[0, 0, -0.28]} color={selected ? '#f8e8b1' : '#e6eadf'} />
  </group>;
}
