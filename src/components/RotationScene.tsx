import { Line } from '@react-three/drei';
import { Quaternion, Vector3 } from 'three';
import { compareRotationOrder, evaluateRotationVector, ROTATION_DEMO_VECTOR } from '../math/rotation';
import { makePose } from '../math/transform';
import { useLabStore } from '../stores/labStore';
import { CoordinateFrame, SceneLabel } from './CoordinateFrame';

function RotationComparisonFrame({ position, orientation, label, sense }: {
  position: [number, number, number]; orientation: Quaternion; label: string; sense: 'active' | 'passive';
}) {
  const origin = new Vector3(...position);
  const initialVector = new Vector3(...ROTATION_DEMO_VECTOR);
  const { worldVector } = evaluateRotationVector(orientation, initialVector, sense);
  const vectorTip = origin.clone().add(worldVector);
  const direction = worldVector.clone().normalize();
  const arrowOrientation = new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), direction);
  return <>
    {([['#ff655c', 1, 0, 0], ['#69dd82', 0, 1, 0], ['#6b9cff', 0, 0, 1]] as const).map(([color, x, y, z]) =>
      <Line key={color} points={[origin.toArray(), origin.clone().add(new Vector3(x, y, z).multiplyScalar(0.85)).toArray()]} color={color} lineWidth={1.5} transparent opacity={0.28} />)}
    {sense === 'active' && <Line points={[origin.toArray(), origin.clone().add(initialVector).toArray()]} color="#d9b75f" lineWidth={1.5} transparent opacity={0.28} dashed dashSize={0.05} gapSize={0.06} />}
    <CoordinateFrame pose={makePose(origin, orientation)} label={label} accent="#e8d08a" length={0.85} selected />
    <Line points={[origin.toArray(), vectorTip.toArray()]} color="#d9b75f" lineWidth={2.5} />
    <mesh position={vectorTip.clone().addScaledVector(direction, -0.045).toArray()} quaternion={arrowOrientation}>
      <coneGeometry args={[0.035, 0.09, 12]} /><meshBasicMaterial color="#d9b75f" />
    </mesh>
    <SceneLabel label={sense === 'active' ? "v'" : 'v (World)'} position={vectorTip.clone().add(new Vector3(0, 0, 0.13)).toArray()} color="#e8d08a" fontSize={0.12} />
  </>;
}

export function RotationScene() {
  const { rotationX, rotationY, rotationZ, rotationPair, rotationSense, rotationProgress } = useLabStore();
  const { forward, reverse } = compareRotationOrder({ X: rotationX, Y: rotationY, Z: rotationZ }, rotationPair, rotationProgress);
  return <>
    <RotationComparisonFrame position={[-1.65, 0, 0.3]} orientation={forward} label="A" sense={rotationSense} />
    <RotationComparisonFrame position={[1.65, 0, 0.3]} orientation={reverse} label="B" sense={rotationSense} />
  </>;
}
