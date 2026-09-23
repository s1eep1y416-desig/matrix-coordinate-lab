import { useMemo } from 'react';
import { Quaternion, Vector3 } from 'three';
import { forwardKinematics, type JointAngles } from '../math/kinematics';
import { identityPose } from '../math/transform';
import { CoordinateFrame, SceneLabel } from './CoordinateFrame';

function Link({ from, to, color }: { from: Vector3; to: Vector3; color: string }) {
  const { center, orientation, length } = useMemo(() => {
    const delta = to.clone().sub(from);
    return {
      center: from.clone().add(to).multiplyScalar(0.5),
      orientation: new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), delta.clone().normalize()),
      length: delta.length(),
    };
  }, [from, to]);
  return <mesh position={center.toArray()} quaternion={orientation} castShadow>
    <cylinderGeometry args={[0.12, 0.12, length, 18]} />
    <meshStandardMaterial color={color} metalness={0.4} roughness={0.35} />
  </mesh>;
}

export function Robot({ angles, step = 3 }: { angles: JointAngles; step?: number }) {
  const fk = useMemo(() => forwardKinematics(angles), [angles]);
  const colors = ['#d9b75f', '#b9ee72', '#76badf'];
  const frames = [identityPose(), fk.T_base_link1, fk.T_base_link2, fk.T_base_tool];
  const names = ['base', 'link1', 'link2', 'tool'];
  return <group>
    {fk.jointPositions.slice(0, 3).map((point, index) => index < step &&
      <Link key={index} from={point} to={fk.jointPositions[index + 1]} color={colors[index]} />)}
    {frames.slice(0, step + 1).map((pose, index) => <group key={names[index]}>
      <CoordinateFrame pose={pose} label={names[index]} length={index === step ? 0.58 : 0.42} selected={index === step} subtle={index !== step} />
      {index > 0 && <mesh position={pose.position.toArray()}><sphereGeometry args={[0.16, 18, 14]} /><meshStandardMaterial color={colors[index - 1]} metalness={0.4} roughness={0.28} /></mesh>}
    </group>)}
    {step === 3 && <SceneLabel label="end_effector" position={fk.T_base_tool.position.clone().add(new Vector3(0, 0, 0.42)).toArray()} color="#f8e8b1" />}
  </group>;
}
