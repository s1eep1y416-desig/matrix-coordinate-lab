import { useMemo } from 'react';
import { Line } from '@react-three/drei';
import { forwardKinematics, type JointAngles } from '../math/kinematics';
import { identityPose } from '../math/transform';
import { CoordinateFrame, SceneLabel } from './CoordinateFrame';

export function Robot({ angles, step = 3 }: { angles: JointAngles; step?: number }) {
  const fk = useMemo(() => forwardKinematics(angles), [angles]);
  const colors = ['#d9b75f', '#d9b75f', '#d9b75f'];
  const frames = [identityPose(), fk.T_base_link1, fk.T_base_link2, fk.T_base_tool];
  const names = ['base', 'link1', 'link2', 'tool'];
  return <group>
    {fk.jointPositions.slice(0, 3).map((point, index) => index < step &&
      <Line key={index} points={[point.toArray(), fk.jointPositions[index + 1].toArray()]} color={colors[index]} lineWidth={3} />)}
    {frames.slice(0, step + 1).map((pose, index) => <group key={names[index]}>
      <CoordinateFrame pose={pose} label={names[index]} accent={index === step ? '#d9b75f' : '#bfc5c1'} length={index === step ? 0.58 : 0.42} selected={index === step} subtle={index !== step} />
      {index > 0 && <mesh position={pose.position.toArray()}><sphereGeometry args={[0.04, 14, 10]} /><meshBasicMaterial color={colors[index - 1]} /></mesh>}
    </group>)}
    {step === 3 && <SceneLabel label="end_effector" position={fk.T_base_tool.position.clone().addScaledVector(fk.T_base_tool.position.clone().normalize(), 0.25).toArray()} color="#f8e8b1" fontSize={0.14} />}
  </group>;
}
