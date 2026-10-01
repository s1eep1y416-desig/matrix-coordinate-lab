import { Vector3 } from 'three';
import { rotationAbout } from './rotation';
import { composePoses, identityPose, makePose, type Pose } from './transform';

export const LINK_LENGTHS = [1.25, 1.0, 0.8] as const;
export type JointAngles = [number, number, number];

/** Normalize a periodic revolute-joint angle to [-180°, 180°). */
export function wrapDegrees(angle: number): number {
  if (!Number.isFinite(angle)) return angle;
  return ((angle + 180) % 360 + 360) % 360 - 180;
}

export interface FKResult {
  T_base_link1: Pose;
  T_link1_link2: Pose;
  T_link2_link3: Pose;
  T_base_link2: Pose;
  T_base_tool: Pose;
  jointPositions: Vector3[];
}

/** Each revolute joint rotates its outgoing link, which then extends along local +X. */
export function forwardKinematics(q: JointAngles): FKResult {
  const T_base_link1 = composePoses(makePose(new Vector3(), rotationAbout('Z', q[0])), makePose(new Vector3(LINK_LENGTHS[0], 0, 0)));
  const T_link1_link2 = composePoses(makePose(new Vector3(), rotationAbout('Y', q[1])), makePose(new Vector3(LINK_LENGTHS[1], 0, 0)));
  const T_link2_link3 = composePoses(makePose(new Vector3(), rotationAbout('Y', q[2])), makePose(new Vector3(LINK_LENGTHS[2], 0, 0)));
  const T_base_link2 = composePoses(T_base_link1, T_link1_link2);
  const T_base_tool = composePoses(T_base_link2, T_link2_link3);
  return {
    T_base_link1, T_link1_link2, T_link2_link3, T_base_link2, T_base_tool,
    jointPositions: [identityPose().position, T_base_link1.position, T_base_link2.position, T_base_tool.position].map((point) => point.clone()),
  };
}
