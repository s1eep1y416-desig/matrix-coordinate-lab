import { Vector3 } from 'three';
import { forwardKinematics, wrapDegrees, type JointAngles } from './kinematics';

export interface JointTrajectorySample {
  progress: number;
  angles: JointAngles;
  position: Vector3;
}

const clamp01 = (value: number) => Math.min(1, Math.max(0, value));

/** Quintic minimum-jerk time law with zero endpoint velocity and acceleration. */
export function minimumJerkBlend(progress: number): number {
  const u = clamp01(progress);
  return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5;
}

/** Interpolate the three joint angles using the shared minimum-jerk time law. */
export function trajectoryAnglesAt(start: JointAngles, goal: JointAngles, progress: number): JointAngles {
  const blend = minimumJerkBlend(progress);
  return start.map((angle, index) => {
    const delta = index === 0 ? wrapDegrees(goal[index] - angle) : goal[index] - angle;
    const interpolated = angle + delta * blend;
    return index === 0 ? wrapDegrees(interpolated) : interpolated;
  }) as JointAngles;
}

/** Sample a joint-space trajectory and its actual end-effector path through FK. */
export function planJointTrajectory(start: JointAngles, goal: JointAngles, sampleCount = 121): JointTrajectorySample[] {
  const count = Math.max(2, Math.round(sampleCount));
  return Array.from({ length: count }, (_, index) => {
    const progress = index / (count - 1);
    const angles = trajectoryAnglesAt(start, goal, progress);
    return {
      progress,
      angles,
      position: forwardKinematics(angles).T_base_tool.position.clone(),
    };
  });
}
