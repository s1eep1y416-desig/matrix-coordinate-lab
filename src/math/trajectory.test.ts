import { describe, expect, it } from 'vitest';
import { forwardKinematics, type JointAngles } from './kinematics';
import { minimumJerkBlend, planJointTrajectory, trajectoryAnglesAt } from './trajectory';

describe('joint-space trajectory planning', () => {
  it('uses a clamped minimum-jerk time law with exact endpoints', () => {
    expect(minimumJerkBlend(-1)).toBe(0);
    expect(minimumJerkBlend(0)).toBe(0);
    expect(minimumJerkBlend(0.5)).toBeCloseTo(0.5, 12);
    expect(minimumJerkBlend(1)).toBe(1);
    expect(minimumJerkBlend(2)).toBe(1);
  });

  it('interpolates joint angles without mutating the endpoints', () => {
    const start: JointAngles = [10, -20, 30];
    const goal: JointAngles = [70, 40, -10];
    expect(trajectoryAnglesAt(start, goal, 0)).toEqual(start);
    expect(trajectoryAnglesAt(start, goal, 1)).toEqual(goal);
    expect(start).toEqual([10, -20, 30]);
    expect(goal).toEqual([70, 40, -10]);
  });

  it('stores FK positions that match every sampled joint configuration', () => {
    const start: JointAngles = [15, -25, 35];
    const goal: JointAngles = [-45, 30, 60];
    const samples = planJointTrajectory(start, goal, 17);
    expect(samples).toHaveLength(17);
    expect(samples[0].angles).toEqual(start);
    expect(samples.at(-1)?.angles).toEqual(goal);
    for (const sample of samples) {
      const expected = forwardKinematics(sample.angles).T_base_tool.position;
      expect(sample.position.distanceTo(expected)).toBeLessThan(1e-12);
    }
  });

  it('takes the short periodic route across the base-joint branch cut', () => {
    const halfway = trajectoryAnglesAt([170, 0, 0], [-170, 0, 0], 0.5);
    expect(Math.abs(halfway[0])).toBeCloseTo(180, 10);
    expect(trajectoryAnglesAt([170, 0, 0], [-170, 0, 0], 1)[0]).toBeCloseTo(-170, 10);
  });
});
