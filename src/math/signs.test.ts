import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';
import { quaternionFromEuler } from './euler';
import { ikStep, orientationErrorVector, positionJacobian } from './inverseKinematics';
import { forwardKinematics, type JointAngles } from './kinematics';
import { axisAngleFromQuaternion, quaternionFromAxisAngle, sameOrientation } from './quaternion';
import { compareRotationOrder, evaluateRotationVector, rotationAbout, rotationMatrix, validateRotation, type Axis, type RotationPair } from './rotation';
import { inversePose, inverseTransformPoint, makePose, matrixRows, transformPoint } from './transform';

const expectVector = (actual: Vector3, expected: number[]) => {
  actual.toArray().forEach((value, index) => expect(value).toBeCloseTo(expected[index], 10));
};
const expectRows = (actual: number[][], expected: number[][]) => {
  actual.flat().forEach((value, index) => expect(value).toBeCloseTo(expected.flat()[index], 10));
};

describe('independent right-hand sign checks', () => {
  it.each([-90, -37, 0, 37, 90])('matches analytic elementary matrices at %s degrees', (angle) => {
    const radians = angle * Math.PI / 180, c = Math.cos(radians), s = Math.sin(radians);
    // Explicit column-vector definitions, not another call to the rotation implementation.
    expectRows(matrixRows(rotationMatrix(rotationAbout('X', angle)), 3), [[1, 0, 0], [0, c, -s], [0, s, c]]);
    expectRows(matrixRows(rotationMatrix(rotationAbout('Y', angle)), 3), [[c, 0, s], [0, 1, 0], [-s, 0, c]]);
    expectRows(matrixRows(rotationMatrix(rotationAbout('Z', angle)), 3), [[c, -s, 0], [s, c, 0], [0, 0, 1]]);
  });

  it.each([
    ['X', [0, 1, 0], [0, 0, 1]],
    ['Y', [1, 0, 0], [0, 0, -1]],
    ['Z', [1, 0, 0], [0, 1, 0]],
  ] as [Axis, number[], number[]][])('%s quarter-turn follows the right-hand rule in both directions', (axis, input, positive) => {
    expectVector(new Vector3(...input).applyQuaternion(rotationAbout(axis, 90)), positive);
    expectVector(new Vector3(...input).applyQuaternion(rotationAbout(axis, -90)), positive.map((value) => -value));
  });

  it('distinguishes active +90° motion from passive -90° coordinates', () => {
    const original = new Vector3(1, 0, 0), rotation = rotationAbout('Z', 90);
    const active = evaluateRotationVector(rotation, original, 'active');
    const passive = evaluateRotationVector(rotation, original, 'passive');
    expectVector(active.worldVector, [0, 1, 0]);
    expectVector(active.coordinates, [0, 1, 0]);
    expectVector(passive.worldVector, [1, 0, 0]);
    expectVector(passive.coordinates, [0, -1, 0]);
    expectVector(original, [1, 0, 0]);
  });

  it('uses -Rᵀp, not just -p, for inverse translation', () => {
    const T_W_A = makePose(new Vector3(1, 2, 3), rotationAbout('Z', 90));
    expectVector(inversePose(T_W_A).position, [-2, 1, -3]);
    expectVector(transformPoint(T_W_A, new Vector3(2, 0, 1)), [1, 4, 4]);
    expectVector(inverseTransformPoint(T_W_A, new Vector3(1, 4, 4)), [2, 0, 1]);
  });

  it('matches the analytic ZYX/RPY formula with mixed positive and negative angles', () => {
    const [r, p, y] = [23, -41, 67].map((angle) => angle * Math.PI / 180);
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    expectRows(matrixRows(rotationMatrix(quaternionFromEuler([23, -41, 67], 'ZYX')), 3), [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
    ]);
  });

  it('keeps the negative axis when a negative angle is represented with a positive axis-angle magnitude', () => {
    const q = quaternionFromAxisAngle(new Vector3(0, 0, 1), -90);
    const representation = axisAngleFromQuaternion(q);
    expectVector(representation.axis, [0, 0, -1]);
    expect(representation.angleDeg).toBeCloseTo(90, 10);
    expectVector(orientationErrorVector(new Quaternion(), q), [0, 0, -Math.PI / 2]);
    expectVector(orientationErrorVector(q, new Quaternion()), [0, 0, Math.PI / 2]);
  });
});

describe('rotation playback order and continuity', () => {
  it('applies the right matrix first at step one, then the left matrix at step two', () => {
    const angles = { X: 90, Y: 90, Z: 0 };
    const initial = compareRotationOrder(angles, 'XY', 0);
    expect(sameOrientation(initial.forward, new Quaternion())).toBe(true);
    const first = compareRotationOrder(angles, 'XY', 1);
    expectVector(new Vector3(0, 0, 1).applyQuaternion(first.forward), [1, 0, 0]);
    expectVector(new Vector3(0, 0, 1).applyQuaternion(first.reverse), [0, -1, 0]);
    const final = compareRotationOrder(angles, 'XY', 2);
    expectVector(new Vector3(0, 1, 0).applyQuaternion(final.forward), [0, 0, 1]);
    expectVector(new Vector3(0, 1, 0).applyQuaternion(final.reverse), [1, 0, 0]);
  });

  it.each(['XY', 'YZ', 'ZX'] as RotationPair[])('keeps %s valid and continuous through the step boundary', (pair) => {
    const angles = { X: -90, Y: 52, Z: -34 };
    [0, 0.5, 1, 1.5, 2].forEach((progress) => {
      const sample = compareRotationOrder(angles, pair, progress);
      expect(validateRotation(sample.forward).valid).toBe(true);
      expect(validateRotation(sample.reverse).valid).toBe(true);
      expect(sample.forward.length()).toBeCloseTo(1, 12);
    });
    const before = compareRotationOrder(angles, pair, 1 - 1e-6);
    const after = compareRotationOrder(angles, pair, 1 + 1e-6);
    expect(before.forward.angleTo(after.forward)).toBeLessThan(1e-5);
    expect(before.reverse.angleTo(after.reverse)).toBeLessThan(1e-5);
  });
});

describe('Z-Y-Y robot joint and solver signs', () => {
  it.each([
    [[90, 0, 0], [0, 3.05, 0]], [[-90, 0, 0], [0, -3.05, 0]],
    [[0, 90, 0], [1.25, 0, -1.8]], [[0, -90, 0], [1.25, 0, 1.8]],
    [[0, 0, 90], [2.25, 0, -0.8]], [[0, 0, -90], [2.25, 0, 0.8]],
  ] as [JointAngles, number[]][])('joint angles %j produce the expected signed endpoint', (angles, expected) => {
    expectVector(forwardKinematics(angles).T_base_tool.position, expected);
  });

  it('has the independently known zero-pose Jacobian: axis × (end − origin)', () => {
    expectRows(positionJacobian([0, 0, 0]), [[0, 0, 0], [3.05, 0, 0], [0, -1.8, -0.8]]);
  });

  it.each([-1, 1])('moves towards a target in signed Z direction %s using negative Y joint increments', (direction) => {
    const result = ikStep([0, 0, 0], new Vector3(3.05, 0, direction * 0.1));
    expect(result.angles[1] * direction).toBeLessThan(0);
    expect(result.angles[2] * direction).toBeLessThan(0);
    expect(result.nextErrorNorm).toBeLessThan(result.errorNorm);
    expect(forwardKinematics(result.angles).T_base_tool.position.z * direction).toBeGreaterThan(0);
  });
});
