import { describe, expect, it } from 'vitest';
import { Matrix4, Quaternion, Vector3 } from 'three';
import { EULER_ORDERS, eulerFromQuaternion, quaternionFromEuler } from './euler';
import { axisAngleFromQuaternion, quaternionFromAxisAngle, sameOrientation } from './quaternion';
import { forwardKinematics, LINK_LENGTHS } from './kinematics';
import { rotationAbout, rotationProduct, validateRotation, validateRotationMatrix } from './rotation';
import { composePoses, identityPose, inversePose, inverseTransformPoint, makePose, poseMatrix, relativePose, transformPoint, validateHomogeneous } from './transform';

const expectVector = (actual: Vector3, expected: Vector3, tolerance = 1e-9) => {
  expect(actual.distanceTo(expected)).toBeLessThan(tolerance);
};

const expectIdentityMatrix = (matrix: Matrix4, tolerance = 1e-9) => {
  const elements = matrix.elements;
  for (let column = 0; column < 4; column++) {
    for (let row = 0; row < 4; row++) {
      expect(Math.abs(elements[column * 4 + row] - (row === column ? 1 : 0))).toBeLessThan(tolerance);
    }
  }
};

describe('rotation representations', () => {
  it.each(EULER_ORDERS)('round-trips Euler → Quaternion → Euler for %s', (order) => {
    const original = quaternionFromEuler([31, -42, 67], order);
    const roundTrip = quaternionFromEuler(eulerFromQuaternion(original, order), order);
    expect(sameOrientation(original, roundTrip)).toBe(true);
  });

  it('round-trips Quaternion → Matrix → Quaternion and q / -q', () => {
    const quaternion = quaternionFromAxisAngle(new Vector3(1, 2, -3), 93);
    const matrix = poseMatrix(makePose(new Vector3(), quaternion));
    const result = new Quaternion().setFromRotationMatrix(matrix);
    expect(sameOrientation(quaternion, result)).toBe(true);
    expect(sameOrientation(quaternion, new Quaternion(-quaternion.x, -quaternion.y, -quaternion.z, -quaternion.w))).toBe(true);
    const axisAngle = axisAngleFromQuaternion(quaternion);
    expect(sameOrientation(quaternionFromAxisAngle(axisAngle.axis, axisAngle.angleDeg), quaternion)).toBe(true);
  });

  it('makes RxRy different from RyRx and validates a rotation', () => {
    const Rx = rotationAbout('X', 40);
    const Ry = rotationAbout('Y', 55);
    expect(sameOrientation(rotationProduct(Rx, Ry), rotationProduct(Ry, Rx))).toBe(false);
    const validation = validateRotation(rotationProduct(Rx, Ry));
    expect(validation.valid).toBe(true);
    expect(validation.determinant).toBeCloseTo(1, 10);
    expect(validateRotationMatrix(new Matrix4().makeScale(2, 1, 1)).valid).toBe(false);
  });
});

describe('transform chain', () => {
  const T_world_A = makePose(new Vector3(1, 2, 3), quaternionFromEuler([20, -10, 35], 'ZYX'));
  const T_A_B = makePose(new Vector3(-0.4, 0.6, 1.1), quaternionFromEuler([0, 25, 40], 'XYZ'));

  it('computes T · inverse(T) ≈ I and a homogeneous last row', () => {
    const product = poseMatrix(composePoses(T_world_A, inversePose(T_world_A)));
    expectIdentityMatrix(product);
    expect(validateHomogeneous(poseMatrix(T_world_A))).toBe(true);
  });

  it('composes multiple relative transforms', () => {
    const T_B_C = makePose(new Vector3(0.5, -0.2, 0), rotationAbout('Z', 30));
    const T_world_C = composePoses(composePoses(T_world_A, T_A_B), T_B_C);
    const matrixProduct = poseMatrix(T_world_A).multiply(poseMatrix(T_A_B)).multiply(poseMatrix(T_B_C));
    expectIdentityMatrix(poseMatrix(T_world_C).clone().invert().multiply(matrixProduct));
    const T_world_B = composePoses(T_world_A, T_A_B);
    const recovered = relativePose(T_world_A, T_world_B);
    expectIdentityMatrix(poseMatrix(recovered).clone().invert().multiply(poseMatrix(T_A_B)));
  });

  it('changes point coordinates without changing the spatial point', () => {
    const pointA = new Vector3(0.2, -1, 0.4);
    const pointWorld = transformPoint(T_world_A, pointA);
    expectVector(inverseTransformPoint(T_world_A, pointWorld), pointA);
    expectVector(transformPoint(identityPose(), pointWorld), pointWorld);
  });
});

describe('3-link forward kinematics', () => {
  it('extends to the total link length when all joints are zero', () => {
    const fk = forwardKinematics([0, 0, 0]);
    expectVector(fk.T_base_tool.position, new Vector3(LINK_LENGTHS.reduce((sum, value) => sum + value, 0), 0, 0));
    expect(fk.jointPositions).toHaveLength(4);
    expectIdentityMatrix(poseMatrix(fk.T_base_tool).clone().invert().multiply(
      poseMatrix(fk.T_base_link1).multiply(poseMatrix(fk.T_link1_link2)).multiply(poseMatrix(fk.T_link2_link3)),
    ));
  });

  it('rotates all downstream links when the base joint moves', () => {
    const fk = forwardKinematics([90, 0, 0]);
    expectVector(fk.T_base_tool.position, new Vector3(0, 3.05, 0));
    expectVector(fk.jointPositions[1], new Vector3(0, 1.25, 0));
  });
});
