import { Matrix4, Quaternion, Vector3 } from 'three';

export interface Pose {
  position: Vector3;
  quaternion: Quaternion;
}

export const identityPose = (): Pose => ({ position: new Vector3(), quaternion: new Quaternion() });

export function makePose(position = new Vector3(), quaternion = new Quaternion()): Pose {
  return { position: position.clone(), quaternion: quaternion.clone().normalize() };
}

/** ^A T_C = ^A T_B · ^B T_C. Point and orientation conventions are active, right handed. */
export function composePoses(T_A_B: Pose, T_B_C: Pose): Pose {
  return {
    position: T_A_B.position.clone().add(T_B_C.position.clone().applyQuaternion(T_A_B.quaternion)),
    quaternion: T_A_B.quaternion.clone().multiply(T_B_C.quaternion).normalize(),
  };
}

export function inversePose(T_A_B: Pose): Pose {
  const inverseQuaternion = T_A_B.quaternion.clone().invert().normalize();
  return {
    position: T_A_B.position.clone().negate().applyQuaternion(inverseQuaternion),
    quaternion: inverseQuaternion,
  };
}

export function relativePose(T_W_A: Pose, T_W_B: Pose): Pose {
  return composePoses(inversePose(T_W_A), T_W_B);
}

export function transformPoint(T_A_B: Pose, p_B: Vector3): Vector3 {
  return p_B.clone().applyQuaternion(T_A_B.quaternion).add(T_A_B.position);
}

export function inverseTransformPoint(T_A_B: Pose, p_A: Vector3): Vector3 {
  return transformPoint(inversePose(T_A_B), p_A);
}

export function poseMatrix(T_A_B: Pose): Matrix4 {
  return new Matrix4().compose(T_A_B.position, T_A_B.quaternion, new Vector3(1, 1, 1));
}

/** Three.js stores matrices column major; UI presents conventional row major matrices. */
export function matrixRows(matrix: Matrix4, size: 3 | 4 = 4): number[][] {
  return Array.from({ length: size }, (_, row) =>
    Array.from({ length: size }, (_, column) => matrix.elements[column * 4 + row]),
  );
}

export function validateHomogeneous(matrix: Matrix4, tolerance = 1e-8): boolean {
  const row = matrixRows(matrix, 4)[3];
  return row.every((value, index) => Math.abs(value - (index === 3 ? 1 : 0)) <= tolerance);
}

export function matrixMaxError(actual: Matrix4, expected: Matrix4): number {
  return Math.max(...actual.elements.map((value, index) => Math.abs(value - expected.elements[index])));
}
