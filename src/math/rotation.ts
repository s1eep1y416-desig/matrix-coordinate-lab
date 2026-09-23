import { Matrix4, MathUtils, Quaternion, Vector3 } from 'three';
import { matrixRows } from './transform';

export type Axis = 'X' | 'Y' | 'Z';
const AXES: Record<Axis, Vector3> = { X: new Vector3(1, 0, 0), Y: new Vector3(0, 1, 0), Z: new Vector3(0, 0, 1) };

export function rotationAbout(axis: Axis, degrees: number): Quaternion {
  return new Quaternion().setFromAxisAngle(AXES[axis], MathUtils.degToRad(degrees)).normalize();
}

/** Matrix product R_left R_right acts on column vectors rightmost first. */
export function rotationProduct(left: Quaternion, right: Quaternion): Quaternion {
  return left.clone().multiply(right).normalize();
}

export function rotationMatrix(quaternion: Quaternion): Matrix4 {
  return new Matrix4().makeRotationFromQuaternion(quaternion.clone().normalize());
}

export function validateRotationMatrix(matrix: Matrix4, tolerance = 1e-8): { orthogonalityError: number; determinant: number; valid: boolean } {
  const orthogonality = matrix.clone().transpose().multiply(matrix);
  const rows = matrixRows(orthogonality, 3);
  const orthogonalityError = Math.max(...rows.flatMap((row, r) => row.map((value, c) => Math.abs(value - (r === c ? 1 : 0)))));
  const determinant = matrix.determinant();
  return { orthogonalityError, determinant, valid: orthogonalityError <= tolerance && Math.abs(determinant - 1) <= tolerance };
}

export function validateRotation(quaternion: Quaternion, tolerance = 1e-8) {
  return validateRotationMatrix(rotationMatrix(quaternion), tolerance);
}
