import { Matrix3, Matrix4, MathUtils, Quaternion, Vector3 } from 'three';

export type Axis = 'X' | 'Y' | 'Z';
export type RotationPair = 'XY' | 'YZ' | 'ZX';
const AXES: Record<Axis, Vector3> = { X: new Vector3(1, 0, 0), Y: new Vector3(0, 1, 0), Z: new Vector3(0, 0, 1) };

export function rotationAbout(axis: Axis, degrees: number): Quaternion {
  return new Quaternion().setFromAxisAngle(AXES[axis], MathUtils.degToRad(degrees)).normalize();
}

/** Matrix product R_left R_right acts on column vectors rightmost first. */
export function rotationProduct(left: Quaternion, right: Quaternion): Quaternion {
  return left.clone().multiply(right).normalize();
}

/** Both products use column vectors: the rightmost rotation acts first. */
export function compareRotationOrder(angles: Record<Axis, number>, pair: RotationPair) {
  const [firstAxis, secondAxis] = pair.split('') as [Axis, Axis];
  const first = rotationAbout(firstAxis, angles[firstAxis]);
  const second = rotationAbout(secondAxis, angles[secondAxis]);
  return {
    firstAxis, secondAxis, first, second,
    forward: rotationProduct(first, second),
    reverse: rotationProduct(second, first),
  };
}

export function rotationMatrix(quaternion: Quaternion): Matrix4 {
  return new Matrix4().makeRotationFromQuaternion(quaternion.clone().normalize());
}

export function validateRotationMatrix(matrix: Matrix4, tolerance = 1e-8): { orthogonalityError: number; determinant: number; valid: boolean } {
  const rotation = new Matrix3().setFromMatrix4(matrix);
  const orthogonality = rotation.clone().transpose().multiply(rotation);
  const rows = Array.from({ length: 3 }, (_, row) => Array.from({ length: 3 }, (_, column) => orthogonality.elements[column * 3 + row]));
  const orthogonalityError = Math.max(...rows.flatMap((row, r) => row.map((value, c) => Math.abs(value - (r === c ? 1 : 0)))));
  const determinant = rotation.determinant();
  return { orthogonalityError, determinant, valid: orthogonalityError <= tolerance && Math.abs(determinant - 1) <= tolerance };
}

export function validateRotation(quaternion: Quaternion, tolerance = 1e-8) {
  return validateRotationMatrix(rotationMatrix(quaternion), tolerance);
}
