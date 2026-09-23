import { Euler, Quaternion, MathUtils } from 'three';

export const EULER_ORDERS = ['XYZ', 'ZYX', 'ZXY', 'YXZ', 'YZX', 'XZY'] as const;
export type EulerOrder = (typeof EULER_ORDERS)[number];
export type AnglesDeg = [number, number, number];

/** Rx/Ry/Rz are intrinsic rotations in the chosen Three.js Euler order. */
export function quaternionFromEuler(angles: AnglesDeg, order: EulerOrder): Quaternion {
  return new Quaternion().setFromEuler(
    new Euler(MathUtils.degToRad(angles[0]), MathUtils.degToRad(angles[1]), MathUtils.degToRad(angles[2]), order),
  ).normalize();
}

export function eulerFromQuaternion(quaternion: Quaternion, order: EulerOrder): AnglesDeg {
  const euler = new Euler().setFromQuaternion(quaternion.clone().normalize(), order);
  return [euler.x, euler.y, euler.z].map(MathUtils.radToDeg) as AnglesDeg;
}

export function isNearGimbalLock(angles: AnglesDeg, order: EulerOrder, toleranceDeg = 0.5): boolean {
  const middleAxis = order[1];
  const value = angles[middleAxis === 'X' ? 0 : middleAxis === 'Y' ? 1 : 2];
  return Math.abs(Math.abs(value) - 90) <= toleranceDeg;
}
