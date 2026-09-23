import { MathUtils, Quaternion, Vector3 } from 'three';

export interface AxisAngle {
  axis: Vector3;
  angleDeg: number;
}

export function normalizedQuaternion(quaternion: Quaternion): Quaternion {
  if (quaternion.lengthSq() < 1e-16) throw new Error('零四元数没有方向');
  return quaternion.clone().normalize();
}

export function quaternionFromAxisAngle(axis: Vector3, angleDeg: number): Quaternion {
  if (axis.lengthSq() < 1e-16) throw new Error('旋转轴不能为零');
  return new Quaternion().setFromAxisAngle(axis.clone().normalize(), MathUtils.degToRad(angleDeg)).normalize();
}

export function axisAngleFromQuaternion(input: Quaternion): AxisAngle {
  const q = normalizedQuaternion(input);
  // q and -q are the same orientation; choose the representative with angle <= 180°.
  if (q.w < 0) q.set(-q.x, -q.y, -q.z, -q.w);
  const angle = 2 * Math.acos(MathUtils.clamp(q.w, -1, 1));
  const sine = Math.sqrt(Math.max(0, 1 - q.w * q.w));
  const axis = sine < 1e-8 ? new Vector3(1, 0, 0) : new Vector3(q.x / sine, q.y / sine, q.z / sine);
  return { axis, angleDeg: MathUtils.radToDeg(angle) };
}

export function sameOrientation(left: Quaternion, right: Quaternion, tolerance = 1e-8): boolean {
  return 1 - Math.abs(normalizedQuaternion(left).dot(normalizedQuaternion(right))) <= tolerance;
}
