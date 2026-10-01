import { Quaternion, Vector3 } from 'three';
import { forwardKinematics, LINK_LENGTHS, wrapDegrees, type JointAngles } from './kinematics';
import { rotationAbout } from './rotation';

export type Jacobian3 = [
  [number, number, number],
  [number, number, number],
  [number, number, number],
];

export interface IKStepResult {
  angles: JointAngles;
  error: Vector3;
  errorNorm: number;
  nextErrorNorm: number;
  deltaRadians: JointAngles;
  jacobian: Jacobian3;
  acceptedScale: number;
}

export interface IKSolveResult {
  angles: JointAngles;
  converged: boolean;
  iterations: number;
  errorNorm: number;
  geometricallyReachable: boolean;
  usedAnalyticFallback: boolean;
}

const degrees = (value: number) => value * 180 / Math.PI;
const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

/** Closed-form position solutions for this lab's Z-Y-Y chain, including both base-yaw and elbow branches. */
export function analyticPositionIKSolutions(target: Vector3, tolerance = 1e-9): JointAngles[] {
  const [link1, link2, link3] = LINK_LENGTHS;
  const radial = Math.hypot(target.x, target.y);
  const azimuth = degrees(Math.atan2(target.y, target.x));
  const vertical = -target.z;
  const solutions: JointAngles[] = [];

  for (const radialSign of [1, -1] as const) {
    const planarTarget = radialSign * radial;
    const dx = planarTarget - link1;
    const distanceSquared = dx * dx + vertical * vertical;
    const cosineJoint3 = (distanceSquared - link2 * link2 - link3 * link3) / (2 * link2 * link3);
    if (cosineJoint3 < -1 - tolerance || cosineJoint3 > 1 + tolerance) continue;
    const joint3Magnitude = Math.acos(clamp(cosineJoint3, -1, 1));
    for (const elbowSign of [1, -1] as const) {
      const joint3 = elbowSign * joint3Magnitude;
      const joint2 = Math.atan2(vertical, dx) - Math.atan2(link3 * Math.sin(joint3), link2 + link3 * Math.cos(joint3));
      const angles: JointAngles = [
        wrapDegrees(azimuth + (radialSign < 0 ? 180 : 0)),
        wrapDegrees(degrees(joint2)),
        wrapDegrees(degrees(joint3)),
      ];
      if (target.distanceTo(forwardKinematics(angles).T_base_tool.position) <= 1e-7
        && !solutions.some((existing) => existing.every((angle, index) => Math.abs(wrapDegrees(angle - angles[index])) < 1e-7))) {
        solutions.push(angles);
      }
    }
  }
  return solutions;
}

export function isPositionReachable(target: Vector3): boolean {
  return analyticPositionIKSolutions(target).length > 0;
}

/** Position Jacobian for the lab's Z-Y-Y revolute chain, expressed in base coordinates. */
export function positionJacobian(angles: JointAngles): Jacobian3 {
  const fk = forwardKinematics(angles);
  const end = fk.T_base_tool.position;
  const q1 = rotationAbout('Z', angles[0]);
  const q12 = q1.clone().multiply(rotationAbout('Y', angles[1])).normalize();
  const axes = [
    new Vector3(0, 0, 1),
    new Vector3(0, 1, 0).applyQuaternion(q1),
    new Vector3(0, 1, 0).applyQuaternion(q12),
  ];
  const origins = [new Vector3(), fk.jointPositions[1], fk.jointPositions[2]];
  const columns = axes.map((axis, index) => axis.clone().cross(end.clone().sub(origins[index])));
  return [
    [columns[0].x, columns[1].x, columns[2].x],
    [columns[0].y, columns[1].y, columns[2].y],
    [columns[0].z, columns[1].z, columns[2].z],
  ];
}

function solve3x3(matrix: number[][], vector: number[]): number[] | null {
  const augmented = matrix.map((row, index) => [...row, vector[index]]);
  for (let pivot = 0; pivot < 3; pivot++) {
    let best = pivot;
    for (let row = pivot + 1; row < 3; row++) if (Math.abs(augmented[row][pivot]) > Math.abs(augmented[best][pivot])) best = row;
    if (Math.abs(augmented[best][pivot]) < 1e-12) return null;
    [augmented[pivot], augmented[best]] = [augmented[best], augmented[pivot]];
    const divisor = augmented[pivot][pivot];
    for (let column = pivot; column < 4; column++) augmented[pivot][column] /= divisor;
    for (let row = 0; row < 3; row++) {
      if (row === pivot) continue;
      const factor = augmented[row][pivot];
      for (let column = pivot; column < 4; column++) augmented[row][column] -= factor * augmented[pivot][column];
    }
  }
  return augmented.map((row) => row[3]);
}

/** One damped-least-squares step: dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e, with a decreasing-error line search. */
export function ikStep(angles: JointAngles, target: Vector3, damping = 0.08, maxStepRadians = 0.28): IKStepResult {
  const current = forwardKinematics(angles).T_base_tool.position;
  const error = target.clone().sub(current);
  const jacobian = positionJacobian(angles);
  const jjT = Array.from({ length: 3 }, (_, row) => Array.from({ length: 3 }, (_, column) =>
    jacobian[row].reduce((sum, value, joint) => sum + value * jacobian[column][joint], 0) + (row === column ? damping * damping : 0),
  ));
  const y = solve3x3(jjT, error.toArray()) ?? [0, 0, 0];
  const rawDelta = Array.from({ length: 3 }, (_, joint) => jacobian.reduce((sum, row, axis) => sum + row[joint] * y[axis], 0));
  const largest = Math.max(...rawDelta.map(Math.abs), 1e-12);
  const limited = rawDelta.map((value) => value * Math.min(1, maxStepRadians / largest)) as JointAngles;
  let scale = 1;
  let next = angles;
  let nextError = error.length();
  while (scale >= 1 / 64) {
    const candidate = angles.map((angle, index) => {
      const next = angle + degrees(limited[index] * scale);
      return index === 0 ? wrapDegrees(next) : clamp(next, -180, 180);
    }) as JointAngles;
    const candidateError = target.distanceTo(forwardKinematics(candidate).T_base_tool.position);
    if (candidateError < nextError) { next = candidate; nextError = candidateError; break; }
    scale *= 0.5;
  }
  return {
    angles: [...next] as JointAngles,
    error,
    errorNorm: error.length(),
    nextErrorNorm: nextError,
    deltaRadians: limited.map((value) => value * (next === angles ? 0 : scale)) as JointAngles,
    jacobian,
    acceptedScale: next === angles ? 0 : scale,
  };
}

function solveFromSeed(initial: JointAngles, target: Vector3, damping: number, maxIterations: number, tolerance: number): Omit<IKSolveResult, 'geometricallyReachable' | 'usedAnalyticFallback'> {
  let angles = [...initial] as JointAngles;
  let errorNorm = target.distanceTo(forwardKinematics(angles).T_base_tool.position);
  let iterations = 0;
  while (iterations < maxIterations && errorNorm > tolerance) {
    const result = ikStep(angles, target, damping);
    iterations++;
    if (result.acceptedScale === 0 || Math.abs(errorNorm - result.nextErrorNorm) < 1e-10) break;
    angles = result.angles;
    errorNorm = result.nextErrorNorm;
  }
  return { angles, converged: errorNorm <= tolerance, iterations, errorNorm };
}

function distanceFromSeed(seed: JointAngles, candidate: JointAngles): number {
  return Math.hypot(wrapDegrees(candidate[0] - seed[0]), candidate[1] - seed[1], candidate[2] - seed[2]);
}

export function solveIK(initial: JointAngles, target: Vector3, damping = 0.08, maxIterations = 240, tolerance = 1e-4): IKSolveResult {
  const analyticSolutions = analyticPositionIKSolutions(target);
  const primary = solveFromSeed(initial, target, damping, maxIterations, tolerance);
  if (primary.converged) return { ...primary, geometricallyReachable: true, usedAnalyticFallback: false };
  if (!analyticSolutions.length) return { ...primary, geometricallyReachable: false, usedAnalyticFallback: false };

  // DLS is local. If it stalls at a singularity or joint boundary, restart from the closest exact geometric branch.
  const seed = [...analyticSolutions].sort((a, b) => distanceFromSeed(initial, a) - distanceFromSeed(initial, b))[0];
  const fallback = solveFromSeed(seed, target, damping, maxIterations, tolerance);
  return {
    ...fallback,
    iterations: primary.iterations + fallback.iterations,
    geometricallyReachable: true,
    usedAnalyticFallback: true,
  };
}

export function jacobianManipulability(jacobian: Jacobian3): number {
  const [a, b, c] = jacobian;
  return Math.abs(
    a[0] * (b[1] * c[2] - b[2] * c[1])
    - a[1] * (b[0] * c[2] - b[2] * c[0])
    + a[2] * (b[0] * c[1] - b[1] * c[0]),
  );
}

export function finiteDifferenceJacobian(angles: JointAngles, epsilonRadians = 1e-6): Jacobian3 {
  const base = forwardKinematics(angles).T_base_tool.position;
  const columns = angles.map((_, joint) => {
    const changed = [...angles] as JointAngles;
    changed[joint] += degrees(epsilonRadians);
    return forwardKinematics(changed).T_base_tool.position.sub(base).multiplyScalar(1 / epsilonRadians);
  });
  return [
    [columns[0].x, columns[1].x, columns[2].x],
    [columns[0].y, columns[1].y, columns[2].y],
    [columns[0].z, columns[1].z, columns[2].z],
  ];
}

export function orientationErrorVector(current: Quaternion, target: Quaternion): Vector3 {
  const delta = current.clone().invert().multiply(target).normalize();
  if (delta.w < 0) delta.set(-delta.x, -delta.y, -delta.z, -delta.w);
  const angle = 2 * Math.acos(clamp(delta.w, -1, 1));
  const sinHalf = Math.sqrt(Math.max(0, 1 - delta.w * delta.w));
  return sinHalf < 1e-8 ? new Vector3() : new Vector3(delta.x, delta.y, delta.z).multiplyScalar(angle / sinHalf);
}
