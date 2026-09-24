import { beforeEach, describe, expect, it } from 'vitest';
import { Vector3 } from 'three';
import { sameOrientation } from '../math/quaternion';
import { matrixMaxError, poseMatrix } from '../math/transform';
import { relativeFramePose, useLabStore, worldPoseFor } from './labStore';

beforeEach(() => useLabStore.getState().reset());

describe('frame limits and hierarchy', () => {
  it('clamps both precise input and 3D dragging to the same position limit', () => {
    const store = useLabStore.getState();
    store.setConstraint('A', 'tx', { min: -0.5, max: 0.5 });
    expect(store.setFramePosition('A', 0, 4)).toBe(0.5);
    store.moveFrameWorld('A', new Vector3(-3, 0, 0));
    expect(useLabStore.getState().frames.find((frame) => frame.id === 'A')!.pose.position.x).toBe(-0.5);
  });

  it('keeps a locked axis fixed and rejects a reversed boundary edit', () => {
    const store = useLabStore.getState();
    store.setConstraint('A', 'rz', { min: -30, max: 30 });
    store.setConstraint('A', 'rz', { min: 80 });
    expect(useLabStore.getState().frames.find((frame) => frame.id === 'A')!.constraints.rz).toMatchObject({ min: 30, max: 30 });
    store.setConstraint('A', 'tx', { locked: true });
    const before = useLabStore.getState().frames.find((frame) => frame.id === 'A')!.pose.position.x;
    store.setFramePosition('A', 0, -9);
    store.moveFrameWorld('A', new Vector3(8, 0, 0));
    expect(useLabStore.getState().frames.find((frame) => frame.id === 'A')!.pose.position.x).toBe(before);
  });

  it('preserves world pose when reparenting within available limits', () => {
    const store = useLabStore.getState();
    store.addFrame('world');
    const before = worldPoseFor(useLabStore.getState().frames, 'B');
    store.reparentFrame('B', 'A');
    const after = worldPoseFor(useLabStore.getState().frames, 'B');
    expect(after.position.distanceTo(before.position)).toBeLessThan(1e-9);
    expect(after.quaternion.angleTo(before.quaternion)).toBeLessThan(1e-7);
  });

  it('can preserve either orientation or the three Euler values when changing order', () => {
    const store = useLabStore.getState();
    store.setFrameEuler('A', 0, 20);
    store.setFrameEuler('A', 1, 30);
    store.setFrameEuler('A', 2, 40);
    const original = useLabStore.getState().frames.find((frame) => frame.id === 'A')!.pose.quaternion.clone();
    store.setEulerOrder('XYZ');
    const preserved = useLabStore.getState().frames.find((frame) => frame.id === 'A')!.pose.quaternion;
    expect(sameOrientation(original, preserved)).toBe(true);
    store.setEulerOrder('ZYX');
    store.setEulerOrderBehavior('angles');
    store.setEulerOrder('XYZ');
    const changed = useLabStore.getState().frames.find((frame) => frame.id === 'A')!.pose.quaternion;
    expect(sameOrientation(original, changed)).toBe(false);
  });

  it('keeps a multi-level frame tree consistent with matrix products after editing and reparenting', () => {
    const store = useLabStore.getState();
    store.setFramePosition('A', 2, 0.8);
    store.setFrameEuler('A', 0, 25);
    store.setFrameEuler('A', 1, -35);
    store.addFrame('A');
    const state = useLabStore.getState();
    const A = state.frames.find((frame) => frame.id === 'A')!;
    const B = state.frames.find((frame) => frame.id === 'B')!;
    const matrixWorldB = poseMatrix(A.pose).multiply(poseMatrix(B.pose));
    expect(matrixMaxError(poseMatrix(worldPoseFor(state.frames, 'B')), matrixWorldB)).toBeLessThan(1e-9);
    expect(matrixMaxError(poseMatrix(relativeFramePose(state.frames, 'A', 'B')), poseMatrix(B.pose))).toBeLessThan(1e-9);
    store.reparentFrame('B', 'world');
    const reparented = useLabStore.getState();
    expect(matrixMaxError(poseMatrix(worldPoseFor(reparented.frames, 'B')), matrixWorldB)).toBeLessThan(1e-9);
  });
});
