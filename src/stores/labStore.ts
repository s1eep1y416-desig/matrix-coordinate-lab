import { Quaternion, Vector3 } from 'three';
import { create } from 'zustand';
import { eulerFromQuaternion, quaternionFromEuler, type EulerOrder } from '../math/euler';
import { composePoses, identityPose, inversePose, makePose, relativePose, transformPoint, type Pose } from '../math/transform';
import type { JointAngles } from '../math/kinematics';

export type DofKey = 'tx' | 'ty' | 'tz' | 'rx' | 'ry' | 'rz';
export type LabMode = 'frames' | 'rotation' | 'chain' | 'fk';
export type PointReference = 'world' | 'frame';
export type EulerOrderBehavior = 'pose' | 'angles';

export interface DofConstraint { min: number; max: number; locked: boolean }
export interface FrameNode {
  id: string;
  name: string;
  parentId: string | null;
  pose: Pose;
  constraints: Record<DofKey, DofConstraint>;
  visible: boolean;
}

const DOFS: DofKey[] = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz'];
const hardBounds = (key: DofKey): [number, number] => key[0] === 't' ? [-10, 10] : [-360, 360];
export const frameConstraints = (): Record<DofKey, DofConstraint> => Object.fromEntries(
  DOFS.map((key) => [key, { min: hardBounds(key)[0], max: hardBounds(key)[1], locked: false }]),
) as Record<DofKey, DofConstraint>;

const frameA = (): FrameNode => ({
  id: 'A', name: 'Frame A', parentId: 'world',
  pose: makePose(new Vector3(1.35, 0.45, 0.55), quaternionFromEuler([0, 0, 30], 'ZYX')),
  constraints: frameConstraints(), visible: true,
});
const world = (): FrameNode => ({ id: 'world', name: 'World', parentId: null, pose: identityPose(), constraints: frameConstraints(), visible: true });
const initialPoint = () => transformPoint(frameA().pose, new Vector3(1.15, 0.65, 0.4));
const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

export function worldPoseFor(frames: FrameNode[], frameId: string, seen = new Set<string>()): Pose {
  if (frameId === 'world') return identityPose();
  const frame = frames.find((node) => node.id === frameId);
  if (!frame || seen.has(frameId)) return identityPose();
  seen.add(frameId);
  return composePoses(worldPoseFor(frames, frame.parentId ?? 'world', seen), frame.pose);
}

export function relativeFramePose(frames: FrameNode[], targetId: string, sourceId: string): Pose {
  return relativePose(worldPoseFor(frames, targetId), worldPoseFor(frames, sourceId));
}

function constrainPose(frame: FrameNode, candidate: Pose, order: EulerOrder, allowLocked = false): Pose {
  const nextPosition = candidate.position.clone();
  const currentAngles = eulerFromQuaternion(frame.pose.quaternion, order);
  const nextAngles = eulerFromQuaternion(candidate.quaternion, order);
  DOFS.forEach((key, index) => {
    const rule = frame.constraints[key];
    if (index < 3) {
      nextPosition.setComponent(index, rule.locked && !allowLocked ? frame.pose.position.getComponent(index) : clamp(nextPosition.getComponent(index), rule.min, rule.max));
    } else {
      const angleIndex = index - 3;
      nextAngles[angleIndex] = rule.locked && !allowLocked ? currentAngles[angleIndex] : clamp(nextAngles[angleIndex], rule.min, rule.max);
    }
  });
  return makePose(nextPosition, quaternionFromEuler(nextAngles, order));
}

function changeFrame(frames: FrameNode[], id: string, transform: (frame: FrameNode) => FrameNode): FrameNode[] {
  return frames.map((frame) => frame.id === id && id !== 'world' ? transform(frame) : frame);
}

interface LabState {
  mode: LabMode;
  frames: FrameNode[];
  nextFrameNumber: number;
  selectedFrameId: string;
  sourceId: string;
  targetId: string;
  eulerOrder: EulerOrder;
  eulerOrderBehavior: EulerOrderBehavior;
  pointWorld: Vector3;
  pointReference: PointReference;
  robotAngles: JointAngles;
  rotationX: number;
  rotationY: number;
  rotationSense: 'active' | 'passive';
  fkStep: number;
  setMode: (mode: LabMode) => void;
  selectFrame: (id: string) => void;
  setSource: (id: string) => void;
  setTarget: (id: string) => void;
  setEulerOrder: (order: EulerOrder) => void;
  setEulerOrderBehavior: (behavior: EulerOrderBehavior) => void;
  setFramePosition: (id: string, axis: 0 | 1 | 2, value: number) => number;
  setFrameEuler: (id: string, axis: 0 | 1 | 2, value: number) => number;
  setFrameQuaternion: (id: string, quaternion: Quaternion) => boolean;
  moveFrameWorld: (id: string, positionWorld: Vector3) => void;
  setConstraint: (id: string, key: DofKey, patch: Partial<DofConstraint>) => void;
  addFrame: (parentId?: string) => void;
  reparentFrame: (id: string, parentId: string) => void;
  deleteFrame: (id: string) => void;
  toggleFrame: (id: string) => void;
  setPointReference: (reference: PointReference) => void;
  setPointCoordinate: (referenceId: string, axis: 0 | 1 | 2, value: number) => number;
  setPointWorld: (position: Vector3) => void;
  setRobotAngle: (axis: 0 | 1 | 2, value: number) => void;
  setRotationDemo: (axis: 'X' | 'Y', value: number) => void;
  setRotationSense: (sense: 'active' | 'passive') => void;
  setFkStep: (step: number) => void;
  reset: () => void;
}

export const useLabStore = create<LabState>((set, get) => ({
  mode: 'frames', frames: [world(), frameA()], nextFrameNumber: 2,
  selectedFrameId: 'A', sourceId: 'A', targetId: 'world', eulerOrder: 'ZYX', eulerOrderBehavior: 'pose',
  pointWorld: initialPoint(), pointReference: 'world', robotAngles: [25, -30, 45],
  rotationX: 45, rotationY: 35, rotationSense: 'active', fkStep: 3,
  setMode: (mode) => {
    if (mode === 'chain' && !get().frames.some((frame) => frame.id === 'B')) {
      const B: FrameNode = { id: 'B', name: 'Frame B', parentId: 'A', pose: makePose(new Vector3(1.25, 0.15, 0.55), quaternionFromEuler([20, -10, 18], get().eulerOrder)), constraints: frameConstraints(), visible: true };
      set((state) => ({ frames: [...state.frames, B], nextFrameNumber: Math.max(state.nextFrameNumber, 3), mode, selectedFrameId: 'B', sourceId: 'B', targetId: 'world' }));
    } else set({ mode });
  },
  selectFrame: (id) => set((state) => state.frames.some((frame) => frame.id === id) ? { selectedFrameId: id, sourceId: id, targetId: state.targetId === id ? 'world' : state.targetId } : {}),
  setSource: (id) => set((state) => state.frames.some((frame) => frame.id === id) ? { sourceId: id, selectedFrameId: id } : {}),
  setTarget: (id) => set((state) => state.frames.some((frame) => frame.id === id) ? { targetId: id } : {}),
  setEulerOrder: (eulerOrder) => set((state) => {
    if (state.eulerOrderBehavior === 'pose' || eulerOrder === state.eulerOrder) return { eulerOrder };
    const selected = state.frames.find((frame) => frame.id === state.selectedFrameId);
    if (!selected || selected.id === 'world') return { eulerOrder };
    const sameNumbers = eulerFromQuaternion(selected.pose.quaternion, state.eulerOrder);
    const candidate = makePose(selected.pose.position, quaternionFromEuler(sameNumbers, eulerOrder));
    return { eulerOrder, frames: changeFrame(state.frames, selected.id, (frame) => ({ ...frame, pose: constrainPose(frame, candidate, eulerOrder, true) })) };
  }),
  setEulerOrderBehavior: (eulerOrderBehavior) => set({ eulerOrderBehavior }),
  setFramePosition: (id, axis, value) => {
    const state = get(), frame = state.frames.find((item) => item.id === id);
    if (!frame || frame.id === 'world' || !Number.isFinite(value)) return 0;
    const key = DOFS[axis], rule = frame.constraints[key];
    const result = rule.locked ? frame.pose.position.getComponent(axis) : clamp(value, rule.min, rule.max);
    set({ frames: changeFrame(state.frames, id, (item) => ({ ...item, pose: makePose(item.pose.position.clone().setComponent(axis, result), item.pose.quaternion) })) });
    return result;
  },
  setFrameEuler: (id, axis, value) => {
    const state = get(), frame = state.frames.find((item) => item.id === id);
    if (!frame || frame.id === 'world' || !Number.isFinite(value)) return 0;
    const angles = eulerFromQuaternion(frame.pose.quaternion, state.eulerOrder);
    const rule = frame.constraints[DOFS[axis + 3]];
    if (!rule.locked) angles[axis] = clamp(value, rule.min, rule.max);
    const quaternion = quaternionFromEuler(angles, state.eulerOrder);
    set({ frames: changeFrame(state.frames, id, (item) => ({ ...item, pose: makePose(item.pose.position, quaternion) })) });
    return eulerFromQuaternion(quaternion, state.eulerOrder)[axis];
  },
  setFrameQuaternion: (id, quaternion) => {
    const state = get(), frame = state.frames.find((item) => item.id === id);
    if (!frame || frame.id === 'world' || quaternion.lengthSq() < 1e-16) return false;
    const candidate = makePose(frame.pose.position, quaternion);
    set({ frames: changeFrame(state.frames, id, (item) => ({ ...item, pose: constrainPose(item, candidate, state.eulerOrder) })) });
    return true;
  },
  moveFrameWorld: (id, positionWorld) => {
    const state = get(), frame = state.frames.find((item) => item.id === id);
    if (!frame || frame.id === 'world') return;
    const parentWorld = worldPoseFor(state.frames, frame.parentId ?? 'world');
    const localPosition = transformPoint(inversePose(parentWorld), positionWorld);
    const candidate = makePose(localPosition, frame.pose.quaternion);
    set({ frames: changeFrame(state.frames, id, (item) => ({ ...item, pose: constrainPose(item, candidate, state.eulerOrder) })) });
  },
  setConstraint: (id, key, patch) => {
    const state = get(), frame = state.frames.find((item) => item.id === id);
    if (!frame || id === 'world') return;
    const previous = frame.constraints[key], [hardMin, hardMax] = hardBounds(key);
    const requestedMin = Number.isFinite(patch.min) ? clamp(patch.min!, hardMin, hardMax) : previous.min;
    const requestedMax = Number.isFinite(patch.max) ? clamp(patch.max!, hardMin, hardMax) : previous.max;
    // Editing one endpoint never silently moves the other endpoint.
    const onlyMin = patch.min !== undefined && patch.max === undefined;
    const onlyMax = patch.max !== undefined && patch.min === undefined;
    const min = onlyMin ? Math.min(requestedMin, previous.max) : onlyMax ? previous.min : Math.min(requestedMin, requestedMax);
    const max = onlyMax ? Math.max(requestedMax, previous.min) : onlyMin ? previous.max : Math.max(requestedMin, requestedMax);
    const nextConstraint = { min, max, locked: patch.locked ?? previous.locked };
    const nextFrame = { ...frame, constraints: { ...frame.constraints, [key]: nextConstraint } };
    // Changing limits may reposition a locked frame; the displayed pose always satisfies the displayed range.
    const pose = constrainPose(nextFrame, frame.pose, state.eulerOrder, true);
    set({ frames: changeFrame(state.frames, id, () => ({ ...nextFrame, pose })) });
  },
  addFrame: (parentId) => set((state) => {
    if (state.frames.length >= 8) return {};
    const parent = state.frames.find((frame) => frame.id === (parentId ?? state.selectedFrameId)) ?? state.frames[0];
    const number = state.nextFrameNumber;
    const id = String.fromCharCode(64 + number);
    const frame: FrameNode = { id, name: `Frame ${id}`, parentId: parent.id, pose: makePose(new Vector3(1.1, 0.25, 0.35), quaternionFromEuler([0, 0, 15], state.eulerOrder)), constraints: frameConstraints(), visible: true };
    return { frames: [...state.frames, frame], nextFrameNumber: number + 1, selectedFrameId: id, sourceId: id, targetId: 'world' };
  }),
  reparentFrame: (id, parentId) => {
    const state = get(), frame = state.frames.find((item) => item.id === id);
    if (!frame || id === 'world' || id === parentId || !state.frames.some((item) => item.id === parentId)) return;
    let ancestor: string | null = parentId;
    while (ancestor) {
      if (ancestor === id) return;
      ancestor = state.frames.find((item) => item.id === ancestor)?.parentId ?? null;
    }
    const oldWorld = worldPoseFor(state.frames, id), parentWorld = worldPoseFor(state.frames, parentId);
    const local = composePoses(inversePose(parentWorld), oldWorld);
    set({ frames: changeFrame(state.frames, id, (item) => ({ ...item, parentId, pose: constrainPose(item, local, state.eulerOrder) })) });
  },
  deleteFrame: (id) => set((state) => {
    if (id === 'world' || id === 'A' || state.frames.some((frame) => frame.parentId === id)) return {};
    return { frames: state.frames.filter((frame) => frame.id !== id), selectedFrameId: 'A', sourceId: 'A', targetId: state.targetId === id ? 'world' : state.targetId };
  }),
  toggleFrame: (id) => set((state) => ({ frames: changeFrame(state.frames, id, (frame) => ({ ...frame, visible: !frame.visible })) })),
  setPointReference: (pointReference) => set({ pointReference }),
  setPointCoordinate: (referenceId, axis, value) => {
    const state = get();
    if (!Number.isFinite(value)) return 0;
    const reference = worldPoseFor(state.frames, referenceId);
    const local = transformPoint(inversePose(reference), state.pointWorld);
    local.setComponent(axis, clamp(value, -20, 20));
    set({ pointWorld: transformPoint(reference, local) });
    return local.getComponent(axis);
  },
  setPointWorld: (pointWorld) => set({ pointWorld: pointWorld.clone() }),
  setRobotAngle: (axis, value) => set((state) => {
    const robotAngles = [...state.robotAngles] as JointAngles;
    robotAngles[axis] = clamp(value, -180, 180);
    return { robotAngles };
  }),
  setRotationDemo: (axis, value) => set({ [axis === 'X' ? 'rotationX' : 'rotationY']: clamp(value, -180, 180) }),
  setRotationSense: (rotationSense) => set({ rotationSense }),
  setFkStep: (fkStep) => set({ fkStep: clamp(Math.round(fkStep), 0, 3) }),
  reset: () => set({ mode: 'frames', frames: [world(), frameA()], nextFrameNumber: 2, selectedFrameId: 'A', sourceId: 'A', targetId: 'world', eulerOrder: 'ZYX', eulerOrderBehavior: 'pose', pointWorld: initialPoint(), pointReference: 'world', robotAngles: [25, -30, 45], rotationX: 45, rotationY: 35, rotationSense: 'active', fkStep: 3 }),
}));
