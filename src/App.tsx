import { useEffect, useMemo, useState } from 'react';
import { Matrix4, Quaternion, Vector3 } from 'three';
import { EULER_ORDERS, eulerFromQuaternion, isNearGimbalLock, type EulerOrder } from './math/euler';
import { forwardKinematics } from './math/kinematics';
import { jacobianManipulability, positionJacobian } from './math/inverseKinematics';
import { axisAngleFromQuaternion } from './math/quaternion';
import { compareRotationOrder, evaluateRotationVector, ROTATION_DEMO_VECTOR, rotationAbout, rotationMatrix, validateRotation, type Axis, type RotationPair } from './math/rotation';
import { inversePose, inverseTransformPoint, matrixMaxError, poseMatrix, transformPoint, validateHomogeneous } from './math/transform';
import { Formula, MatrixView, VectorReadout } from './components/MathView';
import { NumberField, formatValue } from './components/NumberField';
import { Scene } from './components/Scene';
import { RotationPlayback } from './components/RotationPlayback';
import { framePathIds, useLabStore, worldPoseFor, relativeFramePose, type DofKey, type FrameNode, type LabMode } from './stores/labStore';
import { localize, useLocaleStore, type Language } from './stores/localeStore';
import 'katex/dist/katex.min.css';
import './styles.css';

const NAV: { id: LabMode; label: { zh: string; en: string }; short: string }[] = [
  { id: 'frames', label: { zh: '坐标系与变换', en: 'Frames' }, short: 'Frames + Transforms' },
  { id: 'rotation', label: { zh: '旋转矩阵', en: 'Rotation' }, short: 'Rotation' },
  { id: 'fk', label: { zh: '3-Link 正运动学', en: '3-Link FK' }, short: 'Kinematics' },
  { id: 'ik', label: { zh: '逆运动学', en: 'IK' }, short: 'IK Solver' },
  { id: 'pinocchio', label: { zh: 'Pinocchio', en: 'Pinocchio' }, short: 'Workflow' },
];
const navLabel = (mode: LabMode, language: Language) => NAV.find((item) => item.id === mode)?.label[language] ?? '';
const labels = ['X', 'Y', 'Z'] as const;
const dofKeys: DofKey[] = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz'];
const texId = (id: string) => id === 'world' ? 'W' : id.replace(/[^A-Za-z0-9]/g, '');
const vectorText = (vector: Vector3) => `[ ${vector.toArray().map(formatValue).join(' , ')} ]`;
const compactVectorText = (vector: Vector3) => `(${vector.toArray().map((value) => formatValue(Number(value.toFixed(3)))).join(', ')})`;
function frameDepth(frames: FrameNode[], id: string): number {
  let depth = 0;
  let current = frames.find((frame) => frame.id === id);
  while (current?.parentId && depth < frames.length) {
    depth++;
    current = frames.find((frame) => frame.id === current?.parentId);
  }
  return depth;
}

function PaneTitle({ eyebrow, title, aside }: { eyebrow: string; title: string; aside?: React.ReactNode }) {
  return <div className="pane-title"><div><span className="eyebrow">{eyebrow}</span><h2>{title}</h2></div>{aside}</div>;
}

function FrameTree() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { frames, selectedFrameId, selectFrame, addFrame, toggleFrame, deleteFrame } = useLabStore();
  return <section className="control-card">
    <PaneTitle eyebrow="TF TREE" title={l('坐标系层级', 'Frame hierarchy')} aside={<button className="small-dark-button" onClick={() => addFrame()} disabled={frames.length >= 8}>＋ {l('添加子系', 'Add child')}</button>} />
    <div className="frame-tree">
      {frames.map((frame) => <div className={`frame-row ${selectedFrameId === frame.id ? 'selected' : ''}`} key={frame.id}>
        <button className="frame-select" style={{ paddingLeft: `${12 + frameDepth(frames, frame.id) * 13}px` }} onClick={() => selectFrame(frame.id)}>
          <span className={`frame-glyph ${frame.id === 'world' ? 'world' : ''}`} />
          <span>{frame.name}</span><small>{frame.id === 'world' ? 'root' : `↳ ${frame.parentId}`}</small>
        </button>
        {frame.id !== 'world' && <button className="frame-eye" aria-label={`${frame.visible ? l('隐藏', 'Hide') : l('显示', 'Show')} ${frame.name}`} title={frame.visible ? l('隐藏坐标轴', 'Hide axes') : l('显示坐标轴', 'Show axes')} onClick={() => toggleFrame(frame.id)}>{frame.visible ? '◉' : '○'}</button>}
        {frame.id !== 'world' && frame.id !== 'A' && !frames.some((child) => child.parentId === frame.id) && <button className="frame-remove" aria-label={`${l('删除', 'Delete')} ${frame.name}`} onClick={() => deleteFrame(frame.id)}>×</button>}
      </div>)}
    </div>
    <p className="fine-print">{l('选中坐标系，再在 3D 场景中拖动其原点。子坐标系的位置与姿态始终相对于父级。', 'Select a frame, then drag its origin in the 3D scene. A child frame’s position and orientation are always relative to its parent.')}</p>
  </section>;
}

function FramePoseControls({ selected }: { selected: FrameNode }) {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { frames, eulerOrder, eulerOrderBehavior, setEulerOrder, setEulerOrderBehavior, setFramePosition, setFrameEuler, reparentFrame } = useLabStore();
  const euler = eulerFromQuaternion(selected.pose.quaternion, eulerOrder);
  const editable = selected.id !== 'world';
  return <section className="control-card">
    <PaneTitle eyebrow="POSE / ^P T_CHILD" title={l(`${selected.name} 的局部位姿`, `${selected.name} local pose`)} />
    {editable && <label className="select-label">{l('父坐标系', 'Parent frame')}
      <select value={selected.parentId ?? 'world'} onChange={(event) => reparentFrame(selected.id, event.target.value)}>
        {frames.filter((candidate) => candidate.id !== selected.id && !isDescendant(frames, candidate.id, selected.id)).map((candidate) => <option key={candidate.id} value={candidate.id}>{candidate.name}</option>)}
      </select>
    </label>}
    <div className="field-heading"><strong>{l('平移', 'Translation')}</strong><span>{l('相对于父坐标系 · m', 'Relative to parent · m')}</span></div>
    <div className="three-fields">
      {labels.map((axis, index) => <NumberField key={axis} id={`position-${axis}`} label={axis} unit="m" disabled={!editable || selected.constraints[dofKeys[index]].locked}
        value={selected.pose.position.getComponent(index)} min={selected.constraints[dofKeys[index]].min} max={selected.constraints[dofKeys[index]].max}
        onCommit={(value) => setFramePosition(selected.id, index as 0 | 1 | 2, value)} />)}
    </div>
    <div className="field-heading"><strong>{l('旋转', 'Rotation')} · {eulerOrder === 'ZYX' ? 'RPY' : `${eulerOrder} Euler`}</strong><span>{l('左右拖动数值 · °', 'Drag values · °')}</span></div>
    <div className="three-fields">
      {(['roll', 'pitch', 'yaw'] as const).map((axis, index) => <NumberField key={axis} id={`euler-${axis}`} label={axis} unit="°" scrub step={0.5}
        disabled={!editable || selected.constraints[dofKeys[index + 3]].locked} value={euler[index]}
        min={selected.constraints[dofKeys[index + 3]].min} max={selected.constraints[dofKeys[index + 3]].max}
        onCommit={(value) => setFrameEuler(selected.id, index as 0 | 1 | 2, value)} />)}
    </div>
    {editable && <div className="frame-presets" aria-label={l('旋转预设', 'Rotation presets')}>
      <button onClick={() => ([0, 0, 45] as const).forEach((value, index) => setFrameEuler(selected.id, index as 0 | 1 | 2, value))}>{l('绕 Z 45°', 'Rotate Z 45°')}</button>
      <button onClick={() => ([25, -20, 35] as const).forEach((value, index) => setFrameEuler(selected.id, index as 0 | 1 | 2, value))}>{l('空间倾斜', 'Spatial tilt')}</button>
      <button onClick={() => ([0, 0, 0] as const).forEach((value, index) => setFrameEuler(selected.id, index as 0 | 1 | 2, value))}>{l('清零旋转', 'Reset rotation')}</button>
    </div>}
    <label className="select-label">{l('欧拉角顺序', 'Euler order')}
      <select value={eulerOrder} onChange={(event) => setEulerOrder(event.target.value as EulerOrder)}>
        {EULER_ORDERS.map((order) => <option key={order}>{order}</option>)}
      </select>
    </label>
    <div className="segmented order-behavior"><button className={eulerOrderBehavior === 'pose' ? 'active' : ''} onClick={() => setEulerOrderBehavior('pose')}>{l('保留姿态', 'Preserve pose')}</button><button className={eulerOrderBehavior === 'angles' ? 'active' : ''} onClick={() => setEulerOrderBehavior('angles')}>{l('保留角度', 'Preserve angles')}</button></div>
    <p className="fine-print">{eulerOrderBehavior === 'pose' ? l('切换顺序只改变角度读数，空间姿态不变。', 'Changing order only changes the angle readout; spatial orientation stays fixed. ') : l('切换顺序时保留选中坐标系的三个角度数值，空间姿态会改变。', 'Changing order preserves the three angle values, so spatial orientation changes. ')}{l('ZYX / RPY 对应', 'ZYX / RPY corresponds to')} <Formula tex="R_z R_y R_x" />.</p>
    {isNearGimbalLock(euler, eulerOrder) && <div className="teaching-alert">{l('Gimbal Lock：中间轴接近 ±90°，第一轴与第三轴的转动方向重合。四元数表示仍保持有效。', 'Gimbal Lock: the middle axis is near ±90°, aligning the first and third rotation directions. The quaternion remains valid.')}</div>}
  </section>;
}

function isDescendant(frames: FrameNode[], candidateId: string, ancestorId: string): boolean {
  let current = frames.find((frame) => frame.id === candidateId);
  while (current?.parentId) {
    if (current.parentId === ancestorId) return true;
    current = frames.find((frame) => frame.id === current?.parentId);
  }
  return false;
}

function DofControls({ selected }: { selected: FrameNode }) {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const setConstraint = useLabStore((state) => state.setConstraint);
  if (selected.id === 'world') return null;
  return <details className="control-card dof-card">
    <summary><span><span className="eyebrow">JOINT-LIKE LIMITS</span><strong>{l('自由度与限位', 'Degrees of freedom & limits')}</strong></span><small>{dofKeys.filter((key) => !selected.constraints[key].locked).length} / 6 {l('可动', 'free')} ▾</small></summary>
    <p className="fine-print">{l('每个坐标系分别设置。锁定后当前值固定；修改边界时，姿态会落到新范围内。', 'Configure each frame independently. A locked value stays fixed; changing a bound clamps the pose into the new range.')}</p>
    <div className="dof-table">
      {dofKeys.map((key) => {
        const rule = selected.constraints[key];
        return <div className="dof-row" key={key}>
          <strong>{key[0] === 't' ? key[1].toUpperCase() : `R${key[1]}`}</strong>
          <label className="lock-toggle"><input type="checkbox" checked={rule.locked} onChange={(event) => setConstraint(selected.id, key, { locked: event.target.checked })} /><span>{l('锁', 'Lock')}</span></label>
          <NumberField label={l('下限', 'Min')} value={rule.min} onCommit={(value) => { setConstraint(selected.id, key, { min: value }); return useLabStore.getState().frames.find((frame) => frame.id === selected.id)!.constraints[key].min; }} />
          <NumberField label={l('上限', 'Max')} value={rule.max} onCommit={(value) => { setConstraint(selected.id, key, { max: value }); return useLabStore.getState().frames.find((frame) => frame.id === selected.id)!.constraints[key].max; }} />
        </div>;
      })}
    </div>
  </details>;
}

function PointControls({ selected }: { selected: FrameNode }) {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { frames, pointWorld, pointReference, setPointReference, setPointCoordinate } = useLabStore();
  const frameId = selected.id === 'world' ? 'A' : selected.id;
  const T_W_F = worldPoseFor(frames, frameId);
  const pFrame = inverseTransformPoint(T_W_F, pointWorld);
  const referenceId = pointReference === 'world' ? 'world' : frameId;
  const coordinates = pointReference === 'world' ? pointWorld : pFrame;
  return <section className="control-card">
    <PaneTitle eyebrow="SAME POINT · TWO DESCRIPTIONS" title={l('空间点 P', 'Point P in space')} />
    <div className="segmented"><button className={pointReference === 'world' ? 'active' : ''} onClick={() => setPointReference('world')}>{l('在 World 中', 'In World')}</button><button className={pointReference === 'frame' ? 'active' : ''} onClick={() => setPointReference('frame')}>{l(`在 ${frameId} 中`, `In ${frameId}`)}</button></div>
    <div className="three-fields">{labels.map((axis, index) => <NumberField key={axis} label={axis} unit="m" value={coordinates.getComponent(index)} onCommit={(value) => setPointCoordinate(referenceId, index as 0 | 1 | 2, value)} />)}</div>
    <div className="point-comparison"><span><b>World</b>{vectorText(pointWorld)}</span><span><b>{frameId}</b>{vectorText(pFrame)}</span></div>
    <p className="fine-print">{l('切换参考系不会移动点 P，只会改变描述它的坐标。', 'Changing the reference frame does not move Point P; it only changes its coordinates. ')}<Formula tex={`p_W = {}^{W}T_{${texId(frameId)}}\\,p_{${texId(frameId)}}`} /></p>
  </section>;
}

function QuaternionControls({ selected }: { selected: FrameNode }) {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const setFrameQuaternion = useLabStore((state) => state.setFrameQuaternion);
  const [draft, setDraft] = useState<[number, number, number, number]>([selected.pose.quaternion.x, selected.pose.quaternion.y, selected.pose.quaternion.z, selected.pose.quaternion.w]);
  useEffect(() => setDraft([selected.pose.quaternion.x, selected.pose.quaternion.y, selected.pose.quaternion.z, selected.pose.quaternion.w]), [selected.id, selected.pose.quaternion]);
  const norm = Math.hypot(...draft);
  const apply = () => setFrameQuaternion(selected.id, new Quaternion(...draft));
  return <section className="control-card">
    <PaneTitle eyebrow="ORIENTATION" title={l('四元数 q', 'Quaternion q')} aside={<span className="subtle-badge">ROS2: x y z w</span>} />
    <div className="four-fields">{(['x', 'y', 'z', 'w'] as const).map((axis, index) => <NumberField key={`${selected.id}-${axis}`} label={axis} value={draft[index]} disabled={selected.id === 'world'} onCommit={(value) => { setDraft((current) => current.map((n, i) => i === index ? value : n) as [number, number, number, number]); return value; }} />)}</div>
    <div className="inline-result"><span>{l('输入范数', 'Input norm')}</span><strong className={Math.abs(norm - 1) < 1e-6 ? 'good' : 'warn'}>{formatValue(norm)}</strong><button className="small-dark-button" disabled={selected.id === 'world' || norm < 1e-12} onClick={apply}>{l('应用并归一化', 'Apply & normalize')}</button></div>
    <p className="fine-print">{l('内部姿态始终使用单位四元数。', 'Orientation is stored internally as a unit quaternion. ')}<Formula tex="q\\text{ and }-q" /> {l('表示同一个方向；教材也可能使用 [w, x, y, z] 顺序。', 'represent the same orientation; some textbooks use the [w, x, y, z] order.')}</p>
  </section>;
}

function FrameControls() {
  const frames = useLabStore((state) => state.frames);
  const selectedFrameId = useLabStore((state) => state.selectedFrameId);
  const selected = frames.find((frame) => frame.id === selectedFrameId) ?? frames[0];
  return <><FrameTree /><TransformControls /><TransformChainControls /><FramePoseControls selected={selected} /><PointControls selected={selected} /><DofControls selected={selected} /><QuaternionControls selected={selected} /></>;
}

function RotationControls() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { rotationX, rotationY, rotationZ, rotationPair, setRotationPair, setRotationDemo, rotationSense, setRotationSense } = useLabStore();
  const angles: Record<Axis, number> = { X: rotationX, Y: rotationY, Z: rotationZ };
  const control = (axis: Axis) => <div className="slider-row" key={axis}>
    <strong>R{axis}</strong><input aria-label={l(`R${axis} 角度`, `R${axis} angle`)} type="range" min="-180" max="180" step="1" value={angles[axis]} onChange={(event) => setRotationDemo(axis, Number(event.target.value))} />
    <NumberField label={axis} unit="°" value={angles[axis]} scrub step={0.5} onCommit={(number) => { setRotationDemo(axis, number); return Math.max(-180, Math.min(180, number)); }} />
  </div>;
  const [firstAxis, secondAxis] = rotationPair;
  return <>
    <section className="control-card"><PaneTitle eyebrow="SINGLE-AXIS ROTATION" title={l('绕 X / Y / Z 旋转', 'Rotate about X / Y / Z')} />{(['X', 'Y', 'Z'] as const).map(control)}
      <p className="fine-print">{l('三个目标角度独立调节；展开「单轴矩阵」可查看', 'Adjust the three target angles independently; expand “single-axis matrices” to inspect')} <Formula tex="R_x,\;R_y,\;R_z" />{l('。播放时，A / B 矩阵显示当前进度的旋转。', '. During playback, matrices A / B show the in-progress rotation.')}</p>
      <p className="fine-print">{l('右手系 +90°：Rx 将 +Y 转向 +Z，Ry 将 +Z 转向 +X，Rz 将 +X 转向 +Y。−90° 的方向相反。', 'Right-handed +90°: Rx maps +Y toward +Z, Ry maps +Z toward +X, and Rz maps +X toward +Y. −90° reverses each direction.')}</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="ORDER MATTERS" title={l('选两个轴比较次序', 'Compare two axis orders')} />
      <div className="rotation-pair-buttons">{(['XY', 'YZ', 'ZX'] as RotationPair[]).map((pair) => <button key={pair} className={rotationPair === pair ? 'active' : ''} onClick={() => setRotationPair(pair)} aria-pressed={rotationPair === pair}>{pair[0]} / {pair[1]}</button>)}</div>
      <p className="fine-print">{l('当前比较', 'Comparing')} <Formula tex={`R_${firstAxis.toLowerCase()}R_${secondAxis.toLowerCase()}`} /> {l('与', 'and')} <Formula tex={`R_${secondAxis.toLowerCase()}R_${firstAxis.toLowerCase()}`} />. {l('采用列向量：右侧矩阵先作用；场景两侧的彩色坐标轴与下方矩阵完全对应。', 'With column vectors, the rightmost matrix acts first. The colored axes in the scene match the matrices below.')}</p>
    </section>
    <RotationPlayback />
    <section className="control-card"><PaneTitle eyebrow="ACTIVE / PASSIVE" title={l('旋转物体，还是改变描述？', 'Rotate the object or change its description?')} />
      <div className="segmented"><button className={rotationSense === 'active' ? 'active' : ''} onClick={() => setRotationSense('active')}>Active · {l('物体转', 'object')}</button><button className={rotationSense === 'passive' ? 'active' : ''} onClick={() => setRotationSense('passive')}>Passive · {l('坐标系转', 'frame')}</button></div>
      <p className="fine-print">{rotationSense === 'active' ? l('参考系固定，世界向量随旋转改变。', 'The reference frame stays fixed while the world vector rotates.') : l('空间向量固定，变的是观察它的参考坐标系；向量的世界位置不变。', 'The spatial vector stays fixed while its reference frame changes; its world position does not move.')}</p>
    </section>
  </>;
}

function TransformControls() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { frames, sourceId, targetId, setSource, setTarget } = useLabStore();
  return <section className="control-card"><PaneTitle eyebrow="SOURCE → TARGET" title={l('坐标系选择', 'Frame selection')} />
    <div className="two-selects"><label className="select-label">{l('源坐标系', 'Source frame')}<select value={sourceId} onChange={(event) => setSource(event.target.value)}>{frames.map((frame) => <option key={frame.id} value={frame.id}>{frame.name}</option>)}</select></label>
      <label className="select-label">{l('目标坐标系', 'Target frame')}<select value={targetId} onChange={(event) => setTarget(event.target.value)}>{frames.map((frame) => <option key={frame.id} value={frame.id}>{frame.name}</option>)}</select></label></div>
    <p className="fine-print">{l('同一个点 P 保持在原处；切换源与目标后，读数和', 'Point P stays fixed. After changing source and target, the readout and')} <Formula tex={`{}^{${texId(targetId)}}T_{${texId(sourceId)}}`} /> {l('同步变化。', 'update together.')}</p>
  </section>;
}

function TransformChainControls() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { frames, selectedFrameId, chainStep, setChainStep } = useLabStore();
  const path = framePathIds(frames, selectedFrameId);
  const effectiveStep = Math.min(chainStep, Math.max(0, path.length - 1));
  const frameName = (id: string) => frames.find((frame) => frame.id === id)?.name ?? id;
  return <section className="control-card"><PaneTitle eyebrow="CHAIN REPLAY" title={l('逐级查看变换链', 'Step through transform chain')} aside={<span className="subtle-badge">{effectiveStep} / {Math.max(0, path.length - 1)}</span>} />
    {path.length > 1 ? <div className="chain-step-buttons">{path.map((id, index) => <button key={id} className={effectiveStep === index ? 'active' : index < effectiveStep ? 'done' : ''} aria-pressed={effectiveStep === index} onClick={() => setChainStep(index)}><span>0{index}</span><strong>{frameName(id)}</strong></button>)}</div> : <p className="fine-print">{l('先在坐标系层级中选择一个子坐标系。', 'Select a child frame in the hierarchy first.')}</p>}
    <p className="fine-print">{l('点击某一级，金色链路和矩阵会累计到该坐标系；编辑仍作用于当前选中的最终坐标系。', 'Choose a step to accumulate the gold path and matrices to that frame. Editing still applies to the selected final frame.')}</p>
  </section>;
}

function FKControls() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { robotAngles, setRobotAngle, fkStep, setFkStep } = useLabStore();
  return <>
    <section className="control-card"><PaneTitle eyebrow="3 REVOLUTE JOINTS" title={l('关节角度', 'Joint angles')} />
      {robotAngles.map((angle, index) => <div className="slider-row" key={index}><strong>q{index + 1}</strong>
        <input aria-label={l(`关节 ${index + 1} 角度`, `Joint ${index + 1} angle`)} type="range" min="-180" max="180" step="1" value={angle} onChange={(event) => setRobotAngle(index as 0 | 1 | 2, Number(event.target.value))} />
        <NumberField label={l('角度', 'Angle')} unit="°" value={angle} onCommit={(number) => { setRobotAngle(index as 0 | 1 | 2, number); return Math.max(-180, Math.min(180, number)); }} /></div>)}
      <p className="fine-print">{l('J1 绕 Z，J2 与 J3 绕各自局部 Y。每段 link 沿自身 +X 伸出，长度依次为 1.25、1.00、0.80 m。', 'J1 rotates about Z; J2 and J3 rotate about their local Y axes. Each link extends along local +X with lengths 1.25, 1.00, and 0.80 m.')}</p>
      <p className="fine-print">{l('正角遵循右手定则：零位时，J1 正转朝 +Y；J2、J3 正转把其下游 +X 方向转向 −Z。', 'Positive angles follow the right-hand rule: at zero pose, positive J1 turns toward +Y, while positive J2/J3 turn downstream +X toward −Z.')}</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="CHAIN REPLAY" title={l('逐级查看矩阵链', 'Step through matrix chain')} />
      <div className="step-buttons">{['base', 'J1', 'J2', 'tool'].map((label, index) => <button key={label} className={fkStep === index ? 'active' : ''} onClick={() => setFkStep(index)}>{label}</button>)}</div>
      <p className="fine-print">{l('选择一步，场景只显示计算到该级的运动学骨架和坐标系，用于观察每一级位姿如何累积。', 'Select a step to show the kinematic skeleton and frames computed up to that point, so you can inspect how each pose accumulates.')}</p>
    </section>
  </>;
}

function IKControls() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { robotAngles, setRobotAngle, ikTarget, setIkTargetCoordinate, ikDamping, setIkDamping, stepIK, solveIK, ikIterations } = useLabStore();
  return <>
    <section className="control-card"><PaneTitle eyebrow="TARGET IN BASE" title={l('拖动或输入目标点', 'Drag or enter a target')} aside={<span className="subtle-badge">position IK</span>} />
      <div className="three-fields">{labels.map((axis, index) => <NumberField key={axis} label={axis} unit="m" value={ikTarget.getComponent(index)} min={-3.2} max={3.2} onCommit={(value) => setIkTargetCoordinate(index as 0 | 1 | 2, value)} />)}</div>
      <div className="preset-row"><button onClick={() => useLabStore.getState().setIkTarget(new Vector3(1.9, .8, .7))}>{l('目标 A', 'Target A')}</button><button onClick={() => useLabStore.getState().setIkTarget(new Vector3(1.25, -1.1, -.55))}>{l('目标 B', 'Target B')}</button><button onClick={() => useLabStore.getState().setIkTarget(new Vector3(3.4, 0, 0))}>{l('不可达点', 'Unreachable')}</button></div>
      <p className="fine-print">{l('左键可直接拖动绿色 Target；球壳外的目标无法收敛，用于观察残差和可达性。', 'Drag the green Target with the left button. Targets outside the shell cannot converge, which demonstrates residual error and reachability.')}</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="DAMPED LEAST SQUARES" title={l('Jacobian 迭代', 'Jacobian iteration')} />
      <div className="slider-row"><strong>λ</strong><input aria-label={l('阻尼系数', 'Damping coefficient')} type="range" min="0.001" max="0.5" step="0.001" value={ikDamping} onChange={(event) => setIkDamping(Number(event.target.value))} /><NumberField label={l('阻尼', 'Damping')} value={ikDamping} onCommit={(value) => { setIkDamping(value); return Math.max(.001, Math.min(1, value)); }} /></div>
      <div className="solver-actions"><button onClick={stepIK}>{l('单步迭代', 'Single step')}</button><button className="primary-action" onClick={solveIK}>{l('求解到收敛', 'Solve to convergence')}</button></div>
      <div className="inline-result"><span>{l('累计迭代', 'Iterations')}</span><strong>{ikIterations}</strong></div>
      <p className="fine-print"><Formula tex="\Delta q=J^T(JJ^T+\lambda^2I)^{-1}e" />. {l('阻尼抑制奇异位形附近的关节跳变；线搜索只接受让误差下降的步长。', 'Damping suppresses joint jumps near singularities; line search only accepts steps that reduce error.')}</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="INITIAL / CURRENT q" title={l('关节角', 'Joint angles')} />
      {robotAngles.map((angle, index) => <div className="slider-row" key={index}><strong>q{index + 1}</strong><input aria-label={l(`IK 关节 ${index + 1}`, `IK joint ${index + 1}`)} type="range" min="-180" max="180" value={angle} onChange={(event) => setRobotAngle(index as 0 | 1 | 2, Number(event.target.value))} /><NumberField label={l('角度', 'Angle')} unit="°" value={angle} onCommit={(value) => { setRobotAngle(index as 0 | 1 | 2, value); return Math.max(-180, Math.min(180, value)); }} /></div>)}
    </section>
  </>;
}

const PIN_STEPS = [
  ['01', { zh: '读取 URDF', en: 'Load URDF' }, { zh: 'buildModelFromUrdf() 建立关节、Link 与 Frame 拓扑', en: 'buildModelFromUrdf() builds the Joint, Link, and Frame topology' }],
  ['02', { zh: '建立数据', en: 'Create data' }, { zh: 'model.createData() 分配运动学计算缓存', en: 'model.createData() allocates the kinematics cache' }],
  ['03', { zh: '输入 q', en: 'Input q' }, { zh: '关节角从控制器进入模型，单位统一为 rad', en: 'Joint angles enter the model in radians' }],
  ['04', { zh: '执行 FK', en: 'Run FK' }, { zh: 'forwardKinematics(model, data, q) 递推各 Link 位姿', en: 'forwardKinematics(model, data, q) propagates all Link poses' }],
  ['05', { zh: '更新 Frame', en: 'Update Frames' }, { zh: 'updateFramePlacements(model, data) 得到 data.oMf', en: 'updateFramePlacements(model, data) computes data.oMf' }],
  ['06', { zh: '输出与显示', en: 'Output & render' }, { zh: '读取 end_link 的 SE(3)，同步末端数值与 3D 场景', en: 'Read end_link SE(3) and sync the readout with the 3D scene' }],
] as const;

function PinocchioControls() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { pinStep, setPinStep, robotAngles, setRobotAngle } = useLabStore();
  return <>
    <section className="control-card"><PaneTitle eyebrow="DOCUMENT WORKFLOW" title={l('Pinocchio FK 数据流', 'Pinocchio FK data flow')} />
      <div className="pin-step-list">{PIN_STEPS.map((step, index) => <button key={step[0]} className={pinStep === index ? 'active' : pinStep > index ? 'done' : ''} onClick={() => setPinStep(index)}><span>{step[0]}</span><strong>{step[1][language]}</strong><small>{step[2][language]}</small></button>)}</div>
      <p className="fine-print">{l('逐步点击，观察“关节角 → Pinocchio FK → 末端位姿 + 可视化”的数据怎样流动。', 'Step through how data flows from joint angles → Pinocchio FK → end-effector pose + visualization.')}</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="q · degrees in UI" title={l('输入关节配置', 'Input joint configuration')} />
      <div className="three-fields">{robotAngles.map((angle, index) => <NumberField key={index} label={`q${index + 1}`} unit="°" scrub step={.5} value={angle} onCommit={(value) => { setRobotAngle(index as 0 | 1 | 2, value); return Math.max(-180, Math.min(180, value)); }} />)}</div>
      <button className="wide-action" onClick={() => { setRobotAngle(0, 45); setRobotAngle(1, -30); setRobotAngle(2, 15); setPinStep(5); }}>{l('载入文档示例 45, −30, 15', 'Load example 45, −30, 15')}</button>
      <p className="fine-print">{l('界面便于学习而使用度；进入 Pinocchio 计算流程前统一转换为弧度。本实验使用 3 轴链路聚焦展示核心数据流。', 'The UI uses degrees for learning and converts them to radians before the Pinocchio workflow. This lab uses a 3-axis chain to focus on the core data flow.')}</p>
    </section>
  </>;
}

function matrixFromJacobian(rows: ReturnType<typeof positionJacobian>): Matrix4 {
  return new Matrix4().set(rows[0][0], rows[0][1], rows[0][2], 0, rows[1][0], rows[1][1], rows[1][2], 0, rows[2][0], rows[2][1], rows[2][2], 0, 0, 0, 0, 1);
}

function IKResults() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { robotAngles, ikTarget, ikIterations } = useLabStore();
  const fk = useMemo(() => forwardKinematics(robotAngles), [robotAngles]);
  const jacobian = positionJacobian(robotAngles);
  const error = ikTarget.clone().sub(fk.T_base_tool.position);
  const errorNorm = error.length();
  const reachable = ikTarget.length() <= 3.05 + 1e-8;
  const manipulability = jacobianManipulability(jacobian);
  const converged = errorNorm < 1e-3;
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">INVERSE KINEMATICS · POSITION</span><h2>{l('目标位置 → 关节角', 'Target position → joint angles')}</h2><p>{l('当前是教学用 3 关节运动学骨架。绿色点是目标，金色线是当前解；误差线会随每次 Jacobian 迭代缩短。', 'This is a teaching 3-joint kinematic skeleton. The green point is the target, the gold chain is the current solution, and the error line shortens with each Jacobian iteration.')}</p></div>
    <div className="equation-strip"><Formula tex="e=p_{target}-p(q),\quad \Delta q=J^T(JJ^T+\lambda^2I)^{-1}e" /><span>DLS + {l('下降线搜索', 'descent line search')}</span></div>
    <div className="readout-grid"><VectorReadout label={l('目标位置 · m', 'Target position · m')} values={ikTarget.toArray()} /><VectorReadout label={l('当前末端 · m', 'Current end effector · m')} values={fk.T_base_tool.position.toArray()} /><VectorReadout label="Cartesian error · m" values={error.toArray()} /><VectorReadout label={l('当前关节角 · °', 'Current joint angles · °')} values={robotAngles} unit="°" /></div>
    <div className="validation-line"><span className={converged ? 'good' : 'warn'}>● {converged ? l('已收敛', 'Converged') : reachable ? l('等待迭代', 'Awaiting iteration') : l('目标超出最大臂展', 'Target exceeds maximum reach')}</span><span>‖e‖ = {formatValue(errorNorm)} m</span><span>|det(J)| = {formatValue(manipulability)}</span><span>{l('迭代', 'Iterations')} {ikIterations}</span></div>
    <div className="matrix-layout"><MatrixView matrix={matrixFromJacobian(jacobian)} size={3} label="J_v(q)" /><MatrixView matrix={poseMatrix(fk.T_base_tool)} label="{}^{base}T_{tool}(q)" /></div>
    <div className="teaching-band"><strong>{l('为什么这里只有位置 IK？', 'Why position-only IK?')}</strong><span>{l('3 个关节只有 3 个自由度，因此用 3×3 的位置 Jacobian 匹配 x/y/z。完整位姿 IK 需要更多自由度，并可通过', 'Three joints provide only three degrees of freedom, so a 3×3 position Jacobian matches x/y/z. Full-pose IK needs more degrees of freedom and can use')} <Formula tex="\log_6(T_{current}^{-1}T_{target})" /> {l('构造旋转 + 平移的 6D SE(3) 误差。', 'to construct a 6D SE(3) rotation + translation error.')}</span></div>
  </div>;
}

function PinocchioResults() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { robotAngles, pinStep, eulerOrder } = useLabStore();
  const fk = useMemo(() => forwardKinematics(robotAngles), [robotAngles]);
  const q = fk.T_base_tool.quaternion;
  const euler = eulerFromQuaternion(q, eulerOrder);
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">PINOCCHIO × URDF × SE(3)</span><h2>{PIN_STEPS[pinStep][1][language]}</h2><p>{PIN_STEPS[pinStep][2][language]}</p></div>
    <div className="pin-pipeline">{PIN_STEPS.map((step, index) => <button key={step[0]} className={index === pinStep ? 'active' : index < pinStep ? 'done' : ''} onClick={() => useLabStore.getState().setPinStep(index)}><span>{step[0]}</span><strong>{step[1][language]}</strong></button>)}</div>
    <div className="equation-strip"><Formula tex="q\;\longrightarrow\;\mathrm{FK}\;\longrightarrow\;{}^{world}T_{end\_link}\in SE(3)" /><span>{l('数学结果与 3D 场景共用同一份 Pose', 'The math and 3D scene share the same Pose')}</span></div>
    <div className="matrix-layout"><MatrixView matrix={poseMatrix(fk.T_base_tool)} label="data.oMf[\mathrm{end\_link}]" /><MatrixView matrix={rotationMatrix(q)} size={3} label="R_{end\_link}" /></div>
    <div className="readout-grid"><VectorReadout label={l('q · rad (Pinocchio 输入)', 'q · rad (Pinocchio input)')} values={robotAngles.map((value) => value * Math.PI / 180)} /><VectorReadout label="translation · m" values={fk.T_base_tool.position.toArray()} /><VectorReadout label={`${eulerOrder} Euler · °`} values={euler} unit="°" /><VectorReadout label="Quaternion [x, y, z, w]" values={[q.x, q.y, q.z, q.w]} /></div>
    <div className="api-map"><div><code>pin.buildModelFromUrdf()</code><span>URDF → Model</span></div><div><code>pin.forwardKinematics()</code><span>q → Link placements</span></div><div><code>pin.updateFramePlacements()</code><span>Link → Frame placements</span></div><div><code>data.oMf[frame_id]</code><span>{l('读取世界到末端的 SE(3)', 'Read world-to-end SE(3)')}</span></div></div>
    <div className="teaching-band"><strong>{l('与实际工作流的关系', 'Relation to a real workflow')}</strong><span>{l('本页在浏览器中复现通用的数学与数据流；在 Python 工程中可由 Pinocchio 负责运动学计算，MeshCat 负责三维显示。', 'This page reproduces the general mathematics and data flow in the browser. In a Python project, Pinocchio can compute kinematics while MeshCat handles 3D visualization.')}</span></div>
  </div>;
}

function FrameResults() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { frames, selectedFrameId, sourceId, targetId, eulerOrder, pointWorld, chainStep } = useLabStore();
  const T_W_selected = worldPoseFor(frames, selectedFrameId);
  const T_selected_W = inversePose(T_W_selected);
  const T_target_source = relativeFramePose(frames, targetId, sourceId);
  const T_W_source = worldPoseFor(frames, sourceId);
  const T_W_target = worldPoseFor(frames, targetId);
  const pSource = inverseTransformPoint(T_W_source, pointWorld);
  const pTarget = inverseTransformPoint(T_W_target, pointWorld);
  const euler = eulerFromQuaternion(T_W_selected.quaternion, eulerOrder);
  const axisAngle = axisAngleFromQuaternion(T_W_selected.quaternion);
  const validation = validateRotation(T_W_selected.quaternion);
  const quaternion = T_W_selected.quaternion;
  const chainPath = framePathIds(frames, selectedFrameId);
  const effectiveChainStep = Math.min(chainStep, Math.max(0, chainPath.length - 1));
  const currentChainId = chainPath[effectiveChainStep] ?? 'world';
  const chainLinks = chainPath.slice(1, effectiveChainStep + 1).map((id) => frames.find((frame) => frame.id === id)).filter((frame): frame is FrameNode => Boolean(frame));
  const allChainLinks = chainPath.slice(1).map((id) => frames.find((frame) => frame.id === id)).filter((frame): frame is FrameNode => Boolean(frame));
  const relativeMatrix = poseMatrix(T_target_source);
  const independentRelativeMatrix = poseMatrix(T_W_target).invert().multiply(poseMatrix(T_W_source));
  const relativeError = matrixMaxError(relativeMatrix, independentRelativeMatrix);
  const inverseError = matrixMaxError(poseMatrix(T_selected_W), poseMatrix(T_W_selected).invert());
  const pointError = pSource.clone().applyMatrix4(independentRelativeMatrix).distanceTo(pTarget);
  const fullChainProduct = allChainLinks.reduce((product, frame) => product.multiply(poseMatrix(frame.pose)), new Matrix4());
  const chainError = matrixMaxError(poseMatrix(worldPoseFor(frames, selectedFrameId)), fullChainProduct);
  const maxError = Math.max(relativeError, inverseError, pointError, chainError);
  const matrixValid = maxError < 1e-8 && validation.valid && validateHomogeneous(relativeMatrix);
  return <div className="results-stack">
    <div className="conversion-strip" aria-live="polite">
      <div className="conversion-step"><span>{l('源坐标系', 'Source frame')} · {sourceId === 'world' ? 'World' : `Frame ${sourceId}`}</span><strong>{compactVectorText(pSource)}</strong></div>
      <span className="conversion-arrow">→</span>
      <div className="conversion-step"><span>{l('世界坐标', 'World coordinates')} · World</span><strong>{compactVectorText(pointWorld)}</strong></div>
      <span className="conversion-arrow">→</span>
      <div className="conversion-step result"><span>{l('目标坐标系', 'Target frame')} · {targetId === 'world' ? 'World' : `Frame ${targetId}`}</span><strong>{compactVectorText(pTarget)}</strong></div>
    </div>
    <div className="conversion-formula"><Formula tex={`\\tilde p_{${texId(targetId)}} = {}^{${texId(targetId)}}T_{${texId(sourceId)}}\\,\\tilde p_{${texId(sourceId)}}`} /><span>{l('点 P 的空间位置固定 · 齐次坐标 w = 1', 'Point P stays fixed in space · homogeneous coordinate w = 1')}</span></div>
    <div className="matrix-layout core-matrices">
      <MatrixView matrix={relativeMatrix} label={`{}^{${texId(targetId)}}T_{${texId(sourceId)}}`} />
    </div>
    <div className="validation-line"><span className={matrixValid ? 'good' : 'warn'}>● {matrixValid ? l('矩阵换算校验通过', 'Matrix conversion valid') : l('矩阵换算异常', 'Matrix conversion error')}</span><span>{l('相对变换误差', 'Relative-transform error')} {formatValue(relativeError)}</span><span>{l('逆矩阵误差', 'Inverse error')} {formatValue(inverseError)}</span><span>{l('点换算误差', 'Point conversion error')} {formatValue(pointError)}</span><span>{l('完整链连乘误差', 'Full-chain error')} {formatValue(chainError)}</span></div>
    <div className="result-intro transform-chain-intro"><span className="eyebrow">TRANSFORM CHAIN · STEP {effectiveChainStep} / {Math.max(0, chainPath.length - 1)}</span><h2>{l('累计到', 'Accumulated to')} {frames.find((frame) => frame.id === currentChainId)?.name ?? currentChainId}</h2><p>{chainPath.map((id) => frames.find((frame) => frame.id === id)?.name ?? id).join(' → ')}. {l('右侧选择步骤，场景金色链路与下方矩阵同步推进。', 'Choose a step on the right; the gold path and matrices advance together.')}</p></div>
    <div className="equation-strip"><Formula tex={effectiveChainStep === 0 ? '{}^{W}T_W=I' : `{}^{W}T_{${texId(currentChainId)}}=${chainLinks.map((frame) => `{}^{${texId(frame.parentId ?? 'world')}}T_{${texId(frame.id)}}`).join('\\;')}`} /><span>{l('列向量约定 · 右侧局部变换先作用', 'Column-vector convention · rightmost local transform acts first')}</span></div>
    <div className="matrix-layout secondary chain-matrices">
      {chainLinks.map((frame) => <MatrixView key={frame.id} matrix={poseMatrix(frame.pose)} label={`{}^{${texId(frame.parentId ?? 'world')}}T_{${texId(frame.id)}}`} compact />)}
      <MatrixView matrix={poseMatrix(worldPoseFor(frames, currentChainId))} label={effectiveChainStep === 0 ? '{}^{W}T_W=I' : `{}^{W}T_{${texId(currentChainId)}}`} compact />
    </div>
    <details className="advanced-results">
      <summary>{l('查看旋转、四元数与逆变换', 'View rotation, quaternion & inverse')} <span>{l('展开详细校验', 'Expand validation')}</span></summary>
      <div className="readout-grid">
        <VectorReadout label="World position · m" values={T_W_selected.position.toArray()} />
        <VectorReadout label={`${eulerOrder}${eulerOrder === 'ZYX' ? ' / RPY' : ''} Euler · °`} values={euler} unit="°" />
        <VectorReadout label="Quaternion [x, y, z, w]" values={[quaternion.x, quaternion.y, quaternion.z, quaternion.w]} />
        <VectorReadout label="Axis-Angle · axis / °" values={[...axisAngle.axis.toArray(), axisAngle.angleDeg]} />
      </div>
      <div className="matrix-layout secondary">
        <MatrixView matrix={poseMatrix(T_W_selected)} label={`{}^{W}T_{${texId(selectedFrameId)}}`} compact />
        <MatrixView matrix={rotationMatrix(T_W_selected.quaternion)} size={3} label={`{}^{W}R_{${texId(selectedFrameId)}}`} compact />
        <MatrixView matrix={poseMatrix(T_selected_W)} label={`{}^{${texId(selectedFrameId)}}T_W = ({}^{W}T_{${texId(selectedFrameId)}})^{-1}`} compact />
      </div>
      <div className="validation-line"><span className={validation.valid ? 'good' : 'warn'}>● {validation.valid ? l('合法旋转矩阵', 'Valid rotation matrix') : l('旋转矩阵有误', 'Invalid rotation matrix')}</span><span>RᵀR≈I · {l('误差', 'error')} {formatValue(validation.orthogonalityError)}</span><span>det(R) = {formatValue(validation.determinant)}</span><span>‖q‖ = {formatValue(quaternion.length())}</span><span>{l('齐次末行', 'Homogeneous last row')} {validateHomogeneous(poseMatrix(T_W_selected)) ? '[0, 0, 0, 1] ✓' : l('异常', 'invalid')}</span></div>
      <div className="equation-strip"><Formula tex={`{}^{${texId(targetId)}}T_{${texId(sourceId)}} = ({}^{W}T_{${texId(targetId)}})^{-1}\\,{}^{W}T_{${texId(sourceId)}}`} /><span>{l('列向量约定：右边先作用', 'Column-vector convention: rightmost acts first')}</span></div>
    </details>
  </div>;
}

function RotationResults() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { rotationX, rotationY, rotationZ, rotationPair, rotationSense, rotationProgress } = useLabStore();
  const angles: Record<Axis, number> = { X: rotationX, Y: rotationY, Z: rotationZ };
  const { firstAxis, secondAxis, forward, reverse, stageOneProgress, stageTwoProgress } = compareRotationOrder(angles, rotationPair, rotationProgress);
  const baseVector = new Vector3(...ROTATION_DEMO_VECTOR);
  const vectorA = evaluateRotationVector(forward, baseVector, rotationSense);
  const vectorB = evaluateRotationVector(reverse, baseVector, rotationSense);
  const checkForward = validateRotation(forward), checkReverse = validateRotation(reverse);
  const factor = (axis: Axis, fraction: number) => `R_${axis.toLowerCase()}(${(angles[axis] * fraction).toFixed(1)}^\\circ)`;
  const formulaForward = `R_A(t)=${factor(firstAxis, stageTwoProgress)}${factor(secondAxis, stageOneProgress)}`;
  const formulaReverse = `R_B(t)=${factor(secondAxis, stageTwoProgress)}${factor(firstAxis, stageOneProgress)}`;
  const difference = forward.angleTo(reverse) * 180 / Math.PI;
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">ROTATION MATRIX · {Math.round(rotationProgress * 50)}%</span><h2>{l('同样两个角度，不同的执行顺序', 'Same two angles, different execution order')}</h2><p>{l(`A 先绕 ${secondAxis} 再绕 ${firstAxis}；B 先绕 ${firstAxis} 再绕 ${secondAxis}。淡色轴是固定世界方向。`, `A rotates about ${secondAxis}, then ${firstAxis}; B rotates about ${firstAxis}, then ${secondAxis}. Faded axes are fixed world directions. `)}{rotationSense === 'active' ? l('亮色轴表示物体朝向，金色向量随物体一起旋转。', 'Bright axes show object orientation, and the gold vector rotates with it. ') : l('亮色轴表示观察坐标系，金色向量保持世界方向不变。', 'Bright axes show the viewing frame, while the gold vector remains fixed in world space. ')}{l('两组原点错开放置以便对比，不参与旋转计算。', 'The two origins are offset only for comparison and do not affect the rotation math.')}</p></div>
    <div className="matrix-layout rotation-matrices current-rotation-matrices"><MatrixView matrix={rotationMatrix(forward)} size={3} label={formulaForward} /><MatrixView matrix={rotationMatrix(reverse)} size={3} label={formulaReverse} /></div>
    <div className="validation-line"><span className={checkForward.valid && checkReverse.valid ? 'good' : 'warn'}>● {checkForward.valid && checkReverse.valid ? l('两个矩阵均为合法旋转', 'Both matrices are valid rotations') : l('旋转矩阵有误', 'Invalid rotation matrix')}</span><span>RᵀR−I {l('最大误差', 'max error')} {formatValue(Math.max(checkForward.orthogonalityError, checkReverse.orthogonalityError))}</span><span>det(R) = {formatValue(checkForward.determinant)}</span><span>{difference < 1e-6 ? l('当前角度下两者重合', 'The poses coincide at these angles') : l(`两种姿态相差 ${formatValue(difference)}°`, `Pose difference: ${formatValue(difference)}°`)}</span></div>
    <div className="readout-grid"><VectorReadout label={rotationSense === 'active' ? l('A · 旋转后的世界向量', 'A · rotated world vector') : l('A · 固定向量在旋转参考系中的坐标', 'A · fixed vector in rotating frame')} values={vectorA.coordinates.toArray()} /><VectorReadout label={rotationSense === 'active' ? l('B · 旋转后的世界向量', 'B · rotated world vector') : l('B · 固定向量在旋转参考系中的坐标', 'B · fixed vector in rotating frame')} values={vectorB.coordinates.toArray()} /></div>
    <div className="equation-strip"><Formula tex={rotationSense === 'active' ? "v'_W=R(t)v_W" : "v_{local}=R(t)^T v_W"} /><span>{l('初始世界向量', 'Initial world vector')} {vectorText(baseVector)} · {rotationSense === 'active' ? l('向量转动，使用 R', 'vector rotates: use R') : l('参考系转动，使用 Rᵀ = R⁻¹', 'frame rotates: use Rᵀ = R⁻¹')}</span></div>
    <details className="advanced-results"><summary>{l('查看目标角度的单轴矩阵', 'View target single-axis matrices')} <span>Rx / Ry / Rz</span></summary>
      <div className="matrix-layout rotation-matrices single-axis">{(['X', 'Y', 'Z'] as const).map((axis) => <MatrixView key={axis} matrix={rotationMatrix(rotationAbout(axis, angles[axis]))} size={3} label={factor(axis, 1)} compact />)}</div>
    </details>
  </div>;
}

function FKResults() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const { robotAngles, eulerOrder, fkStep } = useLabStore();
  const fk = useMemo(() => forwardKinematics(robotAngles), [robotAngles]);
  const T_base_current = [null, fk.T_base_link1, fk.T_base_link2, fk.T_base_tool][fkStep];
  const finalEuler = eulerFromQuaternion(fk.T_base_tool.quaternion, eulerOrder);
  const q = fk.T_base_tool.quaternion;
  const finalMatrix = poseMatrix(fk.T_base_tool);
  const chainMatrix = poseMatrix(fk.T_base_link1).multiply(poseMatrix(fk.T_link1_link2)).multiply(poseMatrix(fk.T_link2_link3));
  const chainError = Math.max(...finalMatrix.elements.map((value, index) => Math.abs(value - chainMatrix.elements[index])));
  const fkValid = chainError < 1e-8 && validateRotation(q).valid && validateHomogeneous(finalMatrix);
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">FORWARD KINEMATICS</span><h2>{l('关节角 → 末端位姿', 'Joint angles → end-effector pose')}</h2><p>{l('当前显示教学用 3 关节运动学骨架；每个关节改变后，下游坐标轴与矩阵链实时重算。', 'This teaching view shows a 3-joint kinematic skeleton. Each joint change recomputes downstream axes and the matrix chain in real time.')}</p></div>
    <div className="equation-strip"><Formula tex="{}^{\mathrm{base}}T_{\mathrm{tool}} = {}^{\mathrm{base}}T_{\mathrm{link1}}\;{}^{\mathrm{link1}}T_{\mathrm{link2}}\;{}^{\mathrm{link2}}T_{\mathrm{tool}}" /><span>q1 → q2 → q3</span></div>
    <div className="matrix-layout"><MatrixView matrix={finalMatrix} label="{}^{\mathrm{base}}T_{\mathrm{tool}}" />
      <MatrixView matrix={rotationMatrix(fk.T_base_tool.quaternion)} size={3} label="{}^{\mathrm{base}}R_{\mathrm{tool}}" /></div>
    <div className="readout-grid"><VectorReadout label={l('末端位置 [x, y, z] · m', 'End-effector position [x, y, z] · m')} values={fk.T_base_tool.position.toArray()} /><VectorReadout label={`${eulerOrder}${eulerOrder === 'ZYX' ? ' / RPY' : ''} Euler · °`} values={finalEuler} unit="°" />
      <VectorReadout label="Quaternion [x, y, z, w]" values={[q.x, q.y, q.z, q.w]} /></div>
    <div className="validation-line"><span className={fkValid ? 'good' : 'warn'}>● {fkValid ? l('FK 矩阵链有效', 'FK matrix chain valid') : l('FK 矩阵链异常', 'FK matrix chain invalid')}</span><span>{l('连乘误差', 'Chain error')} {formatValue(chainError)}</span><span>‖q‖ = {formatValue(q.length())}</span><span>{l('齐次末行', 'Homogeneous last row')} {validateHomogeneous(finalMatrix) ? '[0, 0, 0, 1] ✓' : l('异常', 'invalid')}</span></div>
    {T_base_current && <div className="equation-strip"><Formula tex={`{}^{\\mathrm{base}}T_{\\mathrm{${fkStep === 3 ? 'tool' : `link${fkStep}`}}}`} /><span>{l('当前逐级结果', 'Current step result')} · {vectorText(T_base_current.position)}</span></div>}
    <div className="matrix-layout secondary"><MatrixView matrix={poseMatrix(fk.T_base_link1)} label="{}^{\mathrm{base}}T_{\mathrm{link1}}" compact /><MatrixView matrix={poseMatrix(fk.T_link1_link2)} label="{}^{\mathrm{link1}}T_{\mathrm{link2}}" compact /><MatrixView matrix={poseMatrix(fk.T_link2_link3)} label="{}^{\mathrm{link2}}T_{\mathrm{tool}}" compact /></div>
  </div>;
}

export default function App() {
  const language = useLocaleStore((state) => state.language);
  const toggleLanguage = useLocaleStore((state) => state.toggleLanguage);
  const l = (zh: string, en: string) => localize(language, zh, en);
  const mode = useLabStore((state) => state.mode);
  const setMode = useLabStore((state) => state.setMode);
  const reset = useLabStore((state) => state.reset);
  useEffect(() => { document.documentElement.lang = language === 'zh' ? 'zh-CN' : 'en'; }, [language]);
  return <div className="app-shell">
    <header className="topbar"><div className="brand"><span className="brand-mark">⌖</span><span>{l('机器人学 · 坐标实验室', 'Robotics · Coordinate Lab')}</span><small>ROBOTICS LAB</small></div>
      <nav className="mode-nav" aria-label={l('学习模块', 'Learning modules')}>{NAV.map((item, index) => <button key={item.id} className={mode === item.id ? 'active' : ''} onClick={() => setMode(item.id)}><small>0{index + 1}</small>{item.label[language]}</button>)}</nav>
      <div className="topbar-actions"><button className="language-button" onClick={toggleLanguage} aria-label={l('切换为英文', 'Switch to Chinese')} title={l('切换为英文', 'Switch to Chinese')}>{language === 'zh' ? 'EN' : '中文'}</button><button className="reset-button" onClick={reset} title={l('重置全部场景', 'Reset all scenes')}>{l('重置', 'Reset')}</button></div></header>
    <main className="workspace">
      <section className="visual-workspace"><div className="scene-heading"><div><span className="eyebrow">INTERACTIVE 3D</span><h1>{navLabel(mode, language)}</h1></div><span className="heading-note">{mode === 'ik' ? l('拖动目标点，用 Jacobian 逐步逼近', 'Drag the target and iterate with the Jacobian') : mode === 'pinocchio' ? l('沿数据流查看 URDF、FK、SE(3) 与可视化', 'Follow the flow from URDF and FK to SE(3) and rendering') : mode === 'fk' ? l('拖动关节角，看连杆与矩阵一起运动', 'Adjust joints and watch links and matrices move together') : mode === 'rotation' ? l('调节 Rx、Ry、Rz，对比旋转次序与主动 / 被动视角', 'Adjust Rx, Ry, Rz and compare order plus active / passive views') : l('拖动坐标系与点，查看相对变换和父子矩阵链', 'Drag frames and points to inspect relative transforms and parent-child chains')}</span></div><Scene />
        {mode === 'rotation' ? <RotationResults /> : mode === 'fk' ? <FKResults /> : mode === 'ik' ? <IKResults /> : mode === 'pinocchio' ? <PinocchioResults /> : <FrameResults />}
      </section>
      <aside className="control-panel" aria-label={l('实验操作栏', 'Experiment controls')}><div className="control-heading"><span className="eyebrow">CONTROLS</span><strong>{l('实验操作', 'Experiment controls')}</strong><span>{NAV.find((item) => item.id === mode)?.short}</span></div>
        {mode === 'rotation' ? <RotationControls /> : mode === 'fk' ? <FKControls /> : mode === 'ik' ? <IKControls /> : mode === 'pinocchio' ? <PinocchioControls /> : <FrameControls />}
      </aside>
    </main>
  </div>;
}
