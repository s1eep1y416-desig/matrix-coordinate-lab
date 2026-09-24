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
import { useLabStore, worldPoseFor, relativeFramePose, type DofKey, type FrameNode, type LabMode } from './stores/labStore';
import 'katex/dist/katex.min.css';
import './styles.css';

const NAV: { id: LabMode; label: string; short: string }[] = [
  { id: 'frames', label: '坐标系与变换', short: 'Frames + Transforms' },
  { id: 'rotation', label: '旋转矩阵', short: 'Rotation' },
  { id: 'fk', label: '3-Link FK', short: 'Kinematics' },
  { id: 'ik', label: '逆运动学', short: 'IK Solver' },
  { id: 'pinocchio', label: 'Pinocchio', short: 'Workflow' },
];
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
  const { frames, selectedFrameId, selectFrame, addFrame, toggleFrame, deleteFrame } = useLabStore();
  return <section className="control-card">
    <PaneTitle eyebrow="TF TREE" title="坐标系层级" aside={<button className="small-dark-button" onClick={() => addFrame()} disabled={frames.length >= 8}>＋ 添加子系</button>} />
    <div className="frame-tree">
      {frames.map((frame) => <div className={`frame-row ${selectedFrameId === frame.id ? 'selected' : ''}`} key={frame.id}>
        <button className="frame-select" style={{ paddingLeft: `${12 + frameDepth(frames, frame.id) * 13}px` }} onClick={() => selectFrame(frame.id)}>
          <span className={`frame-glyph ${frame.id === 'world' ? 'world' : ''}`} />
          <span>{frame.name}</span><small>{frame.id === 'world' ? 'root' : `↳ ${frame.parentId}`}</small>
        </button>
        {frame.id !== 'world' && <button className="frame-eye" aria-label={`${frame.visible ? '隐藏' : '显示'} ${frame.name}`} title={frame.visible ? '隐藏坐标轴' : '显示坐标轴'} onClick={() => toggleFrame(frame.id)}>{frame.visible ? '◉' : '○'}</button>}
        {frame.id !== 'world' && frame.id !== 'A' && !frames.some((child) => child.parentId === frame.id) && <button className="frame-remove" aria-label={`删除 ${frame.name}`} onClick={() => deleteFrame(frame.id)}>×</button>}
      </div>)}
    </div>
    <p className="fine-print">选中坐标系，再在 3D 场景中拖动其原点。子坐标系的位置与姿态始终相对于父级。</p>
  </section>;
}

function FramePoseControls({ selected }: { selected: FrameNode }) {
  const { frames, eulerOrder, eulerOrderBehavior, setEulerOrder, setEulerOrderBehavior, setFramePosition, setFrameEuler, reparentFrame } = useLabStore();
  const euler = eulerFromQuaternion(selected.pose.quaternion, eulerOrder);
  const editable = selected.id !== 'world';
  return <section className="control-card">
    <PaneTitle eyebrow="POSE / ^P T_CHILD" title={`${selected.name} 的局部位姿`} />
    {editable && <label className="select-label">父坐标系
      <select value={selected.parentId ?? 'world'} onChange={(event) => reparentFrame(selected.id, event.target.value)}>
        {frames.filter((candidate) => candidate.id !== selected.id && !isDescendant(frames, candidate.id, selected.id)).map((candidate) => <option key={candidate.id} value={candidate.id}>{candidate.name}</option>)}
      </select>
    </label>}
    <div className="field-heading"><strong>平移</strong><span>相对于父坐标系 · m</span></div>
    <div className="three-fields">
      {labels.map((axis, index) => <NumberField key={axis} id={`position-${axis}`} label={axis} unit="m" disabled={!editable || selected.constraints[dofKeys[index]].locked}
        value={selected.pose.position.getComponent(index)} min={selected.constraints[dofKeys[index]].min} max={selected.constraints[dofKeys[index]].max}
        onCommit={(value) => setFramePosition(selected.id, index as 0 | 1 | 2, value)} />)}
    </div>
    <div className="field-heading"><strong>旋转 · {eulerOrder === 'ZYX' ? 'RPY' : `${eulerOrder} Euler`}</strong><span>左右拖动数值 · °</span></div>
    <div className="three-fields">
      {(['roll', 'pitch', 'yaw'] as const).map((axis, index) => <NumberField key={axis} id={`euler-${axis}`} label={axis} unit="°" scrub step={0.5}
        disabled={!editable || selected.constraints[dofKeys[index + 3]].locked} value={euler[index]}
        min={selected.constraints[dofKeys[index + 3]].min} max={selected.constraints[dofKeys[index + 3]].max}
        onCommit={(value) => setFrameEuler(selected.id, index as 0 | 1 | 2, value)} />)}
    </div>
    {editable && <div className="frame-presets" aria-label="旋转预设">
      <button onClick={() => ([0, 0, 45] as const).forEach((value, index) => setFrameEuler(selected.id, index as 0 | 1 | 2, value))}>绕 Z 45°</button>
      <button onClick={() => ([25, -20, 35] as const).forEach((value, index) => setFrameEuler(selected.id, index as 0 | 1 | 2, value))}>空间倾斜</button>
      <button onClick={() => ([0, 0, 0] as const).forEach((value, index) => setFrameEuler(selected.id, index as 0 | 1 | 2, value))}>清零旋转</button>
    </div>}
    <label className="select-label">欧拉角顺序
      <select value={eulerOrder} onChange={(event) => setEulerOrder(event.target.value as EulerOrder)}>
        {EULER_ORDERS.map((order) => <option key={order}>{order}</option>)}
      </select>
    </label>
    <div className="segmented order-behavior"><button className={eulerOrderBehavior === 'pose' ? 'active' : ''} onClick={() => setEulerOrderBehavior('pose')}>保留姿态</button><button className={eulerOrderBehavior === 'angles' ? 'active' : ''} onClick={() => setEulerOrderBehavior('angles')}>保留角度</button></div>
    <p className="fine-print">{eulerOrderBehavior === 'pose' ? '切换顺序只改变角度读数，空间姿态不变。' : '切换顺序时保留选中坐标系的三个角度数值，空间姿态会改变。'}ZYX / RPY 对应 <Formula tex="R_z R_y R_x" />。</p>
    {isNearGimbalLock(euler, eulerOrder) && <div className="teaching-alert">Gimbal Lock：中间轴接近 ±90°，第一轴与第三轴的转动方向重合。四元数表示仍保持有效。</div>}
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
  const setConstraint = useLabStore((state) => state.setConstraint);
  if (selected.id === 'world') return null;
  return <details className="control-card dof-card">
    <summary><span><span className="eyebrow">JOINT-LIKE LIMITS</span><strong>自由度与限位</strong></span><small>{dofKeys.filter((key) => !selected.constraints[key].locked).length} / 6 可动 ▾</small></summary>
    <p className="fine-print">每个坐标系分别设置。锁定后当前值固定；修改边界时，姿态会落到新范围内。</p>
    <div className="dof-table">
      {dofKeys.map((key) => {
        const rule = selected.constraints[key];
        return <div className="dof-row" key={key}>
          <strong>{key[0] === 't' ? key[1].toUpperCase() : `R${key[1]}`}</strong>
          <label className="lock-toggle"><input type="checkbox" checked={rule.locked} onChange={(event) => setConstraint(selected.id, key, { locked: event.target.checked })} /><span>锁</span></label>
          <NumberField label="下限" value={rule.min} onCommit={(value) => { setConstraint(selected.id, key, { min: value }); return useLabStore.getState().frames.find((frame) => frame.id === selected.id)!.constraints[key].min; }} />
          <NumberField label="上限" value={rule.max} onCommit={(value) => { setConstraint(selected.id, key, { max: value }); return useLabStore.getState().frames.find((frame) => frame.id === selected.id)!.constraints[key].max; }} />
        </div>;
      })}
    </div>
  </details>;
}

function PointControls({ selected }: { selected: FrameNode }) {
  const { frames, pointWorld, pointReference, setPointReference, setPointCoordinate } = useLabStore();
  const frameId = selected.id === 'world' ? 'A' : selected.id;
  const T_W_F = worldPoseFor(frames, frameId);
  const pFrame = inverseTransformPoint(T_W_F, pointWorld);
  const referenceId = pointReference === 'world' ? 'world' : frameId;
  const coordinates = pointReference === 'world' ? pointWorld : pFrame;
  return <section className="control-card">
    <PaneTitle eyebrow="SAME POINT · TWO DESCRIPTIONS" title="空间点 P" />
    <div className="segmented"><button className={pointReference === 'world' ? 'active' : ''} onClick={() => setPointReference('world')}>在 World 中</button><button className={pointReference === 'frame' ? 'active' : ''} onClick={() => setPointReference('frame')}>在 {frameId} 中</button></div>
    <div className="three-fields">{labels.map((axis, index) => <NumberField key={axis} label={axis} unit="m" value={coordinates.getComponent(index)} onCommit={(value) => setPointCoordinate(referenceId, index as 0 | 1 | 2, value)} />)}</div>
    <div className="point-comparison"><span><b>World</b>{vectorText(pointWorld)}</span><span><b>{frameId}</b>{vectorText(pFrame)}</span></div>
    <p className="fine-print">切换参考系不会移动点 P，只会改变描述它的坐标。<Formula tex={`p_W = {}^{W}T_{${texId(frameId)}}\\,p_{${texId(frameId)}}`} /></p>
  </section>;
}

function QuaternionControls({ selected }: { selected: FrameNode }) {
  const setFrameQuaternion = useLabStore((state) => state.setFrameQuaternion);
  const [draft, setDraft] = useState<[number, number, number, number]>([selected.pose.quaternion.x, selected.pose.quaternion.y, selected.pose.quaternion.z, selected.pose.quaternion.w]);
  useEffect(() => setDraft([selected.pose.quaternion.x, selected.pose.quaternion.y, selected.pose.quaternion.z, selected.pose.quaternion.w]), [selected.id, selected.pose.quaternion]);
  const norm = Math.hypot(...draft);
  const apply = () => setFrameQuaternion(selected.id, new Quaternion(...draft));
  return <section className="control-card">
    <PaneTitle eyebrow="ORIENTATION" title="四元数 q" aside={<span className="subtle-badge">ROS2: x y z w</span>} />
    <div className="four-fields">{(['x', 'y', 'z', 'w'] as const).map((axis, index) => <NumberField key={`${selected.id}-${axis}`} label={axis} value={draft[index]} disabled={selected.id === 'world'} onCommit={(value) => { setDraft((current) => current.map((n, i) => i === index ? value : n) as [number, number, number, number]); return value; }} />)}</div>
    <div className="inline-result"><span>输入范数</span><strong className={Math.abs(norm - 1) < 1e-6 ? 'good' : 'warn'}>{formatValue(norm)}</strong><button className="small-dark-button" disabled={selected.id === 'world' || norm < 1e-12} onClick={apply}>应用并归一化</button></div>
    <p className="fine-print">内部姿态始终使用单位四元数。<Formula tex="q\text{ 与 }-q" /> 表示同一个方向；教材也可能使用 [w, x, y, z] 顺序。</p>
  </section>;
}

function FrameControls() {
  const frames = useLabStore((state) => state.frames);
  const selectedFrameId = useLabStore((state) => state.selectedFrameId);
  const selected = frames.find((frame) => frame.id === selectedFrameId) ?? frames[0];
  return <><FrameTree /><TransformControls /><FramePoseControls selected={selected} /><PointControls selected={selected} /><DofControls selected={selected} /><QuaternionControls selected={selected} /></>;
}

function RotationControls() {
  const { rotationX, rotationY, rotationZ, rotationPair, setRotationPair, setRotationDemo, rotationSense, setRotationSense } = useLabStore();
  const angles: Record<Axis, number> = { X: rotationX, Y: rotationY, Z: rotationZ };
  const control = (axis: Axis) => <div className="slider-row" key={axis}>
    <strong>R{axis}</strong><input aria-label={`R${axis} 角度`} type="range" min="-180" max="180" step="1" value={angles[axis]} onChange={(event) => setRotationDemo(axis, Number(event.target.value))} />
    <NumberField label={axis} unit="°" value={angles[axis]} scrub step={0.5} onCommit={(number) => { setRotationDemo(axis, number); return Math.max(-180, Math.min(180, number)); }} />
  </div>;
  const [firstAxis, secondAxis] = rotationPair;
  return <>
    <section className="control-card"><PaneTitle eyebrow="SINGLE-AXIS ROTATION" title="绕 X / Y / Z 旋转" />{(['X', 'Y', 'Z'] as const).map(control)}
      <p className="fine-print">三个目标角度独立调节；展开「单轴矩阵」可查看 <Formula tex="R_x,\;R_y,\;R_z" />。播放时，A / B 矩阵显示当前进度的旋转。</p>
      <p className="fine-print">右手系 +90°：Rx 将 +Y 转向 +Z，Ry 将 +Z 转向 +X，Rz 将 +X 转向 +Y。−90° 的方向相反。</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="ORDER MATTERS" title="选两个轴比较次序" />
      <div className="rotation-pair-buttons">{(['XY', 'YZ', 'ZX'] as RotationPair[]).map((pair) => <button key={pair} className={rotationPair === pair ? 'active' : ''} onClick={() => setRotationPair(pair)} aria-pressed={rotationPair === pair}>{pair[0]} / {pair[1]}</button>)}</div>
      <p className="fine-print">当前比较 <Formula tex={`R_${firstAxis.toLowerCase()}R_${secondAxis.toLowerCase()}`} /> 与 <Formula tex={`R_${secondAxis.toLowerCase()}R_${firstAxis.toLowerCase()}`} />。采用列向量：右侧矩阵先作用；场景两侧的彩色坐标轴与下方矩阵完全对应。</p>
    </section>
    <RotationPlayback />
    <section className="control-card"><PaneTitle eyebrow="ACTIVE / PASSIVE" title="旋转物体，还是改变描述？" />
      <div className="segmented"><button className={rotationSense === 'active' ? 'active' : ''} onClick={() => setRotationSense('active')}>Active · 物体转</button><button className={rotationSense === 'passive' ? 'active' : ''} onClick={() => setRotationSense('passive')}>Passive · 坐标系转</button></div>
      <p className="fine-print">{rotationSense === 'active' ? '参考系固定，世界向量随旋转改变。' : '空间向量固定，变的是观察它的参考坐标系；向量的世界位置不变。'}</p>
    </section>
  </>;
}

function TransformControls() {
  const { frames, sourceId, targetId, setSource, setTarget } = useLabStore();
  return <section className="control-card"><PaneTitle eyebrow="SOURCE → TARGET" title="坐标系选择" />
    <div className="two-selects"><label className="select-label">源坐标系<select value={sourceId} onChange={(event) => setSource(event.target.value)}>{frames.map((frame) => <option key={frame.id} value={frame.id}>{frame.name}</option>)}</select></label>
      <label className="select-label">目标坐标系<select value={targetId} onChange={(event) => setTarget(event.target.value)}>{frames.map((frame) => <option key={frame.id} value={frame.id}>{frame.name}</option>)}</select></label></div>
    <p className="fine-print">同一个点 P 保持在原处；切换源与目标后，读数和 <Formula tex={`{}^{${texId(targetId)}}T_{${texId(sourceId)}}`} /> 同步变化。</p>
  </section>;
}

function FKControls() {
  const { robotAngles, setRobotAngle, fkStep, setFkStep } = useLabStore();
  return <>
    <section className="control-card"><PaneTitle eyebrow="3 REVOLUTE JOINTS" title="关节角度" />
      {robotAngles.map((angle, index) => <div className="slider-row" key={index}><strong>q{index + 1}</strong>
        <input aria-label={`关节 ${index + 1} 角度`} type="range" min="-180" max="180" step="1" value={angle} onChange={(event) => setRobotAngle(index as 0 | 1 | 2, Number(event.target.value))} />
        <NumberField label="角度" unit="°" value={angle} onCommit={(number) => { setRobotAngle(index as 0 | 1 | 2, number); return Math.max(-180, Math.min(180, number)); }} /></div>)}
      <p className="fine-print">J1 绕 Z，J2 与 J3 绕各自局部 Y。每段 link 沿自身 +X 伸出，长度依次为 1.25、1.00、0.80 m。</p>
      <p className="fine-print">正角遵循右手定则：零位时，J1 正转朝 +Y；J2、J3 正转把其下游 +X 方向转向 −Z。</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="CHAIN REPLAY" title="逐级查看矩阵链" />
      <div className="step-buttons">{['base', 'J1', 'J2', 'tool'].map((label, index) => <button key={label} className={fkStep === index ? 'active' : ''} onClick={() => setFkStep(index)}>{label}</button>)}</div>
      <p className="fine-print">选择一步，场景只显示计算到该级的运动学骨架和坐标系；真实外形将在导入你的 URDF 后替换。</p>
    </section>
  </>;
}

function IKControls() {
  const { robotAngles, setRobotAngle, ikTarget, setIkTargetCoordinate, ikDamping, setIkDamping, stepIK, solveIK, ikIterations } = useLabStore();
  return <>
    <section className="control-card"><PaneTitle eyebrow="TARGET IN BASE" title="拖动或输入目标点" aside={<span className="subtle-badge">position IK</span>} />
      <div className="three-fields">{labels.map((axis, index) => <NumberField key={axis} label={axis} unit="m" value={ikTarget.getComponent(index)} min={-3.2} max={3.2} onCommit={(value) => setIkTargetCoordinate(index as 0 | 1 | 2, value)} />)}</div>
      <div className="preset-row"><button onClick={() => useLabStore.getState().setIkTarget(new Vector3(1.9, .8, .7))}>目标 A</button><button onClick={() => useLabStore.getState().setIkTarget(new Vector3(1.25, -1.1, -.55))}>目标 B</button><button onClick={() => useLabStore.getState().setIkTarget(new Vector3(3.4, 0, 0))}>不可达点</button></div>
      <p className="fine-print">左键可直接拖动绿色 Target；球壳外的目标无法收敛，用于观察残差和可达性。</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="DAMPED LEAST SQUARES" title="Jacobian 迭代" />
      <div className="slider-row"><strong>λ</strong><input aria-label="阻尼系数" type="range" min="0.001" max="0.5" step="0.001" value={ikDamping} onChange={(event) => setIkDamping(Number(event.target.value))} /><NumberField label="阻尼" value={ikDamping} onCommit={(value) => { setIkDamping(value); return Math.max(.001, Math.min(1, value)); }} /></div>
      <div className="solver-actions"><button onClick={stepIK}>单步迭代</button><button className="primary-action" onClick={solveIK}>求解到收敛</button></div>
      <div className="inline-result"><span>累计迭代</span><strong>{ikIterations}</strong></div>
      <p className="fine-print"><Formula tex="\Delta q=J^T(JJ^T+\lambda^2I)^{-1}e" />。阻尼抑制奇异位形附近的关节跳变；线搜索只接受让误差下降的步长。</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="INITIAL / CURRENT q" title="关节角" />
      {robotAngles.map((angle, index) => <div className="slider-row" key={index}><strong>q{index + 1}</strong><input aria-label={`IK 关节 ${index + 1}`} type="range" min="-180" max="180" value={angle} onChange={(event) => setRobotAngle(index as 0 | 1 | 2, Number(event.target.value))} /><NumberField label="角度" unit="°" value={angle} onCommit={(value) => { setRobotAngle(index as 0 | 1 | 2, value); return Math.max(-180, Math.min(180, value)); }} /></div>)}
    </section>
  </>;
}

const PIN_STEPS = [
  ['01', '读取 URDF', 'buildModelFromUrdf() 建立关节、Link 与 Frame 拓扑'],
  ['02', '建立数据', 'model.createData() 分配运动学计算缓存'],
  ['03', '输入 q', '关节角从控制器进入模型，单位统一为 rad'],
  ['04', '执行 FK', 'forwardKinematics(model, data, q) 递推各 Link 位姿'],
  ['05', '更新 Frame', 'updateFramePlacements(model, data) 得到 data.oMf'],
  ['06', '输出与显示', '读取 end_link 的 SE(3)，同步末端数值与 3D 场景'],
] as const;

function PinocchioControls() {
  const { pinStep, setPinStep, robotAngles, setRobotAngle } = useLabStore();
  return <>
    <section className="control-card"><PaneTitle eyebrow="DOCUMENT WORKFLOW" title="Pinocchio FK 数据流" />
      <div className="pin-step-list">{PIN_STEPS.map((step, index) => <button key={step[0]} className={pinStep === index ? 'active' : pinStep > index ? 'done' : ''} onClick={() => setPinStep(index)}><span>{step[0]}</span><strong>{step[1]}</strong><small>{step[2]}</small></button>)}</div>
      <p className="fine-print">逐步点击，观察“关节角 → Pinocchio FK → 末端位姿 + 可视化”的数据怎样流动。</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="q · degrees in UI" title="输入关节配置" />
      <div className="three-fields">{robotAngles.map((angle, index) => <NumberField key={index} label={`q${index + 1}`} unit="°" scrub step={.5} value={angle} onCommit={(value) => { setRobotAngle(index as 0 | 1 | 2, value); return Math.max(-180, Math.min(180, value)); }} />)}</div>
      <button className="wide-action" onClick={() => { setRobotAngle(0, 45); setRobotAngle(1, -30); setRobotAngle(2, 15); setPinStep(5); }}>载入文档示例 45, −30, 15</button>
      <p className="fine-print">界面便于学习而使用度；传入 Pinocchio 前转换为弧度。文档的 6 轴示例还包含 q4–q6，本实验先展示相同的 3 轴核心链路。</p>
    </section>
  </>;
}

function matrixFromJacobian(rows: ReturnType<typeof positionJacobian>): Matrix4 {
  return new Matrix4().set(rows[0][0], rows[0][1], rows[0][2], 0, rows[1][0], rows[1][1], rows[1][2], 0, rows[2][0], rows[2][1], rows[2][2], 0, 0, 0, 0, 1);
}

function IKResults() {
  const { robotAngles, ikTarget, ikIterations } = useLabStore();
  const fk = useMemo(() => forwardKinematics(robotAngles), [robotAngles]);
  const jacobian = positionJacobian(robotAngles);
  const error = ikTarget.clone().sub(fk.T_base_tool.position);
  const errorNorm = error.length();
  const reachable = ikTarget.length() <= 3.05 + 1e-8;
  const manipulability = jacobianManipulability(jacobian);
  const converged = errorNorm < 1e-3;
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">INVERSE KINEMATICS · POSITION</span><h2>目标位置 → 关节角</h2><p>当前是教学用 3 关节运动学骨架。绿色点是目标，金色线是当前解；误差线会随每次 Jacobian 迭代缩短。</p></div>
    <div className="equation-strip"><Formula tex="e=p_{target}-p(q),\quad \Delta q=J^T(JJ^T+\lambda^2I)^{-1}e" /><span>DLS + 下降线搜索</span></div>
    <div className="readout-grid"><VectorReadout label="目标位置 · m" values={ikTarget.toArray()} /><VectorReadout label="当前末端 · m" values={fk.T_base_tool.position.toArray()} /><VectorReadout label="Cartesian error · m" values={error.toArray()} /><VectorReadout label="当前关节角 · °" values={robotAngles} unit="°" /></div>
    <div className="validation-line"><span className={converged ? 'good' : 'warn'}>● {converged ? '已收敛' : reachable ? '等待迭代' : '目标超出最大臂展'}</span><span>‖e‖ = {formatValue(errorNorm)} m</span><span>|det(J)| = {formatValue(manipulability)}</span><span>迭代 {ikIterations}</span></div>
    <div className="matrix-layout"><MatrixView matrix={matrixFromJacobian(jacobian)} size={3} label="J_v(q)" /><MatrixView matrix={poseMatrix(fk.T_base_tool)} label="{}^{base}T_{tool}(q)" /></div>
    <div className="teaching-band"><strong>为什么这里只有位置 IK？</strong><span>3 个关节只有 3 个自由度，可用 3×3 的位置 Jacobian 匹配 x/y/z；文档中的 6 轴 reBot 使用 <Formula tex="\log_6(T_{current}^{-1}T_{target})" /> 得到旋转 + 平移的 6D SE(3) 误差。</span></div>
  </div>;
}

function PinocchioResults() {
  const { robotAngles, pinStep, eulerOrder } = useLabStore();
  const fk = useMemo(() => forwardKinematics(robotAngles), [robotAngles]);
  const q = fk.T_base_tool.quaternion;
  const euler = eulerFromQuaternion(q, eulerOrder);
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">PINOCCHIO × URDF × SE(3)</span><h2>{PIN_STEPS[pinStep][1]}</h2><p>{PIN_STEPS[pinStep][2]}</p></div>
    <div className="pin-pipeline">{PIN_STEPS.map((step, index) => <button key={step[0]} className={index === pinStep ? 'active' : index < pinStep ? 'done' : ''} onClick={() => useLabStore.getState().setPinStep(index)}><span>{step[0]}</span><strong>{step[1]}</strong></button>)}</div>
    <div className="equation-strip"><Formula tex="q\;\longrightarrow\;\mathrm{FK}\;\longrightarrow\;{}^{world}T_{end\_link}\in SE(3)" /><span>数学结果与 3D 场景共用同一份 Pose</span></div>
    <div className="matrix-layout"><MatrixView matrix={poseMatrix(fk.T_base_tool)} label="data.oMf[\mathrm{end\_link}]" /><MatrixView matrix={rotationMatrix(q)} size={3} label="R_{end\_link}" /></div>
    <div className="readout-grid"><VectorReadout label="q · rad (Pinocchio 输入)" values={robotAngles.map((value) => value * Math.PI / 180)} /><VectorReadout label="translation · m" values={fk.T_base_tool.position.toArray()} /><VectorReadout label={`${eulerOrder} Euler · °`} values={euler} unit="°" /><VectorReadout label="Quaternion [x, y, z, w]" values={[q.x, q.y, q.z, q.w]} /></div>
    <div className="api-map"><div><code>pin.buildModelFromUrdf()</code><span>URDF → Model</span></div><div><code>pin.forwardKinematics()</code><span>q → Link placements</span></div><div><code>pin.updateFramePlacements()</code><span>Link → Frame placements</span></div><div><code>data.oMf[frame_id]</code><span>读取世界到末端的 SE(3)</span></div></div>
    <div className="teaching-band"><strong>与文档示例的关系</strong><span>本页在浏览器中复现同一套数学与数据流；真实 Python 工程由 Pinocchio 计算，MeshCat 负责显示。下一步接入 reBot URDF 后，可把当前 3-Link 模型替换为 6 轴模型，并让 IK 同时求位置与方向。</span></div>
  </div>;
}

function FrameResults() {
  const { frames, selectedFrameId, sourceId, targetId, eulerOrder, pointWorld } = useLabStore();
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
  const chainFrame = frames.find((frame) => frame.id === selectedFrameId && frame.id !== 'world');
  const chainParentId = chainFrame?.parentId ?? 'world';
  const relativeMatrix = poseMatrix(T_target_source);
  const independentRelativeMatrix = poseMatrix(T_W_target).invert().multiply(poseMatrix(T_W_source));
  const relativeError = matrixMaxError(relativeMatrix, independentRelativeMatrix);
  const inverseError = matrixMaxError(poseMatrix(T_selected_W), poseMatrix(T_W_selected).invert());
  const pointError = pSource.clone().applyMatrix4(independentRelativeMatrix).distanceTo(pTarget);
  const chainError = chainFrame ? matrixMaxError(
    poseMatrix(worldPoseFor(frames, chainFrame.id)),
    poseMatrix(worldPoseFor(frames, chainParentId)).multiply(poseMatrix(chainFrame.pose)),
  ) : 0;
  const maxError = Math.max(relativeError, inverseError, pointError, chainError);
  const matrixValid = maxError < 1e-8 && validation.valid && validateHomogeneous(relativeMatrix);
  return <div className="results-stack">
    <div className="conversion-strip" aria-live="polite">
      <div className="conversion-step"><span>源坐标系 · {sourceId === 'world' ? 'World' : `Frame ${sourceId}`}</span><strong>{compactVectorText(pSource)}</strong></div>
      <span className="conversion-arrow">→</span>
      <div className="conversion-step"><span>世界坐标 · World</span><strong>{compactVectorText(pointWorld)}</strong></div>
      <span className="conversion-arrow">→</span>
      <div className="conversion-step result"><span>目标坐标系 · {targetId === 'world' ? 'World' : `Frame ${targetId}`}</span><strong>{compactVectorText(pTarget)}</strong></div>
    </div>
    <div className="conversion-formula"><Formula tex={`\\tilde p_{${texId(targetId)}} = {}^{${texId(targetId)}}T_{${texId(sourceId)}}\\,\\tilde p_{${texId(sourceId)}}`} /><span>点 P 的空间位置固定 · 齐次坐标 w = 1</span></div>
    <div className="matrix-layout core-matrices">
      <MatrixView matrix={relativeMatrix} label={`{}^{${texId(targetId)}}T_{${texId(sourceId)}}`} />
    </div>
    <div className="validation-line"><span className={matrixValid ? 'good' : 'warn'}>● {matrixValid ? '矩阵换算校验通过' : '矩阵换算异常'}</span><span>相对变换误差 {formatValue(relativeError)}</span><span>逆矩阵误差 {formatValue(inverseError)}</span><span>点换算误差 {formatValue(pointError)}</span>{chainFrame && <span>父子连乘误差 {formatValue(chainError)}</span>}</div>
    {chainFrame && <div className="result-intro transform-chain-intro"><span className="eyebrow">TRANSFORM CHAIN · 当前选中 {chainFrame.name}</span><h2>父子坐标系逐级连乘</h2><p>选中任意子坐标系，下方都会把「World 到父级」与「父级到子级」相乘，得到该坐标系相对于 World 的位姿。</p></div>}
    {chainFrame && <div className="equation-strip"><Formula tex={`{}^{W}T_{${texId(chainParentId)}}\\;{}^{${texId(chainParentId)}}T_{${texId(chainFrame.id)}} = {}^{W}T_{${texId(chainFrame.id)}}`} /><span>列向量约定 · 右侧局部变换先作用</span></div>}
    {chainFrame && <div className="matrix-layout secondary">
      <MatrixView matrix={poseMatrix(worldPoseFor(frames, chainParentId))} label={`{}^{W}T_{${texId(chainParentId)}}`} compact />
      <MatrixView matrix={poseMatrix(chainFrame.pose)} label={`{}^{${texId(chainParentId)}}T_{${texId(chainFrame.id)}}`} compact />
      <MatrixView matrix={poseMatrix(worldPoseFor(frames, chainFrame.id))} label={`{}^{W}T_{${texId(chainFrame.id)}}`} compact />
    </div>}
    <details className="advanced-results">
      <summary>查看旋转、四元数与逆变换 <span>展开详细校验</span></summary>
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
      <div className="validation-line"><span className={validation.valid ? 'good' : 'warn'}>● {validation.valid ? '合法旋转矩阵' : '旋转矩阵有误'}</span><span>RᵀR≈I · 误差 {formatValue(validation.orthogonalityError)}</span><span>det(R) = {formatValue(validation.determinant)}</span><span>‖q‖ = {formatValue(quaternion.length())}</span><span>齐次末行 {validateHomogeneous(poseMatrix(T_W_selected)) ? '[0, 0, 0, 1] ✓' : '异常'}</span></div>
      <div className="equation-strip"><Formula tex={`{}^{${texId(targetId)}}T_{${texId(sourceId)}} = ({}^{W}T_{${texId(targetId)}})^{-1}\\,{}^{W}T_{${texId(sourceId)}}`} /><span>列向量约定：右边先作用</span></div>
    </details>
  </div>;
}

function RotationResults() {
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
    <div className="result-intro"><span className="eyebrow">ROTATION MATRIX · {Math.round(rotationProgress * 50)}%</span><h2>同样两个角度，不同的执行顺序</h2><p>A 先绕 {secondAxis} 再绕 {firstAxis}；B 先绕 {firstAxis} 再绕 {secondAxis}。淡色轴是固定世界方向。{rotationSense === 'active' ? '亮色轴表示物体朝向，金色向量随物体一起旋转。' : '亮色轴表示观察坐标系，金色向量保持世界方向不变。'}两组原点错开放置以便对比，不参与旋转计算。</p></div>
    <div className="matrix-layout rotation-matrices current-rotation-matrices"><MatrixView matrix={rotationMatrix(forward)} size={3} label={formulaForward} /><MatrixView matrix={rotationMatrix(reverse)} size={3} label={formulaReverse} /></div>
    <div className="validation-line"><span className={checkForward.valid && checkReverse.valid ? 'good' : 'warn'}>● {checkForward.valid && checkReverse.valid ? '两个矩阵均为合法旋转' : '旋转矩阵有误'}</span><span>RᵀR−I 最大误差 {formatValue(Math.max(checkForward.orthogonalityError, checkReverse.orthogonalityError))}</span><span>det(R) = {formatValue(checkForward.determinant)}</span><span>{difference < 1e-6 ? '当前角度下两者重合' : `两种姿态相差 ${formatValue(difference)}°`}</span></div>
    <div className="readout-grid"><VectorReadout label={rotationSense === 'active' ? 'A · 旋转后的世界向量' : 'A · 固定向量在旋转参考系中的坐标'} values={vectorA.coordinates.toArray()} /><VectorReadout label={rotationSense === 'active' ? 'B · 旋转后的世界向量' : 'B · 固定向量在旋转参考系中的坐标'} values={vectorB.coordinates.toArray()} /></div>
    <div className="equation-strip"><Formula tex={rotationSense === 'active' ? "v'_W=R(t)v_W" : "v_{local}=R(t)^T v_W"} /><span>初始世界向量 {vectorText(baseVector)} · {rotationSense === 'active' ? '向量转动，使用 R' : '参考系转动，使用 Rᵀ = R⁻¹'}</span></div>
    <details className="advanced-results"><summary>查看目标角度的单轴矩阵 <span>Rx / Ry / Rz</span></summary>
      <div className="matrix-layout rotation-matrices single-axis">{(['X', 'Y', 'Z'] as const).map((axis) => <MatrixView key={axis} matrix={rotationMatrix(rotationAbout(axis, angles[axis]))} size={3} label={factor(axis, 1)} compact />)}</div>
    </details>
  </div>;
}

function FKResults() {
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
    <div className="result-intro"><span className="eyebrow">FORWARD KINEMATICS</span><h2>关节角 → 末端位姿</h2><p>当前只显示教学用 3 关节运动学骨架；每个关节改变后，下游坐标轴与矩阵链实时重算。真实几何将在你提供 URDF 后替换。</p></div>
    <div className="equation-strip"><Formula tex="{}^{\mathrm{base}}T_{\mathrm{tool}} = {}^{\mathrm{base}}T_{\mathrm{link1}}\;{}^{\mathrm{link1}}T_{\mathrm{link2}}\;{}^{\mathrm{link2}}T_{\mathrm{tool}}" /><span>q1 → q2 → q3</span></div>
    <div className="matrix-layout"><MatrixView matrix={finalMatrix} label="{}^{\mathrm{base}}T_{\mathrm{tool}}" />
      <MatrixView matrix={rotationMatrix(fk.T_base_tool.quaternion)} size={3} label="{}^{\mathrm{base}}R_{\mathrm{tool}}" /></div>
    <div className="readout-grid"><VectorReadout label="末端位置 [x, y, z] · m" values={fk.T_base_tool.position.toArray()} /><VectorReadout label={`${eulerOrder}${eulerOrder === 'ZYX' ? ' / RPY' : ''} Euler · °`} values={finalEuler} unit="°" />
      <VectorReadout label="Quaternion [x, y, z, w]" values={[q.x, q.y, q.z, q.w]} /></div>
    <div className="validation-line"><span className={fkValid ? 'good' : 'warn'}>● {fkValid ? 'FK 矩阵链有效' : 'FK 矩阵链异常'}</span><span>连乘误差 {formatValue(chainError)}</span><span>‖q‖ = {formatValue(q.length())}</span><span>齐次末行 {validateHomogeneous(finalMatrix) ? '[0, 0, 0, 1] ✓' : '异常'}</span></div>
    {T_base_current && <div className="equation-strip"><Formula tex={`{}^{\\mathrm{base}}T_{\\mathrm{${fkStep === 3 ? 'tool' : `link${fkStep}`}}}`} /><span>当前逐级结果 · {vectorText(T_base_current.position)}</span></div>}
    <div className="matrix-layout secondary"><MatrixView matrix={poseMatrix(fk.T_base_link1)} label="{}^{\mathrm{base}}T_{\mathrm{link1}}" compact /><MatrixView matrix={poseMatrix(fk.T_link1_link2)} label="{}^{\mathrm{link1}}T_{\mathrm{link2}}" compact /><MatrixView matrix={poseMatrix(fk.T_link2_link3)} label="{}^{\mathrm{link2}}T_{\mathrm{tool}}" compact /></div>
  </div>;
}

export default function App() {
  const mode = useLabStore((state) => state.mode);
  const setMode = useLabStore((state) => state.setMode);
  const reset = useLabStore((state) => state.reset);
  return <div className="app-shell">
    <header className="topbar"><div className="brand"><span className="brand-mark">⌖</span><span>机器人学 · 坐标实验室</span><small>ROBOTICS LAB</small></div>
      <nav className="mode-nav" aria-label="学习模块">{NAV.map((item, index) => <button key={item.id} className={mode === item.id ? 'active' : ''} onClick={() => setMode(item.id)}><small>0{index + 1}</small>{item.label}</button>)}</nav>
      <button className="reset-button" onClick={reset} title="重置全部场景">重置</button></header>
    <main className="workspace">
      <section className="visual-workspace"><div className="scene-heading"><div><span className="eyebrow">INTERACTIVE 3D</span><h1>{NAV.find((item) => item.id === mode)?.label}</h1></div><span className="heading-note">{mode === 'ik' ? '拖动目标点，用 Jacobian 逐步逼近' : mode === 'pinocchio' ? '沿数据流查看 URDF、FK、SE(3) 与可视化' : mode === 'fk' ? '拖动关节角，看连杆与矩阵一起运动' : mode === 'rotation' ? '调节 Rx、Ry、Rz，对比旋转次序与主动 / 被动视角' : '拖动坐标系与点，查看相对变换和父子矩阵链'}</span></div><Scene />
        {mode === 'rotation' ? <RotationResults /> : mode === 'fk' ? <FKResults /> : mode === 'ik' ? <IKResults /> : mode === 'pinocchio' ? <PinocchioResults /> : <FrameResults />}
      </section>
      <aside className="control-panel" aria-label="实验操作栏"><div className="control-heading"><span className="eyebrow">CONTROLS</span><strong>实验操作</strong><span>{NAV.find((item) => item.id === mode)?.short}</span></div>
        {mode === 'rotation' ? <RotationControls /> : mode === 'fk' ? <FKControls /> : mode === 'ik' ? <IKControls /> : mode === 'pinocchio' ? <PinocchioControls /> : <FrameControls />}
      </aside>
    </main>
  </div>;
}
