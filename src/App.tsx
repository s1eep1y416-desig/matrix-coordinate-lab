import { useEffect, useMemo, useState } from 'react';
import { Quaternion, Vector3 } from 'three';
import { EULER_ORDERS, eulerFromQuaternion, isNearGimbalLock, type EulerOrder } from './math/euler';
import { forwardKinematics } from './math/kinematics';
import { axisAngleFromQuaternion } from './math/quaternion';
import { rotationAbout, rotationMatrix, rotationProduct, validateRotation } from './math/rotation';
import { inversePose, inverseTransformPoint, poseMatrix, transformPoint, validateHomogeneous } from './math/transform';
import { Formula, MatrixView, VectorReadout } from './components/MathView';
import { NumberField, formatValue } from './components/NumberField';
import { Scene } from './components/Scene';
import { useLabStore, worldPoseFor, relativeFramePose, type DofKey, type FrameNode, type LabMode } from './stores/labStore';
import 'katex/dist/katex.min.css';
import './styles.css';

const NAV: { id: LabMode; label: string; short: string }[] = [
  { id: 'frames', label: '坐标系与点', short: 'Frames' },
  { id: 'rotation', label: '旋转矩阵', short: 'Rotation' },
  { id: 'chain', label: '变换链', short: 'Transforms' },
  { id: 'fk', label: '3-Link FK', short: 'Kinematics' },
];
const labels = ['X', 'Y', 'Z'] as const;
const dofKeys: DofKey[] = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz'];
const texId = (id: string) => id === 'world' ? 'W' : id.replace(/[^A-Za-z0-9]/g, '');
const vectorText = (vector: Vector3) => `[ ${vector.toArray().map(formatValue).join(' , ')} ]`;
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
  return <><FrameTree /><FramePoseControls selected={selected} /><DofControls selected={selected} /><PointControls selected={selected} /><QuaternionControls selected={selected} /></>;
}

function RotationControls() {
  const { rotationX, rotationY, setRotationDemo, rotationSense, setRotationSense } = useLabStore();
  const control = (axis: 'X' | 'Y', value: number) => <div className="slider-row" key={axis}>
    <strong>R{axis}</strong><input aria-label={`R${axis} 角度`} type="range" min="-180" max="180" step="1" value={value} onChange={(event) => setRotationDemo(axis, Number(event.target.value))} />
    <NumberField label="角度" unit="°" value={value} onCommit={(number) => { setRotationDemo(axis, number); return Math.max(-180, Math.min(180, number)); }} />
  </div>;
  return <>
    <section className="control-card"><PaneTitle eyebrow="MATRIX ORDER" title="两个旋转的次序" />{control('X', rotationX)}{control('Y', rotationY)}
      <p className="fine-print">列向量约定：<Formula tex="R_xR_y" /> 先执行 <Formula tex="R_y" />，再执行 <Formula tex="R_x" />；<Formula tex="R_yR_x" /> 的次序相反。拖动角度可观察两个物体的不同姿态。</p>
    </section>
    <section className="control-card"><PaneTitle eyebrow="ACTIVE / PASSIVE" title="旋转物体，还是改变描述？" />
      <div className="segmented"><button className={rotationSense === 'active' ? 'active' : ''} onClick={() => setRotationSense('active')}>Active · 物体转</button><button className={rotationSense === 'passive' ? 'active' : ''} onClick={() => setRotationSense('passive')}>Passive · 坐标系转</button></div>
      <p className="fine-print">{rotationSense === 'active' ? '参考系固定，世界向量随旋转改变。' : '空间向量固定，变的是观察它的参考坐标系；向量的世界位置不变。'}</p>
    </section>
  </>;
}

function TransformControls() {
  const { frames, sourceId, targetId, setSource, setTarget } = useLabStore();
  return <section className="control-card"><PaneTitle eyebrow="RELATIVE TRANSFORM" title="从哪个 Frame 看？" />
    <div className="two-selects"><label className="select-label">源坐标系<select value={sourceId} onChange={(event) => setSource(event.target.value)}>{frames.map((frame) => <option key={frame.id} value={frame.id}>{frame.name}</option>)}</select></label>
      <label className="select-label">目标坐标系<select value={targetId} onChange={(event) => setTarget(event.target.value)}>{frames.map((frame) => <option key={frame.id} value={frame.id}>{frame.name}</option>)}</select></label></div>
    <p className="fine-print">显示 <Formula tex={`{}^{${texId(targetId)}}T_{${texId(sourceId)}}`} />。它把源坐标系里的点转换为目标坐标系的数值。</p>
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
    </section>
    <section className="control-card"><PaneTitle eyebrow="CHAIN REPLAY" title="逐级查看矩阵链" />
      <div className="step-buttons">{['base', 'J1', 'J2', 'tool'].map((label, index) => <button key={label} className={fkStep === index ? 'active' : ''} onClick={() => setFkStep(index)}>{label}</button>)}</div>
      <p className="fine-print">选择一步，场景只显示计算到该级的连杆和坐标系。</p>
    </section>
  </>;
}

function FrameResults() {
  const { frames, selectedFrameId, sourceId, targetId, eulerOrder, pointWorld, mode } = useLabStore();
  const T_W_selected = worldPoseFor(frames, selectedFrameId);
  const T_selected_W = inversePose(T_W_selected);
  const T_target_source = relativeFramePose(frames, targetId, sourceId);
  const T_W_source = worldPoseFor(frames, sourceId);
  const pSource = inverseTransformPoint(T_W_source, pointWorld);
  const pTarget = inverseTransformPoint(worldPoseFor(frames, targetId), pointWorld);
  const euler = eulerFromQuaternion(T_W_selected.quaternion, eulerOrder);
  const axisAngle = axisAngleFromQuaternion(T_W_selected.quaternion);
  const validation = validateRotation(T_W_selected.quaternion);
  const quaternion = T_W_selected.quaternion;
  const frameB = frames.find((frame) => frame.id === 'B');
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">COORDINATE TRANSFORM</span><h2>空间位置与矩阵同步</h2><p>拖动坐标系或点 P，下面的数字会随场景一起更新。</p></div>
    {mode === 'chain' && frameB && <div className="equation-strip"><Formula tex={`{}^{W}T_{${texId(frameB.parentId ?? 'world')}}\\;{}^{${texId(frameB.parentId ?? 'world')}}T_B = {}^{W}T_B`} /><span>父子变换连乘</span></div>}
    <div className="matrix-layout">
      <MatrixView matrix={poseMatrix(T_W_selected)} label={`{}^{W}T_{${texId(selectedFrameId)}}`} />
      <MatrixView matrix={rotationMatrix(T_W_selected.quaternion)} size={3} label={`{}^{W}R_{${texId(selectedFrameId)}}`} />
    </div>
    <div className="readout-grid">
      <VectorReadout label="World position · m" values={T_W_selected.position.toArray()} />
      <VectorReadout label={`${eulerOrder}${eulerOrder === 'ZYX' ? ' / RPY' : ''} Euler · °`} values={euler} unit="°" />
      <VectorReadout label="Quaternion [x, y, z, w]" values={[quaternion.x, quaternion.y, quaternion.z, quaternion.w]} />
      <VectorReadout label="Axis-Angle · axis / °" values={[...axisAngle.axis.toArray(), axisAngle.angleDeg]} />
    </div>
    <div className="validation-line"><span className={validation.valid ? 'good' : 'warn'}>● {validation.valid ? '合法旋转矩阵' : '旋转矩阵有误'}</span><span>RᵀR≈I · 误差 {formatValue(validation.orthogonalityError)}</span><span>det(R) = {formatValue(validation.determinant)}</span><span>‖q‖ = {formatValue(quaternion.length())}</span><span>齐次末行 {validateHomogeneous(poseMatrix(T_W_selected)) ? '[0, 0, 0, 1] ✓' : '异常'}</span></div>
    <div className="matrix-layout secondary">
      <MatrixView matrix={poseMatrix(T_selected_W)} label={`{}^{${texId(selectedFrameId)}}T_W = ({}^{W}T_{${texId(selectedFrameId)}})^{-1}`} compact />
      <MatrixView matrix={poseMatrix(T_target_source)} label={`{}^{${texId(targetId)}}T_{${texId(sourceId)}}`} compact />
    </div>
    <div className="equation-strip"><Formula tex={`p_{${texId(targetId)}} = ({}^{W}T_{${texId(targetId)}})^{-1}\\,{}^{W}T_{${texId(sourceId)}}\\,p_{${texId(sourceId)}}`} /><span>{vectorText(pSource)} → {vectorText(pTarget)}</span></div>
    {mode === 'chain' && frameB && <div className="matrix-layout secondary">
      <MatrixView matrix={poseMatrix(worldPoseFor(frames, frameB.parentId ?? 'world'))} label={`{}^{W}T_{${texId(frameB.parentId ?? 'world')}}`} compact />
      <MatrixView matrix={poseMatrix(frameB.pose)} label={`{}^{${texId(frameB.parentId ?? 'world')}}T_B`} compact />
      <MatrixView matrix={poseMatrix(worldPoseFor(frames, 'B'))} label="{}^{W}T_B" compact />
    </div>}
  </div>;
}

function RotationResults() {
  const { rotationX, rotationY, rotationSense } = useLabStore();
  const Rx = rotationAbout('X', rotationX), Ry = rotationAbout('Y', rotationY);
  const RxRy = rotationProduct(Rx, Ry), RyRx = rotationProduct(Ry, Rx);
  const baseVector = new Vector3(1, 0.35, 0.4);
  const activeVector = baseVector.clone().applyQuaternion(RxRy);
  const passiveVector = baseVector.clone().applyQuaternion(RxRy.clone().invert());
  const check = validateRotation(RxRy);
  return <div className="results-stack">
    <div className="result-intro"><span className="eyebrow">ROTATION IS NOT COMMUTATIVE</span><h2>同样两个角度，次序不同</h2><p>场景左侧显示 RxRy，右侧显示 RyRx；改变角度时两者的姿态同步变化。</p></div>
    <div className="matrix-layout rotation-matrices"><MatrixView matrix={rotationMatrix(Rx)} size={3} label="R_x(\theta_x)" compact /><MatrixView matrix={rotationMatrix(Ry)} size={3} label="R_y(\theta_y)" compact />
      <MatrixView matrix={rotationMatrix(RxRy)} size={3} label="R_xR_y" /><MatrixView matrix={rotationMatrix(RyRx)} size={3} label="R_yR_x" /></div>
    <div className="validation-line"><span className={check.valid ? 'good' : 'warn'}>● {check.valid ? '合法旋转矩阵' : '旋转矩阵有误'}</span><span>RᵀR−I 最大误差 {formatValue(check.orthogonalityError)}</span><span>det(R) = {formatValue(check.determinant)}</span><span>{RxRy.angleTo(RyRx) < 1e-6 ? '当前角度下两者重合' : `两种姿态相差 ${formatValue(RxRy.angleTo(RyRx) * 180 / Math.PI)}°`}</span></div>
    <div className="equation-strip"><Formula tex={rotationSense === 'active' ? "v_W'=Rv_W" : "v_A=R^{-1}v_W"} /><span>{rotationSense === 'active' ? `物体转：${vectorText(activeVector)}` : `参考系转：世界向量 ${vectorText(baseVector)} 不动，在新坐标系读作 ${vectorText(passiveVector)}`}</span></div>
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
    <div className="result-intro"><span className="eyebrow">FORWARD KINEMATICS</span><h2>关节角 → 末端位姿</h2><p>每个关节改变后，下游连杆、坐标轴与矩阵链实时重算。</p></div>
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
      <section className="visual-workspace"><div className="scene-heading"><div><span className="eyebrow">INTERACTIVE 3D</span><h1>{NAV.find((item) => item.id === mode)?.label}</h1></div><span className="heading-note">{mode === 'fk' ? '拖动关节角，看连杆与矩阵一起运动' : mode === 'rotation' ? '调节 Rx、Ry，对比旋转次序与主动 / 被动视角' : '拖动场景中的坐标系，观察数学如何改变'}</span></div><Scene />
        {mode === 'rotation' ? <RotationResults /> : mode === 'fk' ? <FKResults /> : <FrameResults />}
      </section>
      <aside className="control-panel" aria-label="实验操作栏"><div className="control-heading"><span className="eyebrow">CONTROLS</span><strong>实验操作</strong><span>{NAV.find((item) => item.id === mode)?.short}</span></div>
        {mode === 'rotation' ? <RotationControls /> : mode === 'fk' ? <FKControls /> : <><FrameControls />{mode === 'chain' && <TransformControls />}</>}
      </aside>
    </main>
  </div>;
}
