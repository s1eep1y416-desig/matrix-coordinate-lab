import { useEffect } from 'react';
import { useLabStore } from '../stores/labStore';

export function RotationPlayback() {
  const { rotationPair, rotationProgress, rotationPlaying, setRotationProgress, playRotation, pauseRotation, advanceRotation } = useLabStore();
  useEffect(() => {
    if (!rotationPlaying) return;
    let frameId = 0;
    let previousTime = performance.now();
    const tick = (time: number) => {
      advanceRotation(Math.min((time - previousTime) / 1000, 0.1));
      previousTime = time;
      if (useLabStore.getState().rotationPlaying) frameId = requestAnimationFrame(tick);
    };
    const onVisibilityChange = () => { if (document.hidden) pauseRotation(); };
    frameId = requestAnimationFrame(tick);
    document.addEventListener('visibilitychange', onVisibilityChange);
    return () => {
      cancelAnimationFrame(frameId);
      document.removeEventListener('visibilitychange', onVisibilityChange);
    };
  }, [rotationPlaying, advanceRotation, pauseRotation]);

  const [leftAxis, rightAxis] = rotationPair;
  const phase = rotationProgress === 0 ? '初始姿态' : rotationProgress < 1 ? '正在执行第 1 步' : rotationProgress === 1 ? '第 1 步完成' : rotationProgress < 2 ? '正在执行第 2 步' : '两步旋转完成';
  return <section className="control-card rotation-playback">
    <div className="pane-title"><div><span className="eyebrow">STEP BY STEP</span><h2>把旋转过程拆开看</h2></div></div>
    <div className="rotation-path"><span>A</span><strong>先 {rightAxis} → 再 {leftAxis}</strong><span>B</span><strong>先 {leftAxis} → 再 {rightAxis}</strong></div>
    <div className="rotation-playback-actions">
      <button className="small-dark-button" onClick={rotationPlaying ? pauseRotation : playRotation}>{rotationPlaying ? '暂停' : rotationProgress >= 2 ? '从头播放' : '播放'}</button>
      <output>{phase}</output>
    </div>
    <input className="rotation-progress" aria-label="旋转演示进度" aria-valuetext={`${phase}，${Math.round(rotationProgress * 50)}%`} type="range" min={0} max={2} step={0.01} value={rotationProgress} onChange={(event) => setRotationProgress(Number(event.target.value))} />
    <div className="rotation-pair-buttons">
      {[['初始', 0], ['第 1 步完成', 1], ['最终结果', 2]].map(([label, progress]) => <button key={label} aria-pressed={rotationProgress === progress} className={rotationProgress === progress ? 'active' : ''} onClick={() => setRotationProgress(Number(progress))}>{label}</button>)}
    </div>
    <p className="fine-print">每一步都绕固定的世界轴旋转。拖动进度可逐帧查看；对应矩阵和向量读数实时更新。</p>
  </section>;
}
