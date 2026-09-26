import { useEffect } from 'react';
import { useLabStore } from '../stores/labStore';
import { localize, useLocaleStore } from '../stores/localeStore';

export function RotationPlayback() {
  const language = useLocaleStore((state) => state.language);
  const l = (zh: string, en: string) => localize(language, zh, en);
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
  const phase = rotationProgress === 0 ? l('初始姿态', 'Initial pose') : rotationProgress < 1 ? l('正在执行第 1 步', 'Running step 1') : rotationProgress === 1 ? l('第 1 步完成', 'Step 1 complete') : rotationProgress < 2 ? l('正在执行第 2 步', 'Running step 2') : l('两步旋转完成', 'Both rotations complete');
  return <section className="control-card rotation-playback">
    <div className="pane-title"><div><span className="eyebrow">STEP BY STEP</span><h2>{l('把旋转过程拆开看', 'Inspect each rotation step')}</h2></div></div>
    <div className="rotation-path"><span>A</span><strong>{l(`先 ${rightAxis} → 再 ${leftAxis}`, `${rightAxis} first → then ${leftAxis}`)}</strong><span>B</span><strong>{l(`先 ${leftAxis} → 再 ${rightAxis}`, `${leftAxis} first → then ${rightAxis}`)}</strong></div>
    <div className="rotation-playback-actions">
      <button className="small-dark-button" onClick={rotationPlaying ? pauseRotation : playRotation}>{rotationPlaying ? l('暂停', 'Pause') : rotationProgress >= 2 ? l('从头播放', 'Replay') : l('播放', 'Play')}</button>
      <output>{phase}</output>
    </div>
    <input className="rotation-progress" aria-label={l('旋转演示进度', 'Rotation demo progress')} aria-valuetext={`${phase}, ${Math.round(rotationProgress * 50)}%`} type="range" min={0} max={2} step={0.01} value={rotationProgress} onChange={(event) => setRotationProgress(Number(event.target.value))} />
    <div className="rotation-pair-buttons">
      {[[l('初始', 'Initial'), 0], [l('第 1 步完成', 'Step 1'), 1], [l('最终结果', 'Final'), 2]].map(([label, progress]) => <button key={label} aria-pressed={rotationProgress === progress} className={rotationProgress === progress ? 'active' : ''} onClick={() => setRotationProgress(Number(progress))}>{label}</button>)}
    </div>
    <p className="fine-print">{l('每一步都绕固定的世界轴旋转。拖动进度可逐帧查看；对应矩阵和向量读数实时更新。', 'Each step rotates about a fixed world axis. Scrub the timeline frame by frame; the matrices and vector readouts update live.')}</p>
  </section>;
}
