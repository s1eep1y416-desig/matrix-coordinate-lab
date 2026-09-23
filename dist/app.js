(() => {
  const $ = (s) => document.querySelector(s);
  const $$ = (s) => [...document.querySelectorAll(s)];
  const clamp = (n, min, max) => Math.min(max, Math.max(min, n));
  const rad = (deg) => deg * Math.PI / 180;
  const clean = (n) => Math.abs(n) < 1e-8 ? 0 : Math.round(n * 100) / 100;
  const fmt = (n) => Number.isInteger(clean(n)) ? String(clean(n)) : clean(n).toFixed(2).replace(/0+$/, "");
  const vecFmt = (v) => `(${v.map(fmt).join(", ")})`;
  const numberFrom = (value, fallback = 0) => {
    const n = Number(String(value).replace("，", "."));
    return Number.isFinite(n) ? clamp(n, -50, 50) : fallback;
  };

  const colors = ["#b18cff", "#ffb55d", "#5fd4cf", "#f27caa", "#d2ee62"];
  const lessons = {
    vector: { index: 1, kicker: "第 1 节 · 空间", title: "从平面走进空间", body: "三维点写作 <b>p = (x, y, z)</b>。拖动画布旋转视角，先找到红、绿、蓝三条坐标轴。", note: "红色是 x，绿色是 y，蓝色是 z。颜色在所有坐标系中保持一致。" },
    frame: { index: 2, kicker: "第 2 节 · 参照物", title: "坐标依赖参照系", body: "同一个空间点，在不同坐标系里会有不同的数值。坐标系的位置决定原点，姿态决定三根轴的方向。", note: "点击左侧“添加”，可以同时观察多个局部坐标系。" },
    rotation: { index: 3, kicker: "第 3 节 · 姿态", title: "围绕三根轴旋转", body: "Rx、Ry、Rz 分别表示绕 x、y、z 轴旋转。实验室使用 <b>Rz · Ry · Rx</b> 的顺序组合它们。", note: "旋转顺序很重要：交换两个旋转，通常会得到不同姿态。" },
    convert: { index: 4, kicker: "第 4 节 · 换算", title: "先到世界，再去目标系", body: "从源坐标系换到目标坐标系，先用源矩阵得到世界坐标，再乘目标矩阵的逆。", note: "公式是 p目标 = T目标⁻¹ · T源 · p源。下方结果栏展示了这两步。" }
  };

  const worldFrame = { id: "world", name: "World", color: "#d7dbe3", t: [0,0,0], r: [0,0,0], locked: true };
  const state = {
    frames: [worldFrame, { id: "frame-1", name: "坐标系 A", color: colors[0], t: [1,1,0], r: [0,0,30] }],
    sourceId: "frame-1", targetId: "world", point: [2,1,1], lesson: "vector", completed: new Set(),
    camera: { yaw: -38, pitch: 26, zoom: 54 }, dragging: false, lastPointer: [0,0], frameSerial: 1
  };

  const canvas = $("#sceneCanvas");
  const ctx = canvas.getContext("2d");
  const poseInputs = [$("#tx"),$("#ty"),$("#tz"),$("#rx"),$("#ry"),$("#rz")];
  const pointInputs = [$("#px"),$("#py"),$("#pz")];

  function frameById(id) { return state.frames.find((f) => f.id === id) || worldFrame; }
  function rotationMatrix(r) {
    const [x,y,z] = r.map(rad), cx=Math.cos(x), sx=Math.sin(x), cy=Math.cos(y), sy=Math.sin(y), cz=Math.cos(z), sz=Math.sin(z);
    return [
      cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx,
      sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx,
      -sy,   cy*sx,            cy*cx
    ];
  }
  function mul3(m, v) { return [m[0]*v[0]+m[1]*v[1]+m[2]*v[2], m[3]*v[0]+m[4]*v[1]+m[5]*v[2], m[6]*v[0]+m[7]*v[1]+m[8]*v[2]]; }
  function localToWorld(frame, point) { const p=mul3(rotationMatrix(frame.r),point); return p.map((n,i)=>n+frame.t[i]); }
  function worldToLocal(frame, point) {
    const m=rotationMatrix(frame.r), p=point.map((n,i)=>n-frame.t[i]);
    return [m[0]*p[0]+m[3]*p[1]+m[6]*p[2], m[1]*p[0]+m[4]*p[1]+m[7]*p[2], m[2]*p[0]+m[5]*p[1]+m[8]*p[2]];
  }
  function homogeneous(frame) {
    const m=rotationMatrix(frame.r);
    return [m[0],m[1],m[2],frame.t[0],m[3],m[4],m[5],frame.t[1],m[6],m[7],m[8],frame.t[2],0,0,0,1];
  }

  function project(p, width, height) {
    const yaw=rad(state.camera.yaw), pitch=rad(state.camera.pitch);
    const right=[Math.cos(yaw),-Math.sin(yaw),0];
    const up=[-Math.sin(pitch)*Math.sin(yaw),-Math.sin(pitch)*Math.cos(yaw),Math.cos(pitch)];
    const depth=[Math.cos(pitch)*Math.sin(yaw),Math.cos(pitch)*Math.cos(yaw),Math.sin(pitch)];
    const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    return [width/2+dot(p,right)*state.camera.zoom,height/2-dot(p,up)*state.camera.zoom,dot(p,depth)];
  }
  function line3(a,b,color,width=1,dash=[]) {
    const rect=canvas.getBoundingClientRect(), pa=project(a,rect.width,rect.height), pb=project(b,rect.width,rect.height);
    ctx.save(); ctx.strokeStyle=color; ctx.lineWidth=width; ctx.setLineDash(dash); ctx.lineCap="round";
    ctx.beginPath(); ctx.moveTo(pa[0],pa[1]); ctx.lineTo(pb[0],pb[1]); ctx.stroke(); ctx.restore();
  }
  function arrow3(a,b,color,width=2) {
    const rect=canvas.getBoundingClientRect(), pa=project(a,rect.width,rect.height), pb=project(b,rect.width,rect.height);
    const angle=Math.atan2(pb[1]-pa[1],pb[0]-pa[0]), size=7+width;
    ctx.save(); ctx.strokeStyle=color; ctx.fillStyle=color; ctx.lineWidth=width; ctx.lineCap="round";
    ctx.beginPath(); ctx.moveTo(pa[0],pa[1]); ctx.lineTo(pb[0],pb[1]); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(pb[0],pb[1]); ctx.lineTo(pb[0]-size*Math.cos(angle-.5),pb[1]-size*Math.sin(angle-.5)); ctx.lineTo(pb[0]-size*Math.cos(angle+.5),pb[1]-size*Math.sin(angle+.5)); ctx.closePath(); ctx.fill(); ctx.restore();
  }
  function label3(text,p,color="#dce0e7",offset=[7,-8]) {
    const rect=canvas.getBoundingClientRect(), q=project(p,rect.width,rect.height);
    ctx.save(); ctx.font="600 11px ui-monospace, SFMono-Regular, monospace"; const w=ctx.measureText(text).width+10;
    const x=clamp(q[0]+offset[0]-4,4,rect.width-w-4), y=clamp(q[1]+offset[1]-13,4,rect.height-23);
    ctx.fillStyle="rgba(13,15,19,.78)"; ctx.beginPath(); ctx.roundRect(x,y,w,19,5); ctx.fill();
    ctx.fillStyle=color; ctx.fillText(text,x+4,y+13); ctx.restore();
  }
  function drawFrame(frame, emphasis=false) {
    const m=rotationMatrix(frame.r), o=frame.t, length=frame.locked?2.35:1.28;
    const axes=[[1,0,0],[0,1,0],[0,0,1]], axisColors=["#ff625a","#62d47b","#5995ff"];
    axes.forEach((axis,i)=>{ const d=mul3(m,axis), end=o.map((n,j)=>n+d[j]*length); arrow3(o,end,axisColors[i],emphasis?2.7:1.8); if(emphasis||frame.locked) label3("xyz"[i],end,axisColors[i]); });
    if(!frame.locked){ const rect=canvas.getBoundingClientRect(), q=project(o,rect.width,rect.height); ctx.save(); ctx.fillStyle=frame.color; ctx.strokeStyle="#101217"; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(q[0],q[1],emphasis?6:4.5,0,Math.PI*2); ctx.fill(); ctx.stroke(); ctx.restore(); label3(frame.name,o,frame.color,[8,18]); }
  }
  function draw() {
    const rect=canvas.getBoundingClientRect(), dpr=Math.min(window.devicePixelRatio||1,2), width=rect.width, height=rect.height;
    if(canvas.width!==Math.round(width*dpr)||canvas.height!==Math.round(height*dpr)){ canvas.width=Math.round(width*dpr); canvas.height=Math.round(height*dpr); }
    ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,width,height);
    for(let i=-5;i<=5;i++){
      const major=i===0; line3([i,-5,0],[i,5,0],major?"rgba(255,255,255,.18)":"rgba(255,255,255,.07)",major?1.3:1);
      line3([-5,i,0],[5,i,0],major?"rgba(255,255,255,.18)":"rgba(255,255,255,.07)",major?1.3:1);
    }
    const source=frameById(state.sourceId), target=frameById(state.targetId);
    const others=state.frames.filter(f=>!f.locked&&f.id!==source.id&&f.id!==target.id);
    drawFrame(worldFrame,source.locked||target.locked); others.forEach(f=>drawFrame(f,false));
    if(!target.locked&&target.id!==source.id) drawFrame(target,false);
    if(!source.locked) drawFrame(source,true);
    const worldPoint=localToWorld(source,state.point), sourceOrigin=source.t;
    line3(sourceOrigin,worldPoint,source.color,2,[6,5]);
    const q=project(worldPoint,width,height); ctx.save(); ctx.fillStyle="#b7f34b"; ctx.strokeStyle="#101217"; ctx.lineWidth=2.5; ctx.beginPath(); ctx.arc(q[0],q[1],7,0,Math.PI*2); ctx.fill(); ctx.stroke(); ctx.restore();
    label3(`p ${vecFmt(worldPoint)}`,worldPoint,"#d9fa9e",[10,-12]);
  }

  function renderFrameList() {
    $("#frameList").innerHTML=state.frames.map(f=>`<button type="button" role="option" aria-selected="${f.id===state.sourceId}" class="${f.id===state.sourceId?"selected":""}" data-frame-id="${f.id}" style="--frame-color:${f.color}">${f.locked?'<span class="world-icon" aria-hidden="true"></span>':'<span class="frame-dot" aria-hidden="true"></span>'}<span class="frame-name">${f.name}</span><span class="frame-position">${f.locked?"固定":vecFmt(f.t)}</span></button>`).join("");
    $$("[data-frame-id]").forEach(btn=>btn.addEventListener("click",()=>{ state.sourceId=btn.dataset.frameId; if(state.targetId===state.sourceId){ state.targetId=state.sourceId==="world"?(state.frames[1]?.id||"world"):"world"; } updateUI({force:true}); }));
  }
  function renderSelects() {
    const options=state.frames.map(f=>`<option value="${f.id}">${f.name}</option>`).join("");
    $("#sourceFrame").innerHTML=options; $("#targetFrame").innerHTML=options;
    $("#sourceFrame").value=state.sourceId; $("#targetFrame").value=state.targetId;
  }
  function updateUI({force=false}={}) {
    const source=frameById(state.sourceId), target=frameById(state.targetId), worldPoint=localToWorld(source,state.point), targetPoint=worldToLocal(target,worldPoint);
    renderFrameList(); renderSelects();
    $("#editorTitle").textContent=source.name; $("#frameEditor").disabled=source.locked; $("#deleteFrame").disabled=source.locked;
    $("#selectedColor").style.setProperty("--frame-color",source.color);
    [...source.t,...source.r].forEach((v,i)=>{ if(force||document.activeElement!==poseInputs[i]) poseInputs[i].value=fmt(v); });
    state.point.forEach((v,i)=>{ if(force||document.activeElement!==pointInputs[i]) pointInputs[i].value=fmt(v); });
    $("#localResult").textContent=vecFmt(state.point); $("#worldResult").textContent=vecFmt(worldPoint); $("#targetResult").textContent=vecFmt(targetPoint);
    $("#localFrameLabel").textContent=source.name; $("#targetFrameLabel").textContent=target.name;
    $("#sceneSource").textContent=source.name; $("#sceneTarget").textContent=target.name;
    $("#matrixTitle").innerHTML=`T<sub>World←${source.name.replace("坐标系 ","")}</sub>`;
    $("#matrixReadout").innerHTML=homogeneous(source).map(v=>`<span>${fmt(v)}</span>`).join("");
    $("#addFrame").disabled=state.frames.length>=6;
    draw();
  }

  function addFrame() {
    if(state.frames.length>=6) return null;
    state.frameSerial+=1; const index=state.frameSerial-1, letter=String.fromCharCode(65+index);
    const frame={ id:`frame-${state.frameSerial}`, name:`坐标系 ${letter}`, color:colors[index%colors.length], t:[clean(1+(index%2)*1.2),clean(-1+index*.7),clean((index%3)*.5)], r:[index*10%40,0,index*22%90] };
    state.frames.push(frame); state.sourceId=frame.id; if(state.targetId===frame.id) state.targetId="world"; updateUI({force:true}); return frame;
  }
  function deleteSelected() {
    const source=frameById(state.sourceId); if(source.locked) return false;
    state.frames=state.frames.filter(f=>f.id!==source.id); if(state.targetId===source.id) state.targetId="world"; state.sourceId="world"; updateUI({force:true}); return true;
  }
  function selectLesson(id) {
    const lesson=lessons[id]; if(!lesson) return;
    state.completed.add(state.lesson); state.lesson=id;
    $$(".lesson-item").forEach(item=>{ item.classList.toggle("active",item.dataset.lesson===id); item.classList.toggle("complete",state.completed.has(item.dataset.lesson)); item.setAttribute("aria-current",item.dataset.lesson===id?"step":"false"); });
    $("#lessonKicker").textContent=lesson.kicker; $("#lessonTitle").textContent=lesson.title; $("#lessonBody").innerHTML=lesson.body; $("#lessonNote").textContent=lesson.note;
    $("#progressLabel").textContent=`第 ${lesson.index} / 4 节`; $("#progressBar").style.width=`${lesson.index*25}%`;
    const firstLocal=state.frames.find(f=>!f.locked)||addFrame();
    if(id==="vector"){ state.sourceId="world"; state.targetId=firstLocal?.id||"world"; state.point=[2,1,1]; }
    if(id==="frame"&&firstLocal){ state.sourceId=firstLocal.id; state.targetId="world"; firstLocal.t=[1.5,1,.5]; firstLocal.r=[0,0,0]; state.point=[1,1,1]; }
    if(id==="rotation"&&firstLocal){ state.sourceId=firstLocal.id; state.targetId="world"; firstLocal.r=[25,-20,45]; state.point=[2,0,1]; }
    if(id==="convert"&&firstLocal){ let second=state.frames.find(f=>!f.locked&&f.id!==firstLocal.id); if(!second) second=addFrame(); state.sourceId=firstLocal.id; state.targetId=second?.id||"world"; state.point=[1.5,1,.5]; }
    updateUI({force:true});
  }

  poseInputs.forEach((input,i)=>input.addEventListener("input",()=>{ const f=frameById(state.sourceId); if(f.locked)return; if(i<3)f.t[i]=numberFrom(input.value,f.t[i]); else f.r[i-3]=numberFrom(input.value,f.r[i-3]); updateUI(); }));
  pointInputs.forEach((input,i)=>input.addEventListener("input",()=>{ state.point[i]=numberFrom(input.value,state.point[i]); updateUI(); }));
  $("#sourceFrame").addEventListener("change",e=>{ state.sourceId=e.target.value; updateUI({force:true}); });
  $("#targetFrame").addEventListener("change",e=>{ state.targetId=e.target.value; updateUI({force:true}); });
  $("#addFrame").addEventListener("click",addFrame); $("#deleteFrame").addEventListener("click",deleteSelected);
  $$(".lesson-item").forEach(btn=>btn.addEventListener("click",()=>selectLesson(btn.dataset.lesson)));
  $$("[data-preset]").forEach(btn=>btn.addEventListener("click",()=>{ const f=frameById(state.sourceId); if(f.locked)return; const presets={z45:[0,0,45],tilt:[30,-25,35],identity:[0,0,0]}; f.r=presets[btn.dataset.preset].slice(); updateUI({force:true}); }));
  $("#resetView").addEventListener("click",()=>{ state.camera={yaw:-38,pitch:26,zoom:54}; draw(); });
  $("#resetAll").addEventListener("click",()=>{ state.frames=[worldFrame,{id:"frame-1",name:"坐标系 A",color:colors[0],t:[1,1,0],r:[0,0,30]}]; state.sourceId="frame-1"; state.targetId="world"; state.point=[2,1,1]; state.camera={yaw:-38,pitch:26,zoom:54}; state.completed.clear(); state.frameSerial=1; selectLesson("vector"); });
  $("#checkAnswer").addEventListener("click",()=>{ const values=[$("#answerX"),$("#answerY"),$("#answerZ")].map(el=>Number(el.value)); if(values.some(n=>!Number.isFinite(n))){ $("#feedback").textContent="先填入三个坐标。"; return; } $("#feedback").textContent=values.every((n,i)=>Math.abs(n-[3,3,3][i])<.01)?"正确：世界坐标 = 平移 + 局部坐标。":"再想想：这里只发生平移，三个分量分别相加。"; });

  canvas.addEventListener("pointerdown",e=>{ state.dragging=true; state.lastPointer=[e.clientX,e.clientY]; canvas.classList.add("dragging"); canvas.setPointerCapture(e.pointerId); $("#viewHint").classList.add("hidden"); });
  canvas.addEventListener("pointermove",e=>{ if(!state.dragging)return; const dx=e.clientX-state.lastPointer[0],dy=e.clientY-state.lastPointer[1]; state.camera.yaw+=dx*.45; state.camera.pitch=clamp(state.camera.pitch-dy*.4,-75,75); state.lastPointer=[e.clientX,e.clientY]; draw(); });
  const endDrag=()=>{ state.dragging=false; canvas.classList.remove("dragging"); }; canvas.addEventListener("pointerup",endDrag); canvas.addEventListener("pointercancel",endDrag);
  canvas.addEventListener("wheel",e=>{ e.preventDefault(); state.camera.zoom=clamp(state.camera.zoom*(e.deltaY>0?.92:1.08),30,100); draw(); },{passive:false});
  canvas.addEventListener("keydown",e=>{ const step=e.shiftKey?8:3; if(e.key==="ArrowLeft")state.camera.yaw-=step; if(e.key==="ArrowRight")state.camera.yaw+=step; if(e.key==="ArrowUp")state.camera.pitch=clamp(state.camera.pitch+step,-75,75); if(e.key==="ArrowDown")state.camera.pitch=clamp(state.camera.pitch-step,-75,75); if(e.key.startsWith("Arrow")){e.preventDefault();draw();} });
  window.addEventListener("resize",draw);

  function registerWebMCP() {
    if(!document.modelContext?.registerTool)return;
    const register=(tool)=>Promise.resolve(document.modelContext.registerTool(tool)).catch(()=>{});
    register({ name:"add_coordinate_frame",title:"添加坐标系",description:"向三维场景添加一个相对 World 定义的坐标系，并设为源坐标系。",inputSchema:{type:"object",properties:{name:{type:"string",maxLength:24},translation:{type:"array",minItems:3,maxItems:3,items:{type:"number",minimum:-50,maximum:50}},rotationDegrees:{type:"array",minItems:3,maxItems:3,items:{type:"number",minimum:-360,maximum:360}}},additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},execute(input={}){ const frame=addFrame(); if(!frame)throw new Error("最多添加 5 个局部坐标系"); if(input.name)frame.name=String(input.name).slice(0,24); if(input.translation?.length===3&&input.translation.every(Number.isFinite))frame.t=input.translation.map(n=>clamp(n,-50,50)); if(input.rotationDegrees?.length===3&&input.rotationDegrees.every(Number.isFinite))frame.r=input.rotationDegrees.map(n=>clamp(n,-360,360)); updateUI({force:true}); return {id:frame.id,name:frame.name,translation:frame.t,rotationDegrees:frame.r}; } });
    register({ name:"set_coordinate_conversion",title:"设置坐标换算",description:"选择源和目标坐标系，并设置源坐标系中的三维点。",inputSchema:{type:"object",properties:{sourceId:{type:"string"},targetId:{type:"string"},point:{type:"array",minItems:3,maxItems:3,items:{type:"number",minimum:-50,maximum:50}}},required:["sourceId","targetId","point"],additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},execute(input){ if(!frameById(input.sourceId)||!frameById(input.targetId)||!Array.isArray(input.point)||input.point.length!==3||!input.point.every(Number.isFinite))throw new Error("坐标系或三维点无效"); if(!state.frames.some(f=>f.id===input.sourceId)||!state.frames.some(f=>f.id===input.targetId))throw new Error("坐标系不存在"); state.sourceId=input.sourceId;state.targetId=input.targetId;state.point=input.point.slice();updateUI({force:true});const world=localToWorld(frameById(state.sourceId),state.point);return {worldPoint:world,targetPoint:worldToLocal(frameById(state.targetId),world)}; } });
  }
  updateUI({force:true}); registerWebMCP();
})();
