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
    vector: { index: 1, kicker: "第 1 节 · 空间", title: "从平面走进空间", body: "三维点写作 <b>p = (x, y, z)</b>。用中键拖动旋转视角，再用左键拖动绿色点。", note: "红色是 x，绿色是 y，蓝色是 z。颜色在所有坐标系中保持一致。" },
    frame: { index: 2, kicker: "第 2 节 · 层级", title: "建立父子坐标系", body: "子坐标系的位置和旋转都相对于父坐标系定义。父级移动时，它的全部后代会一起移动。", note: "选中一个坐标系再点击“添加”，新坐标系会成为它的子级。" },
    rotation: { index: 3, kicker: "第 3 节 · 姿态", title: "围绕三根轴旋转", body: "Rx、Ry、Rz 分别表示绕 x、y、z 轴旋转。实验室使用 <b>Rz · Ry · Rx</b> 的顺序组合它们。", note: "旋转顺序很重要：交换两个旋转，通常会得到不同姿态。" },
    convert: { index: 4, kicker: "第 4 节 · 换算", title: "先到世界，再去目标系", body: "从源坐标系换到目标坐标系，先用源矩阵得到世界坐标，再乘目标矩阵的逆。", note: "公式是 p目标 = T目标⁻¹ · T源 · p源。下方结果栏展示了这两步。" }
  };

  const worldFrame = { id: "world", name: "World", color: "#d7dbe3", t: [0,0,0], r: [0,0,0], parentId: null, locked: true };
  const state = {
    frames: [worldFrame, { id: "frame-1", name: "坐标系 A", color: colors[0], t: [1,1,0], r: [0,0,30], parentId: "world" }],
    sourceId: "frame-1", targetId: "world", point: [2,1,1], lesson: "vector", completed: new Set(),
    camera: { yaw: -38, pitch: 26, zoom: 54 }, dragMode: null, dragFrameId: null, lastPointer: [0,0], frameSerial: 1
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
  function mulM3(a,b) {
    const out=[];
    for(let row=0;row<3;row++) for(let col=0;col<3;col++) out[row*3+col]=a[row*3]*b[col]+a[row*3+1]*b[col+3]+a[row*3+2]*b[col+6];
    return out;
  }
  function transpose3(m) { return [m[0],m[3],m[6],m[1],m[4],m[7],m[2],m[5],m[8]]; }
  function worldPose(frame, seen=new Set()) {
    if(frame.locked) return { r:[1,0,0,0,1,0,0,0,1], t:[0,0,0] };
    if(seen.has(frame.id)) return worldPose(worldFrame);
    seen.add(frame.id);
    const parent=frameById(frame.parentId), parentPose=worldPose(parent,seen), localR=rotationMatrix(frame.r);
    return { r:mulM3(parentPose.r,localR), t:mul3(parentPose.r,frame.t).map((n,i)=>n+parentPose.t[i]) };
  }
  function localToWorld(frame, point) { const pose=worldPose(frame), p=mul3(pose.r,point); return p.map((n,i)=>n+pose.t[i]); }
  function worldToLocal(frame, point) {
    const pose=worldPose(frame), p=point.map((n,i)=>n-pose.t[i]);
    return mul3(transpose3(pose.r),p);
  }
  function homogeneous(frame) {
    const pose=worldPose(frame), m=pose.r;
    return [m[0],m[1],m[2],pose.t[0],m[3],m[4],m[5],pose.t[1],m[6],m[7],m[8],pose.t[2],0,0,0,1];
  }
  function matrixToEuler(m) {
    const y=Math.asin(clamp(-m[6],-1,1)), cy=Math.cos(y); let x,z;
    if(Math.abs(cy)>.00001){ x=Math.atan2(m[7],m[8]); z=Math.atan2(m[3],m[0]); }
    else { x=Math.atan2(-m[5],m[4]); z=0; }
    return [x,y,z].map(v=>v*180/Math.PI);
  }
  function descendantsOf(id) {
    const found=new Set(); let changed=true;
    while(changed){ changed=false; state.frames.forEach(f=>{ if(!found.has(f.id)&&(f.parentId===id||found.has(f.parentId))){ found.add(f.id); changed=true; } }); }
    return found;
  }
  function hierarchy(frame) {
    const chain=[]; let current=frame, guard=0;
    while(current&&guard++<state.frames.length+1){ chain.unshift(current.name); if(current.locked)break; current=frameById(current.parentId); }
    return chain;
  }
  function reparentFrame(frame,newParentId) {
    if(frame.locked||frame.id===newParentId||descendantsOf(frame.id).has(newParentId)) return false;
    const oldPose=worldPose(frame), parent=frameById(newParentId), parentPose=worldPose(parent), invParent=transpose3(parentPose.r);
    frame.parentId=parent.id;
    frame.t=mul3(invParent,oldPose.t.map((n,i)=>n-parentPose.t[i])).map(clean);
    frame.r=matrixToEuler(mulM3(invParent,oldPose.r)).map(clean);
    return true;
  }

  function cameraBasis() {
    const yaw=rad(state.camera.yaw), pitch=rad(state.camera.pitch);
    return {
      right:[Math.cos(yaw),-Math.sin(yaw),0],
      up:[-Math.sin(pitch)*Math.sin(yaw),-Math.sin(pitch)*Math.cos(yaw),Math.cos(pitch)],
      depth:[Math.cos(pitch)*Math.sin(yaw),Math.cos(pitch)*Math.cos(yaw),Math.sin(pitch)]
    };
  }
  function project(p, width, height) {
    const {right,up,depth}=cameraBasis();
    const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    return [width/2+dot(p,right)*state.camera.zoom,height/2-dot(p,up)*state.camera.zoom,dot(p,depth)];
  }
  function screenDeltaToWorld(dx,dy) {
    const {right,up}=cameraBasis(), sx=dx/state.camera.zoom, sy=-dy/state.camera.zoom;
    return right.map((n,i)=>n*sx+up[i]*sy);
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
    const pose=worldPose(frame), m=pose.r, o=pose.t, length=frame.locked?2.35:1.28;
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
    state.frames.filter(f=>!f.locked).forEach(f=>{
      const parent=frameById(f.parentId), a=worldPose(parent).t, b=worldPose(f).t;
      line3(a,b,"rgba(183,243,75,.28)",1,[3,5]);
    });
    const source=frameById(state.sourceId), target=frameById(state.targetId);
    const others=state.frames.filter(f=>!f.locked&&f.id!==source.id&&f.id!==target.id);
    drawFrame(worldFrame,source.locked||target.locked); others.forEach(f=>drawFrame(f,false));
    if(!target.locked&&target.id!==source.id) drawFrame(target,false);
    if(!source.locked) drawFrame(source,true);
    const worldPoint=localToWorld(source,state.point), sourceOrigin=worldPose(source).t;
    arrow3(sourceOrigin,worldPoint,"#b7f34b",2.2);
    const q=project(worldPoint,width,height); ctx.save(); ctx.fillStyle="#b7f34b"; ctx.strokeStyle="#101217"; ctx.lineWidth=2.5; ctx.beginPath(); ctx.arc(q[0],q[1],7,0,Math.PI*2); ctx.fill(); ctx.stroke(); ctx.restore();
    label3(`p ${vecFmt(worldPoint)}`,worldPoint,"#d9fa9e",[10,-12]);
  }

  function renderFrameList() {
    const rows=[];
    const walk=(parent,depth)=>{
      rows.push(`<button type="button" role="option" aria-selected="${parent.id===state.sourceId}" class="${parent.id===state.sourceId?"selected":""}" data-frame-id="${parent.id}" style="--frame-color:${parent.color};--depth:${depth}">${parent.locked?'<span class="world-icon" aria-hidden="true"></span>':'<span class="frame-dot" aria-hidden="true"></span>'}<span class="frame-name">${parent.name}</span><span class="frame-position">${parent.locked?"根":`↳ ${frameById(parent.parentId).name.replace("坐标系 ","")}`}</span></button>`);
      state.frames.filter(f=>f.parentId===parent.id).forEach(child=>walk(child,depth+1));
    };
    walk(worldFrame,0); $("#frameList").innerHTML=rows.join("");
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
    const blocked=descendantsOf(source.id);
    $("#parentFrame").innerHTML=state.frames.filter(f=>f.id!==source.id&&!blocked.has(f.id)).map(f=>`<option value="${f.id}">${f.name}</option>`).join("");
    if(!source.locked) $("#parentFrame").value=source.parentId;
    $("#hierarchyPath").textContent=hierarchy(source).join(" → ");
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

  function addFrame(parentId=state.sourceId) {
    if(state.frames.length>=6) return null;
    state.frameSerial+=1; const index=state.frameSerial-1, letter=String.fromCharCode(65+index);
    const parent=state.frames.some(f=>f.id===parentId)?frameById(parentId):worldFrame;
    const frame={ id:`frame-${state.frameSerial}`, name:`坐标系 ${letter}`, color:colors[index%colors.length], t:[1.5,clean((index%2)*.6),clean((index%3)*.35)], r:[index*8%30,0,index*18%60], parentId:parent.id };
    state.frames.push(frame); state.sourceId=frame.id; if(state.targetId===frame.id) state.targetId="world"; updateUI({force:true}); return frame;
  }
  function deleteSelected() {
    const source=frameById(state.sourceId); if(source.locked) return false;
    const nextParent=source.parentId;
    state.frames.filter(f=>f.parentId===source.id).forEach(child=>reparentFrame(child,nextParent));
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
    if(id==="frame"&&firstLocal){ let child=state.frames.find(f=>f.parentId===firstLocal.id); if(!child) child=addFrame(firstLocal.id); state.sourceId=child.id; state.targetId="world"; firstLocal.t=[1.5,1,.5]; firstLocal.r=[0,0,25]; child.t=[1.5,0,.5]; child.r=[0,20,15]; state.point=[1,1,1]; }
    if(id==="rotation"&&firstLocal){ state.sourceId=firstLocal.id; state.targetId="world"; firstLocal.r=[25,-20,45]; state.point=[2,0,1]; }
    if(id==="convert"&&firstLocal){ let second=state.frames.find(f=>f.parentId===firstLocal.id); if(!second) second=addFrame(firstLocal.id); state.sourceId=second?.id||firstLocal.id; state.targetId="world"; state.point=[1.5,1,.5]; }
    updateUI({force:true});
  }

  poseInputs.forEach((input,i)=>input.addEventListener("input",()=>{ const f=frameById(state.sourceId); if(f.locked)return; if(i<3)f.t[i]=numberFrom(input.value,f.t[i]); else f.r[i-3]=numberFrom(input.value,f.r[i-3]); updateUI(); }));
  pointInputs.forEach((input,i)=>input.addEventListener("input",()=>{ state.point[i]=numberFrom(input.value,state.point[i]); updateUI(); }));
  $("#parentFrame").addEventListener("change",e=>{ const f=frameById(state.sourceId); if(reparentFrame(f,e.target.value)) updateUI({force:true}); });
  $("#sourceFrame").addEventListener("change",e=>{ state.sourceId=e.target.value; updateUI({force:true}); });
  $("#targetFrame").addEventListener("change",e=>{ state.targetId=e.target.value; updateUI({force:true}); });
  $("#addFrame").addEventListener("click",addFrame); $("#deleteFrame").addEventListener("click",deleteSelected);
  $$(".lesson-item").forEach(btn=>btn.addEventListener("click",()=>selectLesson(btn.dataset.lesson)));
  $$("[data-preset]").forEach(btn=>btn.addEventListener("click",()=>{ const f=frameById(state.sourceId); if(f.locked)return; const presets={z45:[0,0,45],tilt:[30,-25,35],identity:[0,0,0]}; f.r=presets[btn.dataset.preset].slice(); updateUI({force:true}); }));
  $("#resetView").addEventListener("click",()=>{ state.camera={yaw:-38,pitch:26,zoom:54}; draw(); });
  $("#resetAll").addEventListener("click",()=>{ state.frames=[worldFrame,{id:"frame-1",name:"坐标系 A",color:colors[0],t:[1,1,0],r:[0,0,30],parentId:"world"}]; state.sourceId="frame-1"; state.targetId="world"; state.point=[2,1,1]; state.camera={yaw:-38,pitch:26,zoom:54}; state.completed.clear(); state.frameSerial=1; selectLesson("vector"); });
  $("#checkAnswer").addEventListener("click",()=>{ const values=[$("#answerX"),$("#answerY"),$("#answerZ")].map(el=>Number(el.value)); if(values.some(n=>!Number.isFinite(n))){ $("#feedback").textContent="先填入三个坐标。"; return; } $("#feedback").textContent=values.every((n,i)=>Math.abs(n-[3,3,3][i])<.01)?"正确：世界坐标 = 平移 + 局部坐标。":"再想想：这里只发生平移，三个分量分别相加。"; });

  function hitTest(clientX,clientY) {
    const rect=canvas.getBoundingClientRect(), x=clientX-rect.left, y=clientY-rect.top, source=frameById(state.sourceId);
    const pointWorld=localToWorld(source,state.point), pointScreen=project(pointWorld,rect.width,rect.height);
    const targets=[{type:"point",id:"point",screen:pointScreen,radius:15}];
    state.frames.filter(f=>!f.locked).forEach(f=>targets.push({type:"frame",id:f.id,screen:project(worldPose(f).t,rect.width,rect.height),radius:14}));
    return targets.map(t=>({...t,distance:Math.hypot(x-t.screen[0],y-t.screen[1])})).filter(t=>t.distance<=t.radius).sort((a,b)=>a.distance-b.distance)[0]||null;
  }
  canvas.addEventListener("pointerdown",e=>{
    const hit=hitTest(e.clientX,e.clientY), rotate=e.button===1||(e.pointerType==="touch"&&!hit);
    if(rotate){ state.dragMode="camera"; canvas.classList.add("rotating"); e.preventDefault(); }
    else if(e.button===0&&hit){
      state.dragMode=hit.type; state.dragFrameId=hit.type==="frame"?hit.id:null; canvas.classList.add("moving");
      if(hit.type==="frame"){ state.sourceId=hit.id; if(state.targetId===hit.id)state.targetId="world"; updateUI({force:true}); }
    } else return;
    state.lastPointer=[e.clientX,e.clientY]; canvas.setPointerCapture(e.pointerId); $("#viewHint").classList.add("hidden");
  });
  canvas.addEventListener("pointermove",e=>{
    if(!state.dragMode){ canvas.classList.toggle("hovering",!!hitTest(e.clientX,e.clientY)); return; }
    const dx=e.clientX-state.lastPointer[0],dy=e.clientY-state.lastPointer[1]; state.lastPointer=[e.clientX,e.clientY];
    if(state.dragMode==="camera"){ state.camera.yaw+=dx*.45; state.camera.pitch=clamp(state.camera.pitch-dy*.4,-75,75); draw(); return; }
    const delta=screenDeltaToWorld(dx,dy);
    if(state.dragMode==="frame"){
      const frame=frameById(state.dragFrameId), pose=worldPose(frame), nextWorld=pose.t.map((n,i)=>n+delta[i]);
      frame.t=worldToLocal(frameById(frame.parentId),nextWorld).map(clean);
    } else if(state.dragMode==="point"){
      const source=frameById(state.sourceId), current=localToWorld(source,state.point), next=current.map((n,i)=>n+delta[i]);
      state.point=worldToLocal(source,next).map(clean);
    }
    updateUI({force:true});
  });
  const endDrag=()=>{ state.dragMode=null; state.dragFrameId=null; canvas.classList.remove("rotating","moving"); };
  canvas.addEventListener("pointerup",endDrag); canvas.addEventListener("pointercancel",endDrag); canvas.addEventListener("pointerleave",()=>{ if(!state.dragMode)canvas.classList.remove("hovering"); });
  canvas.addEventListener("auxclick",e=>{ if(e.button===1)e.preventDefault(); });
  canvas.addEventListener("wheel",e=>{ e.preventDefault(); state.camera.zoom=clamp(state.camera.zoom*(e.deltaY>0?.92:1.08),30,100); draw(); },{passive:false});
  canvas.addEventListener("keydown",e=>{ const step=e.shiftKey?8:3; if(e.key==="ArrowLeft")state.camera.yaw-=step; if(e.key==="ArrowRight")state.camera.yaw+=step; if(e.key==="ArrowUp")state.camera.pitch=clamp(state.camera.pitch+step,-75,75); if(e.key==="ArrowDown")state.camera.pitch=clamp(state.camera.pitch-step,-75,75); if(e.key.startsWith("Arrow")){e.preventDefault();draw();} });
  window.addEventListener("resize",draw);

  function registerWebMCP() {
    if(!document.modelContext?.registerTool)return;
    const register=(tool)=>Promise.resolve(document.modelContext.registerTool(tool)).catch(()=>{});
    register({ name:"add_coordinate_frame",title:"添加子坐标系",description:"向三维场景添加一个坐标系，可指定父坐标系，并把新坐标系设为源坐标系。",inputSchema:{type:"object",properties:{name:{type:"string",maxLength:24},parentId:{type:"string"},translation:{type:"array",minItems:3,maxItems:3,items:{type:"number",minimum:-50,maximum:50}},rotationDegrees:{type:"array",minItems:3,maxItems:3,items:{type:"number",minimum:-360,maximum:360}}},additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},execute(input={}){ if(input.parentId&&!state.frames.some(f=>f.id===input.parentId))throw new Error("父坐标系不存在"); const frame=addFrame(input.parentId||state.sourceId); if(!frame)throw new Error("最多添加 5 个局部坐标系"); if(input.name)frame.name=String(input.name).slice(0,24); if(input.translation?.length===3&&input.translation.every(Number.isFinite))frame.t=input.translation.map(n=>clamp(n,-50,50)); if(input.rotationDegrees?.length===3&&input.rotationDegrees.every(Number.isFinite))frame.r=input.rotationDegrees.map(n=>clamp(n,-360,360)); updateUI({force:true}); return {id:frame.id,name:frame.name,parentId:frame.parentId,translation:frame.t,rotationDegrees:frame.r}; } });
    register({ name:"set_frame_parent",title:"设置父坐标系",description:"改变一个局部坐标系的父级，同时保持它当前的世界位置和姿态。",inputSchema:{type:"object",properties:{frameId:{type:"string"},parentId:{type:"string"}},required:["frameId","parentId"],additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},execute(input){ const frame=state.frames.find(f=>f.id===input.frameId),parent=state.frames.find(f=>f.id===input.parentId); if(!frame||frame.locked||!parent)throw new Error("坐标系不存在或不能修改"); if(!reparentFrame(frame,parent.id))throw new Error("不能把坐标系设为自身或其后代的子级");state.sourceId=frame.id;updateUI({force:true});return {frameId:frame.id,parentId:frame.parentId,hierarchy:hierarchy(frame)}; } });
    register({ name:"set_coordinate_conversion",title:"设置坐标换算",description:"选择源和目标坐标系，并设置源坐标系中的三维点。",inputSchema:{type:"object",properties:{sourceId:{type:"string"},targetId:{type:"string"},point:{type:"array",minItems:3,maxItems:3,items:{type:"number",minimum:-50,maximum:50}}},required:["sourceId","targetId","point"],additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},execute(input){ if(!frameById(input.sourceId)||!frameById(input.targetId)||!Array.isArray(input.point)||input.point.length!==3||!input.point.every(Number.isFinite))throw new Error("坐标系或三维点无效"); if(!state.frames.some(f=>f.id===input.sourceId)||!state.frames.some(f=>f.id===input.targetId))throw new Error("坐标系不存在"); state.sourceId=input.sourceId;state.targetId=input.targetId;state.point=input.point.slice();updateUI({force:true});const world=localToWorld(frameById(state.sourceId),state.point);return {worldPoint:world,targetPoint:worldToLocal(frameById(state.targetId),world)}; } });
  }
  updateUI({force:true}); registerWebMCP();
})();
