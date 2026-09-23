(() => {
  const $ = (selector) => document.querySelector(selector);
  const $$ = (selector) => [...document.querySelectorAll(selector)];
  const clamp = (n, min, max) => Math.min(max, Math.max(min, n));
  const clean = (n) => Math.abs(n) < 0.0001 ? 0 : Math.round(n * 100) / 100;
  const fmt = (n) => Number.isInteger(clean(n)) ? String(clean(n)) : clean(n).toFixed(2).replace(/0+$/, "");
  const parse = (value, fallback = 0) => {
    const n = Number(String(value).replace("，", "."));
    return Number.isFinite(n) ? clamp(n, -10, 10) : fallback;
  };

  const lessonData = {
    vector: {
      index: 1, kicker: "第 1 节 · 基础", title: "先认识一个向量",
      body: "向量 <b>v = (x, y)</b> 可以看成从原点出发的一支箭头。试着拖动图中的端点，观察坐标如何变化。",
      note: "横向是 x，纵向是 y。方向和长度共同决定一个向量。",
      matrix: [1, 0, 0, 1], vector: [2, 1]
    },
    scale: {
      index: 2, kicker: "第 2 节 · 对角线", title: "伸缩，也可以翻转",
      body: "对角线上的两个数分别控制 x、y 方向的缩放。数值大于 1 会拉伸，0 到 1 会压缩，负数还会翻转方向。",
      note: "当 det(A) 为负数时，平面方向发生了翻转。",
      matrix: [1.5, 0, 0, 0.7], vector: [2, 1]
    },
    rotate: {
      index: 3, kicker: "第 3 节 · 旋转", title: "让基向量转过一个角度",
      body: "旋转矩阵不改变长度，只改变方向。它的两列告诉我们原来的 x、y 基向量分别去了哪里。",
      note: "纯旋转的行列式始终为 1，所以面积和方向都保持不变。",
      matrix: [0.866, -0.5, 0.5, 0.866], vector: [2, 1]
    },
    shear: {
      index: 4, kicker: "第 4 节 · 组合", title: "剪切会让网格倾斜",
      body: "剪切把一个坐标轴的分量混入另一个坐标轴。多个矩阵相乘时，执行顺序从右向左，而且交换顺序通常会得到不同结果。",
      note: "观察单位方格：面积不变，但直角被推成了斜角。",
      matrix: [1, 0.8, 0, 1], vector: [2, 1]
    }
  };

  const challenges = [
    { matrix: [2, 0, 0, 1], vector: [2, -1] },
    { matrix: [0, -1, 1, 0], vector: [3, 1] },
    { matrix: [-1, 0, 0, 1], vector: [-2, 2] }
  ];

  const state = {
    matrix: [1, 0, 0, 1], vector: [2, 1], progress: 1,
    lesson: "vector", dragging: false, animation: 0, challengeIndex: 0,
    completed: new Set(), hasDragged: false
  };

  const canvas = $("#gridCanvas");
  const ctx = canvas.getContext("2d");
  const canvasWrap = $("#canvasWrap");
  const matrixInputs = [$("#m00"), $("#m01"), $("#m10"), $("#m11")];
  const vectorInputs = [$("#vectorX"), $("#vectorY")];

  function multiply(matrix = state.matrix, vector = state.vector) {
    return [matrix[0] * vector[0] + matrix[1] * vector[1], matrix[2] * vector[0] + matrix[3] * vector[1]];
  }

  function interpolatedMatrix() {
    const t = state.progress;
    return [1 + (state.matrix[0] - 1) * t, state.matrix[1] * t, state.matrix[2] * t, 1 + (state.matrix[3] - 1) * t];
  }

  function pointToCanvas(x, y, width, height, scale) {
    return [width / 2 + x * scale, height / 2 - y * scale];
  }

  function drawArrow(from, to, color, width, dashed = false) {
    const angle = Math.atan2(to[1] - from[1], to[0] - from[0]);
    ctx.save();
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
    ctx.lineCap = "round"; ctx.lineJoin = "round";
    if (dashed) ctx.setLineDash([6, 6]);
    ctx.beginPath(); ctx.moveTo(from[0], from[1]); ctx.lineTo(to[0], to[1]); ctx.stroke();
    ctx.setLineDash([]);
    const size = width * 3.8;
    ctx.beginPath(); ctx.moveTo(to[0], to[1]);
    ctx.lineTo(to[0] - size * Math.cos(angle - Math.PI / 6), to[1] - size * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(to[0] - size * Math.cos(angle + Math.PI / 6), to[1] - size * Math.sin(angle + Math.PI / 6));
    ctx.closePath(); ctx.fill(); ctx.restore();
  }

  function drawLabel(text, point, color, dx = 9, dy = -10) {
    ctx.save(); ctx.font = "600 12px ui-monospace, SFMono-Regular, monospace";
    const width = ctx.measureText(text).width + 12;
    ctx.fillStyle = "rgba(255,255,255,.92)";
    ctx.beginPath(); ctx.roundRect(point[0] + dx - 5, point[1] + dy - 13, width, 20, 5); ctx.fill();
    ctx.fillStyle = color; ctx.fillText(text, point[0] + dx, point[1] + dy + 1); ctx.restore();
  }

  function draw() {
    const rect = canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = rect.width, height = rect.height;
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const scale = Math.max(34, Math.min(54, Math.min(width, height) / 8.8));
    const matrix = interpolatedMatrix();
    const origin = pointToCanvas(0, 0, width, height, scale);

    ctx.save(); ctx.strokeStyle = "#e5e7eb"; ctx.lineWidth = 1;
    for (let i = -12; i <= 12; i++) {
      const a = multiply(matrix, [i, -12]), b = multiply(matrix, [i, 12]);
      const c = multiply(matrix, [-12, i]), d = multiply(matrix, [12, i]);
      const pa = pointToCanvas(a[0], a[1], width, height, scale), pb = pointToCanvas(b[0], b[1], width, height, scale);
      const pc = pointToCanvas(c[0], c[1], width, height, scale), pd = pointToCanvas(d[0], d[1], width, height, scale);
      ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(pc[0], pc[1]); ctx.lineTo(pd[0], pd[1]); ctx.stroke();
    }
    ctx.restore();

    const xAxisEnd = multiply(matrix, [12, 0]), xAxisStart = multiply(matrix, [-12, 0]);
    const yAxisEnd = multiply(matrix, [0, 12]), yAxisStart = multiply(matrix, [0, -12]);
    ctx.save(); ctx.strokeStyle = "#b9bdc6"; ctx.lineWidth = 1.4;
    [[xAxisStart, xAxisEnd], [yAxisStart, yAxisEnd]].forEach(([a, b]) => {
      a = pointToCanvas(a[0], a[1], width, height, scale); b = pointToCanvas(b[0], b[1], width, height, scale);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    });
    ctx.restore();

    const unit = [[0,0],[1,0],[1,1],[0,1]].map((p) => multiply(matrix, p)).map((p) => pointToCanvas(p[0], p[1], width, height, scale));
    ctx.save(); ctx.fillStyle = "rgba(182,243,74,.20)"; ctx.strokeStyle = "rgba(92,135,18,.55)"; ctx.lineWidth = 1.4;
    ctx.beginPath(); unit.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();

    const originalEnd = pointToCanvas(state.vector[0], state.vector[1], width, height, scale);
    const transformed = multiply(matrix, state.vector);
    const transformedEnd = pointToCanvas(transformed[0], transformed[1], width, height, scale);
    drawArrow(origin, originalEnd, "#a8acb6", 2.3, true);
    drawArrow(origin, transformedEnd, "#ff685e", 3.2);
    if (state.progress > .04) drawLabel(`v′ (${fmt(transformed[0])}, ${fmt(transformed[1])})`, transformedEnd, "#ce3b33");
    drawLabel(`v (${fmt(state.vector[0])}, ${fmt(state.vector[1])})`, originalEnd, "#5e636f", 8, 22);
    ctx.save(); ctx.fillStyle = "#b6f34a"; ctx.strokeStyle = "#17191f"; ctx.lineWidth = 2.3;
    ctx.beginPath(); ctx.arc(originalEnd[0], originalEnd[1], 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke(); ctx.restore();
  }

  function updateUI(options = {}) {
    const result = multiply();
    const [a,b,c,d] = state.matrix;
    matrixInputs.forEach((input, i) => { if (document.activeElement !== input || options.force) input.value = fmt(state.matrix[i]); });
    vectorInputs.forEach((input, i) => { if (document.activeElement !== input || options.force) input.value = fmt(state.vector[i]); });
    $("#determinant").textContent = fmt(a * d - b * c);
    $("#resultVector").textContent = `(${fmt(result[0])}, ${fmt(result[1])})`;
    $("#equation").innerHTML = `<span>[ ${fmt(a)}  ${fmt(b)} ; ${fmt(c)}  ${fmt(d)} ]</span> × <span>[ ${fmt(state.vector[0])} ; ${fmt(state.vector[1])} ]</span> = <span>[ ${fmt(result[0])} ; ${fmt(result[1])} ]</span>`;
    $("#progressSlider").value = Math.round(state.progress * 100);
    $("#progressOutput").textContent = `${Math.round(state.progress * 100)}%`;
    draw();
  }

  function setTransform(matrix, vector = state.vector, { animate = true, preset = null } = {}) {
    state.matrix = matrix.map((n) => clean(parse(n)));
    state.vector = vector.map((n) => clean(parse(n)));
    $$("[data-preset]").forEach((button) => button.classList.toggle("selected", button.dataset.preset === preset));
    if (animate && !window.matchMedia("(prefers-reduced-motion: reduce)").matches) play();
    else { state.progress = 1; updateUI({ force: true }); }
  }

  function play() {
    cancelAnimationFrame(state.animation);
    state.progress = 0; $("#playAnimation").classList.add("playing");
    const started = performance.now(), duration = 780;
    const tick = (now) => {
      const raw = clamp((now - started) / duration, 0, 1);
      state.progress = 1 - Math.pow(1 - raw, 3);
      updateUI();
      if (raw < 1) state.animation = requestAnimationFrame(tick);
      else $("#playAnimation").classList.remove("playing");
    };
    state.animation = requestAnimationFrame(tick);
  }

  function selectLesson(id) {
    const lesson = lessonData[id]; if (!lesson) return;
    state.completed.add(state.lesson); state.lesson = id;
    $$(".lesson-item").forEach((item) => {
      item.classList.toggle("active", item.dataset.lesson === id);
      item.classList.toggle("complete", state.completed.has(item.dataset.lesson));
      item.setAttribute("aria-current", item.dataset.lesson === id ? "step" : "false");
    });
    $("#lessonKicker").textContent = lesson.kicker;
    $("#lessonTitle").textContent = lesson.title;
    $("#lessonBody").innerHTML = lesson.body;
    $("#lessonNote").textContent = lesson.note;
    $("#progressLabel").textContent = `第 ${lesson.index} / 4 节`;
    $("#progressBar").style.width = `${lesson.index * 25}%`;
    setTransform(lesson.matrix, lesson.vector, { animate: true, preset: id === "vector" ? null : id });
  }

  function updateChallenge() {
    const q = challenges[state.challengeIndex];
    const [a,b,c,d] = q.matrix, [x,y] = q.vector;
    $("#challengeCount").textContent = `${state.challengeIndex + 1} / ${challenges.length}`;
    $("#challengePrompt").textContent = `[ ${a}  ${b} ; ${c}  ${d} ] × (${x}, ${y})`;
    $("#answerX").value = ""; $("#answerY").value = ""; $("#feedback").textContent = "";
  }

  function checkAnswer() {
    const q = challenges[state.challengeIndex], expected = multiply(q.matrix, q.vector);
    const x = Number($("#answerX").value), y = Number($("#answerY").value);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      $("#feedback").textContent = "先填入两个坐标，再检查。"; return { correct: false, reason: "missing_answer" };
    }
    const correct = Math.abs(x - expected[0]) < .01 && Math.abs(y - expected[1]) < .01;
    if (correct) {
      $("#feedback").textContent = state.challengeIndex === challenges.length - 1 ? "全部完成！你已经掌握了矩阵 × 向量。" : "正确。行乘列，就是这个结果。下一题已准备好。";
      if (state.challengeIndex < challenges.length - 1) setTimeout(() => { state.challengeIndex += 1; updateChallenge(); }, 950);
    } else $("#feedback").textContent = `再试一次：第一行算 x′，第二行算 y′。`;
    return { correct, expected: correct ? expected : undefined, challenge: state.challengeIndex + 1 };
  }

  matrixInputs.forEach((input, i) => input.addEventListener("input", () => {
    state.matrix[i] = parse(input.value, state.matrix[i]); state.progress = 1;
    $$("[data-preset]").forEach((button) => button.classList.remove("selected")); updateUI();
  }));
  vectorInputs.forEach((input, i) => input.addEventListener("input", () => {
    state.vector[i] = parse(input.value, state.vector[i]); state.progress = 1; updateUI();
  }));
  $$(".lesson-item").forEach((button) => button.addEventListener("click", () => selectLesson(button.dataset.lesson)));
  $$("[data-preset]").forEach((button) => button.addEventListener("click", () => {
    const presets = { rotate: [0.866,-0.5,0.5,0.866], scale: [1.5,0,0,1], shear: [1,.8,0,1], reflect: [-1,0,0,1] };
    setTransform(presets[button.dataset.preset], state.vector, { preset: button.dataset.preset });
  }));
  $("#identityButton").addEventListener("click", () => setTransform([1,0,0,1], state.vector));
  $("#resetAll").addEventListener("click", () => { state.completed.clear(); state.challengeIndex = 0; updateChallenge(); selectLesson("vector"); });
  $("#playAnimation").addEventListener("click", play);
  $("#progressSlider").addEventListener("input", (event) => {
    cancelAnimationFrame(state.animation); $("#playAnimation").classList.remove("playing");
    state.progress = Number(event.target.value) / 100; updateUI();
  });
  $("#checkAnswer").addEventListener("click", checkAnswer);
  [$("#answerX"), $("#answerY")].forEach((input) => input.addEventListener("keydown", (event) => { if (event.key === "Enter") checkAnswer(); }));

  function pointerVector(event) {
    const rect = canvas.getBoundingClientRect(), scale = Math.max(34, Math.min(54, Math.min(rect.width, rect.height) / 8.8));
    return [clamp((event.clientX - rect.left - rect.width / 2) / scale, -5, 5), clamp(-(event.clientY - rect.top - rect.height / 2) / scale, -5, 5)];
  }
  canvas.addEventListener("pointerdown", (event) => { state.dragging = true; canvas.setPointerCapture(event.pointerId); $("#dragHint").classList.add("hidden"); });
  canvas.addEventListener("pointermove", (event) => {
    if (!state.dragging) return; state.vector = pointerVector(event).map((n) => Math.round(n * 10) / 10); state.progress = 1; updateUI({ force: true });
  });
  canvas.addEventListener("pointerup", () => { state.dragging = false; });
  canvas.addEventListener("pointercancel", () => { state.dragging = false; });
  window.addEventListener("resize", draw);

  function registerWebMCP() {
    if (!document.modelContext?.registerTool) return;
    const register = (tool) => Promise.resolve(document.modelContext.registerTool(tool)).catch(() => {});
    register({
      name: "set_transformation", title: "设置矩阵变换",
      description: "设置二维矩阵和输入向量，并立即更新可视化结果。",
      inputSchema: { type: "object", properties: {
        matrix: { type: "array", minItems: 4, maxItems: 4, items: { type: "number", minimum: -10, maximum: 10 } },
        vector: { type: "array", minItems: 2, maxItems: 2, items: { type: "number", minimum: -10, maximum: 10 } }
      }, required: ["matrix", "vector"], additionalProperties: false },
      annotations: { readOnlyHint: false, untrustedContentHint: false },
      execute(input) {
        if (!input || !Array.isArray(input.matrix) || input.matrix.length !== 4 || !Array.isArray(input.vector) || input.vector.length !== 2 || ![...input.matrix, ...input.vector].every(Number.isFinite)) throw new Error("需要 4 个矩阵数值和 2 个向量数值");
        setTransform(input.matrix, input.vector, { animate: false });
        return { matrix: state.matrix, vector: state.vector, result: multiply() };
      }
    });
    register({
      name: "open_lesson", title: "打开课程",
      description: "打开指定的矩阵课程，并载入对应的示例。",
      inputSchema: { type: "object", properties: { lesson: { type: "string", enum: ["vector", "scale", "rotate", "shear"] } }, required: ["lesson"], additionalProperties: false },
      annotations: { readOnlyHint: false, untrustedContentHint: false },
      execute(input) { if (!lessonData[input?.lesson]) throw new Error("未知课程"); selectLesson(input.lesson); return { lesson: input.lesson, matrix: state.matrix, vector: state.vector }; }
    });
  }

  updateChallenge(); updateUI({ force: true }); registerWebMCP();
})();
