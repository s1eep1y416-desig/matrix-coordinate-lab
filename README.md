# Matrix Coordinate Lab · 三维机器人学实验室

通过拖动坐标系、改变旋转角和关节角，把矩阵计算与三维空间运动对应起来。面向正在学习 ROS2、TF、URDF 和机械臂控制的人，不是单纯的公式计算器。

[在线实验室（需访问权限）](https://matrix-coordinate-lab.s1eep1y416.chatgpt.site/) · [reBot URDF 交接清单](docs/URDF_INPUT.md)

## 当前可以做什么

| 模块 | 已实现的交互 |
| --- | --- |
| 坐标系与变换 | World / Frame A；添加任意层级子系、修改父级、显示与隐藏；拖动原点及轴端；精确输入位姿；自由度锁定与限位；相对变换、逆变换和逐级播放的父子矩阵链。 |
| 点坐标转换 | 固定空间点 P，切换源 / 目标坐标系，实时显示坐标与变换矩阵。 |
| 旋转矩阵 | 独立调节 Rx / Ry / Rz；比较 XY / YZ / ZX 两种乘法次序；播放、暂停、拖动进度，查看第一步及最终结果。 |
| 主动 / 被动旋转 | 主动模式旋转金色向量；被动模式保持世界向量不动，显示其在旋转参考系中的坐标。 |
| 旋转表示 | 六种 Euler 顺序、ZYX / RPY、四元数、Axis-Angle；切换顺序时可保留姿态或保留角度；万向节锁提示。 |
| 3-Link FK | Z-Y-Y 三关节教学骨架；末端位置与姿态；逐级显示矩阵链。 |
| 位置 IK | 解析位置 Jacobian、阻尼最小二乘、下降线搜索；单步迭代与自动求解；可拖动目标点。 |
| Pinocchio 流程 | 以相同的教学骨架解释 URDF → Model → q → FK → Frame placements；目前不是实际 Pinocchio 运行环境。 |

页面沿用白绿操作区、黑金场景，坐标轴保持 X 红、Y 绿、Z 蓝。

## 本地运行

需要 Node.js **20.19+（20.x）或 22.12+**，推荐 22.12+，以及支持 WebGL 的浏览器。

```bash
npm ci
npm run dev
```

打开终端显示的本地地址，默认是 `http://127.0.0.1:5173/`。

```bash
npm test         # 数学与状态回归测试
npm run build   # TypeScript 检查 + 生成 dist/
```

技术栈：React 19、TypeScript、Three.js、React Three Fiber、Drei、Zustand、KaTeX、Vite、Vitest。

## 操作方式

- **中键拖动**：旋转视角；**滚轮**：缩放；右上角可重置视角。
- **左键拖动子坐标系原点**：在当前视图平面内平移；拖动彩色轴端：改变姿态；拖动 P / IK Target：移动空间点。
- **旋转数值左右拖动**：调节角度；按住 Shift 微调；点击输入框可以输入精确值，按 Enter 或失焦提交。
- **右侧操作栏滚轮**：上下滚动参数；窄屏时操作栏排在场景下方。
- **旋转演示**：选择轴对后，从头播放或点击「初始 / 第 1 步完成 / 最终结果」，也可以拖动进度条。调节目标角度或切换模块会暂停播放。
- **自由度与限位**：展开面板，为每个坐标系分别设置平移 / Euler 分量范围与锁定状态。
- **变换链回放**：选中任意深度的子坐标系，在「逐级查看变换链」中点击 World、A、B……；金色路径、局部矩阵和累计矩阵同步推进。

演示角度和场景状态仅保存在当前页面内存中，刷新后恢复默认值。

## 数学约定：先看这里

### 参考系、单位与符号

- 使用**右手坐标系**，世界 **Z 向上**；长度为米。
- 界面输入角度使用 **度**；三角函数和 Jacobian 关节增量使用 **弧度**。
- 使用**列向量**，矩阵乘积右侧先作用。
- `^A T_B` 表示 **Frame B 相对于 Frame A 的位姿**，把 B 中的点坐标映射到 A 中。
- Pose 的真值是 `position: Vector3` 和单位 `quaternion: Quaternion`。Euler、旋转矩阵和 Axis-Angle 从四元数推导，不作为独立姿态真值同步保存。
- 四元数顺序为 `[x, y, z, w]`；`q` 与 `-q` 表示相同姿态。
- Three.js 内部按列存储矩阵；页面按通常的行列布局显示，不把存储顺序当成转置。

```text
p_A = R_A_B · p_B + position_A_B
^A T_B = [ R_A_B  position_A_B ]
         [ 0 0 0       1      ]
^W T_B = ^W T_A · ^A T_B
^A T_W = inverse(^W T_A)
inverse([R p; 0 1]) = [Rᵀ  -Rᵀp; 0 1]
```

注意逆变换的平移项是 **−Rᵀp**，不是单纯的 −p。切换描述 P 的坐标系时，P 的世界位置不变。

### 正负号与旋转顺序

令 `c = cos(θ)`、`s = sin(θ)`，主动旋转矩阵为：

```text
Rx = [1  0  0]    Ry = [ c  0  s]    Rz = [c -s  0]
     [0  c -s]         [ 0  1  0]         [s  c  0]
     [0  s  c]         [-s  0  c]         [0  0  1]
```

独立的 ±90° 检查：

| 输入向量 | +90° 主动旋转 | −90° 主动旋转 |
| --- | --- | --- |
| Rx 作用于 +Y | +Z | −Z |
| Ry 作用于 +X | **−Z** | +Z |
| Rz 作用于 +X | +Y | −Y |

正角按右手定则定义，不按当前屏幕上的顺 / 逆时针定义。尤其注意，绕 +Y 正转时，+X 朝 −Z 运动并不是符号错误。

主动旋转使用 `v′ = Rv`。被动旋转把参考系旋转 R，保持世界向量不变，坐标变成 `v_local = Rᵀv_world`。因此，参考系绕 Z 转 +90° 后，固定的世界 +X 向量在新参考系中的坐标是 `[0, -1, 0]`。

旋转比较页的 `RxRy` 表示**先绕固定 Y 轴，再绕固定 X 轴**；与 `RyRx` 比较时，两组原点只是错开放置方便观察，不把展示偏移加入旋转计算。进度 `0 → 1` 执行右侧矩阵，`1 → 2` 执行左侧矩阵。

Euler 采用 Three.js 的内禀序列约定。三个输入始终对应 X / Y / Z 分量；ZYX / RPY 对应 `Rz(yaw) Ry(pitch) Rx(roll)`，也等价于固定轴依次 X → Y → Z。不要混淆序列名称、矩阵书写顺序与固定轴执行顺序。

Euler 表示不唯一，在 ±180° 边界或万向节锁附近，读数可能跳到等价分支；Axis-Angle 使用 0–180° 角度及相应轴方向，因此负角可能显示为「反向轴 + 正角」。这些不代表空间姿态反向。

### 教学机械臂与 IK

当前骨架为 Z-Y-Y 旋转关节，连杆沿各自局部 +X 伸出，长度依次为 `1.25 / 1.00 / 0.80 m`。每段变换为先旋转再沿旋转后的 +X 平移。用于说明递推计算，并非真实 reBot 的 URDF 坐标系定义。

```text
T_base_tool = T_base_link1 · T_link1_link2 · T_link2_link3
```

零位末端为 `[3.05, 0, 0]`。单独将 q2 改为 +90°，末端为 `[1.25, 0, -1.80]`；改为 −90°，末端为 `[1.25, 0, +1.80]`。

位置 IK 的符号约定为：

```text
e = p_target − p_current
J_i = axis_i × (p_end − p_joint_i)   # 全部表达在 base 中
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
q_next = q + Δq
```

## 代码结构

```text
src/
├── math/                       # 与 React / 3D 渲染分离的计算
│   ├── rotation.ts             # 单轴旋转、次序比较、主动/被动向量、合法性验证
│   ├── euler.ts                # 六种 Euler 顺序
│   ├── quaternion.ts           # 单位化、Axis-Angle、姿态等价
│   ├── transform.ts            # Pose、连乘、逆变换、坐标转换
│   ├── kinematics.ts           # 3-Link FK
│   ├── inverseKinematics.ts    # Jacobian / DLS 位置 IK
│   ├── math.test.ts
│   └── signs.test.ts           # 独立解析公式与正负号检查
├── components/
│   ├── CoordinateFrame.tsx     # RGB 坐标轴、透明文字与拖拽
│   ├── RotationScene.tsx       # A/B 旋转和向量的对比场景
│   ├── RotationPlayback.tsx    # 播放、暂停、分步与进度控制
│   ├── Robot.tsx               # 教学机械臂骨架
│   ├── Scene.tsx               # 相机、点、场景组合
│   ├── MathView.tsx            # 矩阵、公式与数值
│   └── NumberField.tsx         # 精确输入与拖动数值
├── stores/
│   ├── labStore.ts             # 位姿、层级、限制与交互状态
│   └── labStore.test.ts
├── App.tsx
└── styles.css
docs/URDF_INPUT.md              # 后续真实模型所需资料
```

## 验证与当前边界

`npm test` 覆盖六种 Euler 往返、四元数 / 矩阵往返、`q ≡ −q`、`T · inverse(T) ≈ I`、多级变换链、点坐标转换、FK、解析 Jacobian 与有限差分、IK 误差下降，以及限位和播放状态。正负号测试另用显式 sin / cos 矩阵和已知端点验证，避免只做自洽的往返测试。

页面实时检查 `RᵀR ≈ I`、`det(R) ≈ 1`、单位四元数和齐次末行，采用浮点容差而不是严格相等；常用合法性容差为 `1e-8`。显示时会舍入，计算仍使用完整精度。

已知边界：

- 当前 **没有真实 URDF 导入、Mesh 加载或 Pinocchio Python/WASM 运行时**。Pinocchio 页是计算流程教学，计算来自本项目 TypeScript 模型。
- IK 仅求解三维位置，不求完整六维末端位姿；是局部迭代方法，受初值、奇异性、限位与不可达目标影响，不保证收敛。最大臂展球壳只是外边界，壳内不代表必然可达。
- 坐标系旋转限位限制当前 Euler 表示的分量，不等同于 URDF 的独立单轴关节限位；Euler 奇异与等价分支仍需注意。
- 当前 Gimbal Lock 是数值提示，尚未实现独立的轴线重合教学动画。SLERP 对比、真实六轴 FK / 位姿 IK、夹爪及 tool/world 运动对比仍待扩展。
- 尚未做碰撞检测、动力学或真实机械臂控制。不要将教学数值直接用于硬件执行。

## 后续接入 reBot

先提供实际 `base_link`、六个运动 Link、`tool0`、`joint1–joint6`，以及安装位姿、关节轴、限位、Mesh 和一组已知姿态。夹爪按真实结构补充 Link / Joint / mimic。完整说明见 [URDF 交接清单](docs/URDF_INPUT.md)。

收到模型后再按实际拓扑建立运动学链，对照已知末端位姿检查轴向、正负号和单位，然后扩展六轴 FK、位姿 IK 与 Pinocchio 结果对照。

## 发布与隐私

本站使用 Sites 托管，项目配置在 `.openai/hosting.json`，静态构建输出为 `dist/`。网站访问权限与 GitHub 仓库可见性是两套独立设置；当前均按私有方式维护。发布时只使用正式构建产物，不包含临时截图、凭证或本地环境文件。
