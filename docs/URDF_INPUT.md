# reBot URDF 交接清单

网站将按你提供的真实 URDF 重建 Link、Joint、TF、FK 与 IK。名称可以不同，但请在交付时指出哪个是根 Link、哪个是末端工具坐标系。

## 必需的 Link

| 作用 | 建议名称 | 说明 |
| --- | --- | --- |
| 机器人根 | `base_link` | 六轴链的起点；`world` 可以由外部 TF 连接，不必是 URDF Link。 |
| 六段运动链 | `link1` 至 `link6` | 每个运动关节的 child Link。请按真实机械结构命名。 |
| 末端坐标系 | `tool0` 或 `end_link` | 与第六轴之间一般使用 fixed Joint；请标注 TCP 是否与此重合。 |
| 夹爪（如需显示） | `gripper_base`、`left_finger`、`right_finger` 等 | 有哪些零件就提供哪些 Link。 |

## 必需的 Joint

| 作用 | 建议名称与连接 | 必需字段 |
| --- | --- | --- |
| 六个运动关节 | `joint1`: `base_link` → `link1`，依次到 `joint6`: `link5` → `link6` | `type`，`parent`，`child`，`origin xyz/rpy`，`axis xyz`，角度限位。 |
| 末端固定连接 | `tool0_fixed`: `link6` → `tool0` | `type="fixed"`，`parent`，`child`，`origin xyz/rpy`。 |
| 夹爪连接（如有） | 夹爪基座与手指对应的 fixed / prismatic / revolute Joint | 同样提供父子关系、安装位姿、运动轴及限位；耦合手指注明 mimic。 |

每个运动关节请提供准确的零位定义、正方向、轴方向和 `limit lower/upper/velocity/effort`。`origin xyz/rpy` 是父 Link 到关节坐标系的固定位姿，单位为米和弧度；`axis xyz` 在关节坐标系中表达。所有坐标系使用右手系。

## 一并交付

- 完整 `.urdf` 或 `.xacro` 文件；如果是 Xacro，也请给出可展开的 URDF 或必要的宏与参数文件。
- URDF 引用的所有 mesh 文件及其目录结构、比例（`scale`）；只做 FK/IK 时 mesh 可以先不提供，做 3D 外观时需要。
- 明确指定末端求解 Frame（例如 `tool0` / `end_link`）和 TCP 相对它的固定偏移。
- 一组已知关节角及对应的末端位姿，或至少一张零位姿态参考图，方便交叉核对坐标方向。
- 如果需要使用真实关节限位和夹爪动作，请附驱动层对关节的命名映射。

视觉外观需要 `<visual>`；碰撞检测需要 `<collision>`；质量动力学需要 `<inertial>`。本阶段的坐标变换、FK、位置/位姿 IK 首先依赖关节拓扑、安装位姿、运动轴、限位与工具坐标系。
