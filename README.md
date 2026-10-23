# 取块机构（catch_block）调参手册

本文只讲**怎么调**，不讲代码怎么分层。目标是让你在 Keil 的 Debug 模式下，把一台刚装好的升降机构从"完全没标定"调到"能稳定托住 3.3 kg 并走完 395 mm"。

---

## 1. 文件构成

| 文件 | 作用 |
|---|---|
| `catch_block.hpp/.cpp` | 状态机、命令入口、故障策略、`catch_debug` 调试窗口 |
| `catch_block_driver.hpp/.cpp` | 电机 + 两路继电器 + 零点 + 米↔弧度换算 |
| `catch_block_config.hpp/.cpp` | 28 个可调参数与两段校验 |

入口只有一个：`MainFrame.cpp` 里的 `System.RegistApp(CatchBlock::GetInstance());`

---

## 2. 先算清楚这台机构的三个数

调参前必须知道下面三个换算，否则看到 `rad` 不知道对应多少毫米：

```
1 rad（转子）  = 0.025 × 2 ÷ 19.2032 = 2.6037 mm
1 圈（转子）   = 6.2832 rad = 16.36 mm
满行程 395 mm  = 151.7 rad
```

反过来，速度与转速的对照：

| 机构速度 | 转子角速度 | 转子转速 |
|---|---|---|
| 10 mm/s | 3.84 rad/s | 36.7 rpm |
| **20 mm/s**（默认规划速度） | 7.68 rad/s | 73.4 rpm |
| **30 mm/s**（默认超速门限） | 11.52 rad/s | 110.0 rpm |

静态承载电流（这是最容易踩的坑）：

| 负载 | 理想电流 | 按 80% 效率 |
|---|---|---|
| 3.3 kg（带块） | 5.39 A | **6.74 A** |
| 2.6 kg（卸块后） | 4.25 A | 5.31 A |

> **`max_current_A` 必须大于 6.8 A**，否则机构在顶端根本托不住。默认值 `0` 会被校验拒绝，这是有意的——逼你先算一遍。

---

## 3. 调试窗口 `catch_debug`

在 Keil 的 Watch 窗口输入 `catch_debug`，展开后就是全部可写/可读量。

### 3.0 在 Keil 里的操作要点

1. `Ctrl+F5` 进入 Debug，`F5` **全速运行**——调参全程都要在运行状态。
2. `View → Watch Windows → Watch 1`，在 `<Enter expression>` 里输入 `catch_debug`，回车展开。
3. 勾上 **Periodic Window Update**（View → Periodic Window Update），否则数值不刷新。
4. 写值：双击 Value 列，输入新值回车。**只有在 Run 状态下写才会被程序看到**。
5. **不要在 halt 状态下让机构带载悬空**：CPU 停了，但 C620 会继续执行最后一条电流指令，
   而看门狗/超时保护也已经不跑了。
6. 如果某个量显示 `<not in scope>` 或值不变，检查是否编译成了 `-O2`；本工程的调试量都是
   `volatile`，`-O0`/`-O1` 下都能正常观察。

### 3.1 可写

| 路径 | 含义 |
|---|---|
| `catch_debug.config.<字段>` | 28 个参数，含义见第 6 节 |
| `catch_debug.command` | 命令号，写后下一拍自动清 0 |
| `catch_debug.target_height_m` | 命令 3 的目标高度（米，0~0.395） |

另有三个**运行开关**（不在 `catch_debug` 里，是独立全局量）：

| 名字 | 含义 |
|---|---|
| `catch_pump_on` | 非 0 → PE6 输出高，气泵吸附 |
| `catch_valve_closed` | 非 0 → PE12 输出高，夹爪闭合 |
| `catch_motor_enable` | 非 0 → 允许电机输出 |

> `catch_motor_enable = 0` 是**撤力**，不是刹车。没有外部支承时不要靠它"停住"。

### 3.2 命令号

| 号 | 动作 | 前置条件 |
|---|---|---|
| 1 | 配置（参数取自 `catch_debug.config`） | 电机未使能、参数通过校验 |
| 2 | 把当前位置标为 0 m | 已配置、反馈健康、机构静止 |
| 3 | 移动到 `target_height_m` | 状态为 OriginHold 或 ManualHold、已使能 |
| 4 | 以当前位置保持 | 状态为 OriginHold / ManualMoving / ManualHold |
| 5 | 清故障、清反馈锁存、回到未配置 | 代码不设条件；操作前提是机构有支承（零点会丢） |
| 6 | 软急停（保持式停车，不松力） | 电机已使能 |

`catch_debug.result` 回读执行结果：

| 值 | 含义 |
|---|---|
| 0 | 还没执行 |
| 1 | 已接受（**不代表动作成功**，要看 `state`） |
| 2 | 被拒绝（条件不满足，看第 9 节） |
| 3 | 执行后处于故障态 |

### 3.3 可读

| 路径 | 含义 |
|---|---|
| `catch_debug.state` | 状态号，见 3.4 |
| `catch_debug.faults` | 应用故障 \| 驱动故障（位定义见 3.5，两套位有重叠） |
| `catch_debug.actual_height_m` | 相对原点的高度（m） |
| `catch_debug.feedback_valid` | 1 = 反馈有效 |
| `catch_debug.motor_enabled` | 1 = 驱动正在输出 |
| `catch_debug.pump_on` / `valve_closed` | 两路引脚**实际**电平 |
| `catch_debug.target_position_rad` / `measured_position_rad` | 目标 / 实测转子角 |
| `catch_debug.reference_position_rad` / `reference_velocity_rad_s` | TD 参考轨迹（v1/v2） |
| `catch_debug.eso_position_rad` / `eso_velocity_rad_s` / `disturbance_rad_s2` | ESO 的 z1/z2/z3 |
| `catch_debug.raw_current_A` / `limited_current_A` / `final_current_A` | ADRC 原始 / 限幅后 / 实际下发 |
| `catch_debug.accepted_command_A` | CAN 邮箱已接受，不是实测电流 |
| `catch_debug.feedback_current_raw` / `feedback_current_A_valid` | 反馈电流原始码（未标定时不能当安培用） |
| `catch_debug.temperature_C` | 电调温度 |

### 3.4 状态号对照

| 值 | 状态 | 含义 |
|---|---|---|
| 0 | Unconfigured | 未配置 |
| 1 | Idle | 已配置、未标零 |
| 2 | OriginHold | 原点保持（调参主战场） |
| 3 | GripClose | 夹爪闭合中 |
| 4 | SuctionOn | 气泵建立中 |
| 5 | GripVerify | 等人工确认夹牢 |
| 6 | Lifting | 上升中 |
| 7 | TargetHold | 顶端保持 |
| 8 | ReleaseReady | 许可已给，复核一拍 |
| 9 | Release | 允许投放 |
| 10 | Descending | 下降中 |
| 11 | ManualMoving | 调试位移中 |
| 12 | ManualHold | 调试保持中 |
| 13 | FaultHold | 故障但仍在保持（有危险，别久留） |
| 14 | Fault | 故障且已断驱动 |

### 3.5 故障位（两套有重叠，注意）

`catch_debug.faults` 是**应用故障与驱动故障按位或**的结果，两套位定义不同但都从 bit0 起：

| bit | 应用侧（CatchBlock::Fault） | 驱动侧（MotorDJI::FaultBits） |
|---|---|---|
| 1 | Configuration | ConfigFault |
| 2 | Feedback | FeedbackFault |
| 4 | Controller | TimingFault |
| 8 | Timing | NumericFault |
| 16 | Timeout | PositionLimitFault |
| 32 | Travel | TransmitFault |
| 64 | CanTransmit | OverspeedFault |
| 128 | OperatorHold | — |

判断方法：先看 `catch_debug.state`。处于 `Fault`/`FaultHold` 时多半是应用侧；`feedback_valid` 突然变 0 且 bit2/bit64 亮，多半是驱动侧的超速锁存（见 8.2）。

---

## 4. 调参总原则

1. **先方向、再承载、后动态，顺序不能反。** 方向错了任何闭环都会飞车。
2. **同一个参数改完只做一件事**：改完 → 命令 5 复位 → 命令 1 重配 → 命令 2 重标零 → 再测。`config` 只在配置那一刻被采纳，改了不重配是无效的。
3. **每次加电前先确认 `catch_motor_enable = 0`。**
4. **第一次使能时机构必须被支承或悬空**，因为当前版本没有重力前馈（见 8.1）。

---

## 5. 分阶段调参流程

### 阶段 0：继电器通断（不需要配置）

1. `catch_motor_enable = 0`
2. `catch_pump_on = 1` → 听继电器"嗒"、看模块 LED、气泵出气
3. `catch_pump_on = 0`，`catch_valve_closed = 1` → 夹爪动作
4. 同时观察 `catch_debug.pump_on` / `valve_closed` 是否跟随（读的是引脚实际电平）

不通过就别往下走：检查继电器是 5 V 还是 3.3 V 触发、是低电平有效还是高电平有效、接的是 NO 还是 NC。

### 阶段 1：方向与零点

**前提：机构被支承或悬空，负载取下。**

1. 填 `catch_debug.config`：`esc_id`、`lift_encoder_sign`、`current_encoder_sign`（先随便填 ±1）、四个 `*_verified` 设 1、`max_current_A = 8`、其余按第 6 节起始值。
2. `command = 1` → 期望 `result = 1`、`state = 1`。
   - `result = 2`：参数没通过校验，逐项核对第 6 节的前置条件。
   - `result = 3`：`state` 会变 14，硬件初始化失败（多半是 `esc_id` 写错或 CAN 没通）。
3. `command = 2` → 期望 `state = 2`（OriginHold）。此时电机**还没使能**，标零只是读编码器。
4. `catch_motor_enable = 1` → 下一拍 `catch_debug.motor_enabled` 应变成 1。
5. `target_height_m = 0.01`，`command = 3` → 机构应向上走 10 mm，状态 11 → 12。
6. **看方向**：`actual_height_m` 应该是 **+0.01**。
   - 如果是 −0.01 或不动：`lift_encoder_sign` 取反，回第 2 步重来。
   - 如果机构**向下冲**或剧烈抖动：`current_encoder_sign` 反了，立刻 `catch_motor_enable = 0`，取反后回第 2 步。

> 方向验证这一步只要错一次就可能撞限位或飞车，务必先让机构悬空、行程留足余量。

### 阶段 2：静态承载（最关键）

1. 装 3.3 kg 负载，落到原点，命令 2 重新标零。
2. `catch_motor_enable = 1`，`command = 3` 走到 `target_height_m = 0.2`（半程）。
3. 稳定后读三个量：

| 观测量 | 期望 | 说明 |
|---|---|---|
| `final_current_A` | 5.4 ~ 6.8 A | 静态托举电流；明显偏小说明 Kt 填大了 |
| `disturbance_rad_s2`（z3） | 约 −2600 | 重力扰动的等效加速度；量级不对说明 `inertia_kg_m2` 或 `torque_constant_Nm_A` 错了 |
| `actual_height_m` | 稳定在 0.2 ±0.001 | 缓慢下沉说明电流不够 |

4. 若托不住：先把 `max_current_A` 加到 9~10 A 试，仍然托不住就回头核对 `torque_constant_Nm_A`。
5. 若抖动/啸叫：把 `current_slew_A_s` 从 2000 降到 800，再把 `controller_Hz` 从 4 降到 2.5。

### 阶段 3：速度与加速度规划

| 参数 | 起始 | 调整方向 |
|---|---|---|
| `speed_m_s` | 0.020 | 想快就加，但必须让 `overspeed_m_s` 保持 ≥ 1.2 倍 |
| `acceleration_m_s2` | 0.040 | 加太猛会激发 ESO 的 z3 振荡；配合 `current_slew_A_s` 一起调 |
| `overspeed_m_s` | 0.030 | **不要贴着 `speed_m_s` 设**，见 8.2 |

改完重配 → 重标零 → 命令 3 走全程，看 `reference_velocity_rad_s`（TD 的 v2）是否平滑、`eso_velocity_rad_s`（z2）是否紧跟。z2 明显滞后于 v2 说明 `observer_Hz` 偏低。

### 阶段 4：ADRC 带宽

| 参数 | 起始 | 说明 |
|---|---|---|
| `observer_Hz` | 20 | ESO 带宽。低 → 扰动估计慢、抗扰差；高 → 噪声放大、电流抖 |
| `controller_Hz` | 4 | 位置环带宽。必须 **小于** `observer_Hz`，建议 1/5 关系 |

调法：先固定 `controller_Hz`，把 `observer_Hz` 从 10 往上加，直到 `final_current_A` 开始明显抖，退回到抖动前一档；再调 `controller_Hz`，从 2 往上加，直到阶跃（命令 3 改个小目标）出现超调，退回一档。

`viscous_Nm_s_rad` 保持 0，让 ESO 自己吸收摩擦；只有在 z3 长期饱和时才考虑填一个正数。

---

## 6. 参数速查表

### 6.1 必填（填 0 会被校验拒绝）

| 字段 | 单位 | 起始值 | 说明 |
|---|---|---|---|
| `esc_id` | — | 1 | C620 编号 1~8，反馈 ID = 0x200+id |
| `lift_encoder_sign` | ±1 | ±1 | 高度上升时编码器累计值的方向 |
| `current_encoder_sign` | ±1 | ±1 | 正电流让转子加速的方向 |
| `inertia_kg_m2` | kg·m² | 3.2e-5 | 折算到转子轴的整机惯量 |
| `torque_constant_Nm_A` | N·m/A | 0.0156 | **转子域**，手册的 0.3 是输出轴值，要除以 19.2032 |
| `max_current_A` | A | 8 | 必须 > 6.8 A 才托得住 3.3 kg；上限 20 A 被校验卡死 |
| `current_slew_A_s` | A/s | 2000 | 太小托不住、太大冲击 |
| `observer_Hz` | Hz | 20 | ESO 带宽 |
| `controller_Hz` | Hz | 4 | 位置环带宽，须 < `observer_Hz` |

### 6.2 四个闸门

`direction_verified`、`model_verified`、`current_limits_verified`、`motion_limits_verified`
——四项全为 1 才允许配置。它们不是开关，是**你签字的记录**：没实测过就别置 1。

### 6.3 几何与规划

| 字段 | 默认 | 说明 |
|---|---|---|
| `travel_m` | 0.395 | 原点至顶端 |
| `pulley_radius_m` | 0.025 | 同步带轮有效节圆半径 |
| `external_speed_ratio` | 2.0 | 带轮转速是减速箱输出的 2 倍 |
| `gearbox_ratio` | 3591/187 ≈ 19.2032 | M3508 减速比，不要改 |
| `speed_m_s` / `acceleration_m_s2` / `overspeed_m_s` | 0.02 / 0.04 / 0.03 | 见阶段 3 |

### 6.4 工艺阈值

| 字段 | 默认 | 说明 |
|---|---|---|
| `position_tolerance_m` | 0.001 | 到位判定 |
| `settled_speed_m_s` | 0.001 | 到位判定的速度门限 |
| `travel_fault_margin_m` | 0.003 | 软限位余量（比容差只大 3 倍，偏紧） |
| `feedback_timeout_us` | 10000 | 2000~20000 |
| `settle_ms` | 500 | 到位需持续稳定多久 |
| `relay_dwell_ms` | 300 | 夹爪/气泵动作间隔 |
| `phase_timeout_ms` | 45000 | 单阶段超时；20 mm/s 走完全程约 20 s，够用 |

---

## 7. 观测量怎么读

调 ADRC 时按这条链看，一眼就能定位是哪一段出了问题：

```
目标高度 ──► TD(v1,v2) ──► 位置环 ──► ADRC 输出(raw_current_A) ──► 限幅 ──► 斜坡 ──► final_current_A
                    ▲                                        │
                    └────────── ESO(z1,z2,z3) ◄── 反馈 ───────┘
```

| 现象 | 说明 |
|---|---|
| `reference_velocity_rad_s` 是方波 | `acceleration_m_s2` 设太大，TD 没起到平滑作用 |
| `eso_velocity_rad_s` 长期滞后 `reference_velocity_rad_s` | `observer_Hz` 偏低 |
| `disturbance_rad_s2` 持续饱和（±几万） | `inertia_kg_m2` 或 `torque_constant_Nm_A` 错，b0 不对 |
| `raw_current_A` 正常但 `final_current_A` 明显小 | 被 `max_current_A` 限住了 |
| `final_current_A` 与 `accepted_command_A` 差很多 | CAN 发送未被接受，看 `faults` 的 bit64 |
| `measured_position_rad` 不动但 `final_current_A` 很大 | 电机没使能 / 机械卡死 / 电流方向反 |

---

## 8. 已知限制与坑

### 8.1 没有重力前馈（最重要）

`ADRC::Reset()` 会把 z3 清零，而 z3 收敛需要约 `3/(2π·observer_Hz)`（20 Hz 下约 **24 ms**）。
在这 24 ms 里控制器完全"不知道"有重力，输出只有位置误差项。

**首次使能时机构会自由下坠：约 4.4 ms 后转子就到 110 rpm，帧级跳变检查锁存 `AngleJump`，反馈永久失效。**
（4.4 ms 而不是 3 ms，是因为 `inertia_kg_m2` 里含折算负载，等效加速度约 6.9 m/s² 而非 9.8。）

> 危险的不是位移——自由下坠 4.4 ms 只走 **0.07 mm**，机械上几乎看不出来。
> 危险的是**反馈被锁存**：`feedback_valid` 变 0 后整个闭环失效，只能 `command = 5` 复位。
> 所以你会看到"刚使能，机构没动，但已经故障了"，这不是硬件坏了。

对策：
- 使能时机构必须有支承或悬空，别让它在顶端直接使能；
- 一旦锁存，`feedback_valid` 变 0、`faults` 亮 bit2/bit64，必须 `command = 5` 复位后才能重来。

### 8.2 `overspeed_m_s` 同时是"帧级跳变门限"

这一个值同时用于 `ConfigureFeedbackGuard()`（帧级跳变/速度检查）和 `SpdLimit()`。
只要有一帧超过它，就会锁存 `AngleJump`，**反馈永久失效直到复位**。
所以 `overspeed_m_s` 至少要是 `speed_m_s` 的 1.2~1.5 倍，别贴着设。

### 8.3 哪些故障会直接撤驱动

`Fail(fault, can_hold)` 的第二个参数决定"托住"还是"松手"。按当前实现：

| 是否撤驱动 | 触发点 |
|---|---|
| **撤驱动**（can_hold = false） | 配置失败、`Timing`（应用周期异常）、`CanTransmit`（CAN 未接受）、`CheckMotion` 里的 `Controller\|Feedback`、`CheckMotion` 里的 `Travel`（越界） |
| 保持（can_hold = true） | `SoftStop` 的 `OperatorHold`、阶段超时 `Timeout`、`ApplyHeight` 下发目标失败的 `Travel`、`ReleaseReady` 复核失败的 `Controller` |

也就是说：**运行中检查（`CheckMotion`）发现的异常一律撤驱动**，因为那时已经无法确认闭环可信；
而流程层面的失败优先尝试保持。撤驱动对重力负载等于松手，这几类故障发生时必须有外部支承。

### 8.4 改 `esc_id` 必须先复位

`Configure()` 拒绝在已配置状态下更换 ESC 编号。要换编号，先 `command = 5`。

### 8.5 参数只在"配置那一刻"生效

`catch_debug.config` 是输入面板，改完不执行 `command = 1` 不会生效。
而 Keil 写内存不受中断屏蔽保护，命令 1 有可能读到半新半旧的参数——**填完先读一遍确认，再写 command**。

### 8.6 没有机械防坠、没有夹取传感器

文中所有"保持"都依赖驱动器持续输出电流。断电、故障撤力、驱动器过热都会掉负载。
夹爪是否夹牢、物块是否离开，全部靠人工确认（对应 `GripVerify` / `Release` 两个状态的命令）。

---

## 9. 常见问题

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `command = 1` 返回 2 | 有必填项为 0、符号不是 ±1、`controller_Hz ≥ observer_Hz`、`max_current_A > 20`、四个 verified 没置 1 | 逐项对照 6.1 / 6.2 |
| `command = 1` 返回 3，state 变 14 | `esc_id` 错、CAN 未通、C620 未上电 | 查接线与编号；改完 `command = 5` 再试 |
| `command = 2` 返回 2 | 状态不是 Idle（可能还在 Fault）、机构在动（速度超 `settled_speed_m_s`）、反馈不健康 | 先 `command = 5`，等机构静止 |
| `command = 3` 返回 2 | 状态不是 OriginHold/ManualHold、未使能、目标超范围 | 先使能（`catch_motor_enable = 1`） |
| 使能后立刻 state=13 或 14 | 重力前馈缺失导致超速锁存（8.1） | `command = 5`，改为有支承时使能 |
| 高度一直不动，但电流很大 | 电机未使能 / 机械卡死 / `current_encoder_sign` 反 | 看 `motor_enabled` 与 `final_current_A` |
| 高度读数跳变 | 编码器方向反、`lift_encoder_sign` 错、反馈锁存 | 查 `feedback_valid` 与 `faults` |
| 走到一半报 Travel | 零点或换算系数错（正常路径碰不到越界） | 重新标零；核对 `pulley_radius_m`/`gearbox_ratio` |
| 停在顶端后缓慢下沉 | `max_current_A` 不够 | 加到 9~10 A 再试 |

---

## 10. 一张最短路径卡片

```
1. catch_motor_enable = 0
2. 填 catch_debug.config（四个 verified = 1，max_current_A ≥ 7）
3. catch_debug.command = 1      → 期望 state = 1
4. catch_debug.command = 2      → 期望 state = 2
5. catch_motor_enable = 1       → 期望 motor_enabled = 1
6. target_height_m = 0.01, command = 3  → 看方向对不对
7. 方向对了：加负载，走到 0.2，读 final_current_A 与 disturbance_rad_s2
8. 出错：catch_motor_enable = 0 → command = 5 → 改参数 → 回到第 3 步
```
