# 取块机构（catch_block）

竖直升降 + 夹取。M3508 + C620，行程 395 mm，负载 3.3 kg。

**三份文档分工**

| 文件 | 看它做什么 |
|---|---|
| **本文** | 总览：换算、参数、限制、上手指引 |
| `调参流程.md` | 照着一步步调，每步都有判据 |
| `状态位速查.md` | 调试时查表，卡住了查哪个数 |

---

## 1. 代码在哪

| 文件 | 作用 |
|---|---|
| `catch_block.hpp/.cpp` | 状态机、命令入口、故障策略、`catch_debug` 调试窗口 |
| `catch_block_driver.hpp/.cpp` | 电机 + 两路继电器 + 零点 + 米↔弧度换算 |
| `catch_block_config.hpp/.cpp` | 24 个参数与两段校验 |

入口只有一处：`MainFrame.cpp` 里的 `System.RegistApp(CatchBlock::GetInstance());`

**分层规矩**：`Extensions70`（电机库、ADRC）和 `FrameComponets`（框架）**只读，一个字都不能改**，
所有应用逻辑只在 `Users/` 里。库管算法和收发，驱动层管单位/采样/健康判据，状态机管流程。

### 1.1 各任务跑多快（`RtosCpp.cpp`）

| 任务 | 入口 | 写法 | 频率 | 跟本项目的关系 |
|---|---|---|---|---|
| Control | `ControlCpp()` | `osDelay(1)` | **1 kHz** | **唯一出力点**：ADRC 算电流、CAN 发帧 |
| App | `ApplicationCpp()` | `osDelayUntil(5)` | **200 Hz（5 ms）** | `CatchBlock::Update()`、`Publish()` |
| RobotSys | `RobotSystemCpp()` | `osDelayUntil(5)` | 200 Hz | DWT 维护、`System.Run()` |
| StateCore | `StateCoreCpp()` | `osDelayUntil(4)` | 250 Hz | 本项目没用 |
| SpiRead | `SpiReadCpp()` | `osDelayUntil(2)` | 500 Hz | 本项目没用 |
| SpiConsume | `SpiConsumeCpp()` | 等信号量 | 事件驱动 | 本项目没用 |
| CAN 接收 | `HAL_CAN_RxFifo0MsgPendingCallback` | 中断 | **1 kHz** | C620 主动上报，累加转子角度 |

六个任务**全是 `osPriorityNormal`**，时间片轮转，没有实时优先级差别。

> **`Update()` 确实是 200 Hz，已核对到底**：`ApplicationCpp()` 用 `osDelayUntil(5)` = 200 Hz；
> `System::_Update_Applications()` 里还有一层 `CntFull()` 按 `prescaler` 预分频，
> 但 `prescaler` 默认 1 且**全工程无人修改**，所以每个 App 拍都调 `Update()`。
> `Extensions70/App_70/Verif/MotorDM_Test_Pzh.cpp:8` 的注释也写明"Update() 由框架固定为 200Hz（5ms/次）"。

**这对判据意味着什么**：

- `catch_debug` 所有字段**每 5 ms 才刷新一次**——Watch 里看到的最快变化就是 5 ms 一档
- `now_us` 也是每 5 ms 更新一次（`AdvanceClock()` 在采样时走），
  所以 `settle_ms`(500)、`relay_dwell_ms`(300)、`phase_timeout_ms`(45000) 的实际精度都是 **±5 ms**
- 状态机对命令的响应延迟**最多 5 ms**（命令在邮箱里等下一拍）
- `CheckMotion()` 那个 `2~15 ms` 的判据，就是在量 App 任务的实际抖动
  （名义 5 ms，允许 2~15 ms 是因为 `osDelayUntil` 会被同优先级任务挤）

---

## 2. 先记住三个数

```
1 rad（转子）= 0.025 ÷ 19.2032 = 1.3019 mm
1 圈（转子）= 8.18 mm
满行程 395 mm = 48.3 圈（转子）
```

电机齿轮与带轮齿轮 **1:1** 直连，中间只有 19.2032 的减速箱。

| 机构速度 | 转子转速 | 395 mm 走完 |
|---|---|---|
| 20 mm/s（起步） | 147 rpm | 20 s |
| 100 mm/s | 733 rpm | 4 s |
| 250 mm/s（实用上限） | 1834 rpm | 1.6 s |

**静态托住 3.3 kg 要 2.7 A，按 80% 效率算 3.4 A。**
所以 `max_current_A` 必须 > 3.4；默认填 8 是留余量，**别再加**。

---

## 3. 最短上手路径

> 只想先验证「原点 → 最高点」这一段，看 `调参流程.md` 开头那张卡，比这里更短。

```
1. catch_motor_enable = 0
2. 填 catch_debug.config（两个 sign 先填 ±1，max_current_A ≥ 4）
3. catch_debug.command = 1   →  state = 1
4. catch_debug.command = 2   →  state = 2
5. catch_motor_enable = 1    →  motor_enabled = 1
6. target_height_m = 0.01，command = 3  →  看机构往哪边走
7. 方向对了：加负载，走到 0.2，读 final_current_A
8. 出错：catch_motor_enable = 0 → command = 5 → 改参数 → 回第 3 步
```

命令号记这些就够：

```
流程   1 配置   2 标零   5 复位   6 软急停
单步   7 泵开   8 泵关   9 阀吸合(夹紧)   10 阀松开
      11 抬升到顶        12 降回原点
自动   13 夹取  14 上升  15 许可  16 回程
```

---

## 4. 调试窗口

Keil 里 Watch 输入 `catch_debug`，点左边 `+` 展开。

**只有三个能写**：`config`（参数面板）、`command`（命令号）、`target_height_m`。
其余全是只读，每 5 ms 被覆盖一遍。

另外三个独立开关（不在结构体里）：

| 名字 | 作用 |
|---|---|
| `catch_motor_enable` | 非 0 = 允许电机出力 |
| `catch_pump_on` | 非 0 = PE12 高，气泵吸附 |
| `catch_valve_closed` | 非 0 = PE6 高，夹爪闭合 |

**最常看的几个只读量**：

| 字段 | 含义 |
|---|---|
| `state` | 现在停在哪一步，见 `状态位速查.md` §4 |
| `blockers` | 此刻为什么不能出力，见 `状态位速查.md` §5 |
| `faults` / `motor_faults` | 出过什么错，见 `状态位速查.md` §6 |
| `actual_height_m` | 相对原点的高度，m |
| `final_current_A` | 真正发出去的电流，A。静载托住两个物块时实测 **−3.16 A**（`lift_encoder_sign = -1` 下托举为负） |
| `disturbance_rad_s2` | ESO 估的总扰动，rad/s²。同一工况实测 **+2952**。**它和 `final_current_A` 必须反号**，同号说明 `b0` 或接线反了 |
| `feedback_valid` | 1 = 这份反馈可信。变 0 时高度冻结、使能被挡，**帧回来会自己恢复** |
| `feedback_hz` | 反馈帧率，正常 C620 是 1000。掉到几百说明总线丢帧 |
| `motor_enabled` | 1 = 正在输出。**只读**，开关是 `catch_motor_enable` |

**观测量的读法**：按这条链看，一眼定位是哪一段出问题。

```
目标 ──► TD(v1,v2) ──► 位置环 ──► ADRC 输出 ──► 限幅 ──► 斜坡 ──► final_current_A
              ▲                                      │
              └──────── ESO(z1,z2,z3) ◄── 反馈 ──────┘
```

| 现象 | 说明 |
|---|---|
| `reference_velocity_rad_s` 是方波 | `acceleration_m_s2` 太大，TD 没起平滑作用 |
| `eso_velocity_rad_s` 长期滞后参考 | `observer_Hz` 偏低 |
| `disturbance_rad_s2` 持续饱和（±几万） | `inertia_kg_m2` 或 `torque_constant_Nm_A` 错 |
| `raw_current_A` 贴着 ±`max_current_A`（±8 A） | ADRC 输出饱和：托不住，或位置环增益不够 |
| `raw_current_A` 明显大于 `final_current_A` | 还在爬坡中（库内 1098.63 A/s），瞬态正常；稳态两者相等 |
| `raw_current_A` 与 `limited_current_A` 永远相等 | **正常**。库在 `CalcPos()` 末尾就把 `i_des` 限幅了，本层再 clamp 一次是空操作 |
| `measured_position_rad` 是负数 | **正常**。那是转子绝对累计角，可正可负，有意义的只有 `actual_height_m` |
| `feedback_hz` 掉到 500 以下 | 总线丢帧。它同时是观测器的采样率，先修线再调参 |

---

## 5. 参数表

结构体默认值已经能跑，**第一次烧写基本不用改**，只有两个符号位要现场测。

| 字段 | 默认 | 说明 |
|---|---|---|
| `esc_id` | 1 | C620 编号 1~8，反馈 ID = 0x200 + 编号 |
| `lift_encoder_sign` | −1 | 上升时编码器往哪边变，**必须实测** |
| `current_encoder_sign` | 1 | **只能填 +1**。方向反了要调换电机相线，软件改不了 |
| `inertia_kg_m2` | 1.5e-5 | 折算到转子轴的整机惯量 |
| `torque_constant_Nm_A` | 0.015625 | **转子域**。手册的 0.3 是输出轴值，要除以 19.2 |
| `viscous_Nm_s_rad` | 0 | 留给 ESO 自己估 |
| `max_current_A` | 8 | 必须 > 3.4 才托得住，上限 20 |
| `current_slew_A_s` | 2000 | **当前不生效**，库把爬坡率写死成 1098.63 A/s |
| `observer_Hz` | 50 | ESO 带宽 |
| `controller_Hz` | 15 | 位置环带宽，必须 < `observer_Hz` |
| `travel_m` | 0.435 | 量出来的实际行程再减 5 mm。现在离机械极限（约 455 mm）只剩 20 mm，别再加 |
| `pulley_radius_m` | 0.025 | 带轮节圆半径 |
| `external_speed_ratio` | 1.0 | **1:1 直连，别填 2** |
| `gearbox_ratio` | 19.2032 | 不要改 |
| `speed_m_s` | 0.35 | 台架已验：395 mm 行程约 1.4 s |
| `acceleration_m_s2` | 1.00 | 三个速度参数必须一起改 |
| `overspeed_m_s` | 0.52 | 取 `speed_m_s` 的 1.4~1.5 倍。贴着设一动就锁超速 |
| `position_tolerance_m` | 0.001 | 1 mm 算到位 |
| `settled_speed_m_s` | 0.001 | 1 mm/s 算停稳 |
| `travel_fault_margin_m` | 0.005 | 软限位余量 |
| `feedback_timeout_us` | 7000 | 多久没新帧判掉线。**别超过"最高速转半圈"的时间**（`4090 / overspeed_m_s` µs）：0.52 m/s 时只有 7.87 ms，所以填 7000 |
| `settle_ms` | 500 | 稳住 0.5 s 才算到位 |
| `relay_dwell_ms` | 300 | 继电器两次动作之间至少隔多久 |
| `phase_timeout_ms` | 45000 | 单阶段超时，**等人确认也算在里面** |

> ⚠ **`travel_m = 0.435` 是补偿值，不是真实行程。**
> 实测填 0.435 时机构正好走到 395 mm，说明 `metres_per_rotor_rad_` 偏大约 10%
> （等效带轮有效半径 ≈ 22.7 mm，而 `pulley_radius_m` 填的是 25 mm）。
> **不要按"真实行程 395 mm"把它改回 0.395** —— 那样只能走到 359 mm。
>
> 连带影响（都是这一个比例误差带来的）：
> - `actual_height_m` 全程读数**偏大 10.1%**：显示 0.100 时实际在 0.0908
> - `speed_m_s` / `overspeed_m_s` 同理偏大 → **超速门限实际偏松 10%**（0.52 对应真实 0.472 m/s）
> - `travel_fault_margin_m` 的 5 mm 实际只有 **4.54 mm**，末端容差比看起来更紧
> - `position_tolerance_m` 的 1 mm 实际是 0.91 mm
>
> 想彻底修对是改 `pulley_radius_m = 0.0227`（然后 `travel_m` 填 0.395），
> 一次把整条米制换算拉正。但那是另一轮改动，会牵动零点和所有阈值，**目前按现状使用、别动**。

> 怎么调快、四档参数、每档的验收判据，见 `调参流程.md` §2.5。

### 5.1 两版实测对照：提速只改 4 个参数

其余 20 个参数两版完全一致，**别一起动**。

| 参数 | B 档（395 mm 行程 1.7~2.0 s） | C 档（当前，395 mm 行程 1.3~1.5 s） | 作用 |
|---|---|---|---|
| `speed_m_s` | 0.25 | **0.35** | TD 的速度上限，**提速的主要贡献** |
| `acceleration_m_s2` | 0.60 | **1.00** | TD 的加速度上限，配套改 |
| `overspeed_m_s` | 0.38 | **0.52** | 只用来算超速门限，取 `speed_m_s` 的 1.5 倍 |
| `feedback_timeout_us` | 20000 | **7000** | 必须随速度降，否则断线后盲开距离过大 |

> 表里两个时间都是 **395 mm 行程**下测的。行程改成现在的 `travel_m = 0.435` 之后两档都会变长：
> B 档约 2.16 s、C 档约 1.59 s。**报速度时要说清行程**，否则数字对不上。

**为什么 `feedback_timeout_us` 必须跟着降**：它不能超过"最高速下转子转半圈"的时间
（`4090 / overspeed_m_s` µs）。B 档 10.76 ms、C 档 7.87 ms。超了这条线，
断线后控制器还拿旧位置算，机构走多远才报出来就失控了。

**回 B 档做对比测试**：只改回前三个，`feedback_timeout_us` 留 7000 也没问题
（B 档半圈 10.76 ms > 7 ms，仍然安全）。

---

## 6. 必须知道的六条限制

1. **没有重力前馈。** 使能那一瞬间控制器还不知道有重量，机构会先往下掉。
   位移只有 0.13 mm，肉眼看不出，但**会触发超速锁存**（`motor_faults = 64`），
   表现是"一使能就掉下来，然后就动不了了"。
   → 使能时必须有支承；首试把 `overspeed_m_s` 放宽到 **0.06~0.10**。
2. **没有机械防坠。** 所有"保持"都靠驱动器持续出电流，断电、故障撤力、过热都会掉负载。
3. **没有夹取传感器。** 夹没夹牢、物块走没走，全靠人眼看，对应 `command = 14` / `16`。
4. **参数只在 `command = 1` 那一刻生效。** 改完不重配等于没改。
5. **`esc_id` 改不了。** 换编号要先 `command = 5`，再**重新上电**；否则 `Configure()` 直接拒绝。
6. **使能期间不要停核。** 下断点、单步、开内存窗口都会让 App 周期超时并撤力。

---

## 7. 常见问题

| 现象 | 处置 |
|---|---|
| `command = 1` 返回 2 | 看 `reason`：1 阈值、2 硬件参数、6 还在使能 |
| `command = 1` 返回 3、state 变 14 | `esc_id` 错或 CAN 没通 → 查线，`command = 5` 再试 |
| `command = 2` 返回 2、`reason = 5` | 反馈没通。看 `blockers` 是不是 64 |
| 一使能就 13 / 14 | 悬空使能导致超速锁存 → `command = 5`，改成坐底使能 |
| 高度不动但电流很大 | 没使能 / 机械卡死 / 电流方向反 |
| 停在顶端缓慢下沉 | `max_current_A` 不够，加到 4~5 A |
| 走到一半报 Travel | 零点丢了，重新 `command = 2` |
| `7~10` 返回 1 但继电器不响 | 硬件侧：查触发电压、NO/NC 接法、`pump_on` / `valve_closed` 回读 |

> 更细的分诊见 `调参流程.md` §3，每个状态位的意思见 `状态位速查.md`。
