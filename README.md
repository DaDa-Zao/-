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
其余全是只读，每 4 ms 被覆盖一遍。

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
| `final_current_A` | 真正发出去的电流，A。静载托住 3.3 kg 时应该是 2.7~3.4 A |
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
| `raw_current_A` 正常但 `final_current_A` 明显小 | 被 `max_current_A` 限住了 |
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
| `travel_m` | 0.395 | 量一下实际行程再填 |
| `pulley_radius_m` | 0.025 | 带轮节圆半径 |
| `external_speed_ratio` | 1.0 | **1:1 直连，别填 2** |
| `gearbox_ratio` | 19.2032 | 不要改 |
| `speed_m_s` | 0.020 | 先用 20 mm/s 把流程走通 |
| `acceleration_m_s2` | 0.040 | 三个速度参数必须一起改 |
| `overspeed_m_s` | 0.030 | 取 `speed_m_s` 的 1.4~1.5 倍。贴着设一动就锁超速 |
| `position_tolerance_m` | 0.001 | 1 mm 算到位 |
| `settled_speed_m_s` | 0.001 | 1 mm/s 算停稳 |
| `travel_fault_margin_m` | 0.005 | 软限位余量 |
| `feedback_timeout_us` | 10000 | 多久没新帧判掉线，范围 2000~20000 |
| `settle_ms` | 500 | 稳住 0.5 s 才算到位 |
| `relay_dwell_ms` | 300 | 继电器两次动作之间至少隔多久 |
| `phase_timeout_ms` | 45000 | 单阶段超时，**等人确认也算在里面** |

> 怎么调快、四档参数、每档的验收判据，见 `调参流程.md` §2.5。

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
