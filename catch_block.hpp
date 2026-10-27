#pragma once
#include "System.hpp"
#include "catch_block_driver.hpp"

/**
 * @file catch_block.hpp
 * @brief 取块机构的业务层：状态机 + 人工确认 + 故障策略 + 到位判据。
 *
 * 分工：这一层只决定"什么时候做、做不到怎么办"，不碰寄存器也不认识 MotorDJI；
 *       所有硬件动作都调 CatchBlockDriver，对外单位一律是米 / 米每秒 / 毫秒。
 * 上下文：Update() 在 App 任务里跑（5 ms 一拍，200 Hz，见 RtosCpp.cpp），位置环在 Control 任务里跑（1 kHz）。
 * 安全边界：机构没有失电保持，也没有夹取/位置传感器。故障动作是"撤驱动"而不是
 *           "安全停止"，必须靠外部支承和防坠结构兜底。
 */

/**
 * @brief Keil Watch 里唯一要展开看的全局变量
 * @note command 可写，写完由下一拍自动清零。命令号分三组：
 *
 *   【走状态机】 1=配置  2=当前位置标零  3=走到 target_height_m  4=原地保持
 *               5=清故障回未配置  6=软急停
 *   【单步动作】 7/8=气泵开/关(PE12)   9/10=电磁阀闭合/松开(PE6)
 *               11=抬升到行程顶端   12=降回原点
 *   【自动流程】 13=开始夹取  14=确认夹牢并上升  15=许可投放  16=确认已取走并回程
 *
 * 7~10 是硬件直出，不看状态也不看配置，只用来单独验证气路和继电器。
 * 11~12 是运动，必须"配置 → 标零 → 使能"都做完，否则返回 2；到位看 state 变成 ManualHold。
 *
 * result 回读：0=没执行 1=已接受（7~12 表示已执行）2=被拒绝（看 reason）3=执行后处于故障。
 * 其余字段全部只读，由 Publish() 每拍刷新。
 */
class CatchBlockDebug
{
public:
  // 这里只有三类东西，加新字段前先想清楚属于哪一类——不要出现第四类：
  //   ① 输入：只有 command / target_height_m / config 能写，其余全部每 5 ms 被 Publish() 覆盖
  //   ② 事实：硬件和状态机"现在是什么样"，一个事实只写一处
  //   ③ 诊断：把"为什么不"翻译成能直接看的数，全部由 ② 推导，不存第二份独立事实
  CatchBlockConfig config;              ///< @brief ① 命令 1 使用的参数面板，可在 Watch 里逐项填
  volatile uint8_t command = 0;         ///< @brief ① 调试命令号 0~16，写后自动清 0
  volatile float target_height_m = 0;   ///< @brief ① 命令 3 要走到的高度 (m)

  // ② 事实
  volatile uint8_t state = 0;           ///< @brief State 枚举值，顺序即数值
  volatile uint32_t faults = 0;         ///< @brief 应用故障 | 驱动故障（两个位布局按位或），锁存，command 1/5 清。只看驱动层用下面的 motor_faults
  volatile uint32_t motor_faults = 0;   ///< @brief 驱动层锁定故障原码（CatchBlockDriver::DriverFault）：1 配置 2 反馈非法 8 数值 64 超速，其余是保留位
  volatile uint8_t motor_enabled = 0;   ///< @brief 1 = 驱动正在输出。只读，开关是 catch_motor_enable
  volatile uint8_t feedback_valid = 0;  ///< @brief 1 = 反馈过了库的初始化窗口，且没超过 feedback_timeout_us
  volatile uint16_t feedback_age_ms = 0;      ///< @brief 距上一帧多少 ms；超过 config.feedback_timeout_us/1000 就判 Stale
  volatile uint16_t feedback_age_peak_ms = 0; ///< @brief 自 command = 1 以来最长帧间隔，ms。用它判"到底断没断过帧"
  volatile uint16_t feedback_hz = 0;          ///< @brief 库统计的反馈帧率，Hz。正常 1000；掉到几百说明丢帧或被别的报文挤占
  volatile uint8_t pump_on = 0;         ///< @brief 1 = PE12 实际为高
  volatile uint8_t valve_closed = 0;    ///< @brief 1 = PE6 实际为高
  // 面板按键的两级证据。按键"没反应"时先看这两个，能分开是接线/引脚问题还是被状态机拒了。
  volatile uint16_t keys_raw = 0xFFFF;  ///< @brief 9 个键**消抖后**的电平位图，bit i = 1 表示第 i 个键松开。按下时对应位变 0
  volatile uint8_t keys_last = 0xFF;    ///< @brief 最近一次**真正触发了动作**的键索引 0~8，0xFF = 从没触发过
  volatile float actual_height_m = 0;   ///< @brief 当前实际高度 (m)

  // ③ 诊断
  volatile uint8_t result = 0;          ///< @brief 上一条命令的结果：0 没执行 1 已接受 2 被拒 3 出故障
  volatile uint8_t reason = 0;          ///< @brief result=2 的原因：1 阈值 2 硬件 3 状态 4 目标 5 未就绪 6 还在使能 7 还在动
  volatile uint16_t blockers = 0;       ///< @brief 现在为什么不能出力，0 = 可以。位定义见 CatchBlock::Blocker

  // 调参用的控制器观测量，与 CatchBlockDriver 的公有诊断量一一对应。
  volatile float target_position_rad = 0;      ///< @brief 位置环目标 (rad)
  volatile float measured_position_rad = 0;    ///< @brief 位置环反馈 (rad)
  volatile float reference_position_rad = 0;   ///< @brief TD 参考位置 v1 (rad)
  volatile float reference_velocity_rad_s = 0; ///< @brief TD 参考速度 v2 (rad/s)
  volatile float eso_position_rad = 0;         ///< @brief ESO z1 (rad)
  volatile float eso_velocity_rad_s = 0;       ///< @brief ESO z2 (rad/s)
  volatile float disturbance_rad_s2 = 0;       ///< @brief ESO z3 (rad/s²)
  volatile float raw_current_A = 0;            ///< @brief ADRC 输出 (A)。库内已限幅，与下一项相同；撤力时为 0
  volatile float limited_current_A = 0;        ///< @brief 限幅后 (A)。与上一项相同，撤力时为 0
  volatile float final_current_A = 0;          ///< @brief 真正下发的电流 (A)
  volatile float accepted_command_A = 0;       ///< @brief CAN 邮箱已接受，不是实测
  volatile int16_t feedback_current_raw = 0;   ///< @brief 反馈原始电流码，未标定
  volatile uint8_t feedback_current_A_valid = 0; ///< @brief 1 = raw 可当安培用
  volatile uint8_t temperature_C = 0;          ///< @brief 电调温度 (°C)
};

/// @brief 全局唯一的调试窗口实例，Keil Watch 里展开即可读到上面所有字段
extern CatchBlockDebug catch_debug;

// 仅供有外部防坠保护的场合调试；任何电控指令都不能保证失电防坠。
class CatchBlock : public Application
{
  // 单例，由框架注册后按应用周期调用。
  SINGLETON(CatchBlock) : Application("CatchBlock")
  {
  }
  APPLICATION_OVERRIDE
public:
  using Config = CatchBlockConfig; ///< @brief 参数面板类型，与调试窗口共用同一个类

  // 流程状态。除 Unconfigured / Idle / FaultHold / Fault 外，其余状态都允许电机输出。
  enum class State : uint8_t
  {
    Unconfigured, ///< 未配置：只有它能接受 Configure
    Idle,         ///< 已配置、未标零：等待 ArmAtOrigin
    OriginHold,   ///< 原点保持：可关夹爪，可点动
    GripClose,    ///< 夹爪闭合中，等 relay_dwell_ms
    SuctionOn,    ///< 气泵建立中，等 dwell 且两路标志都为高
    GripVerify,   ///< 等操作员确认夹取成功
    Lifting,      ///< 位置环上升中
    TargetHold,   ///< 顶端保持：等操作员许可投放
    ReleaseReady, ///< 许可已给：再校验一拍后进入 Release
    Release,      ///< 已许可投放：等确认物块离开
    Descending,   ///< 受控下降回原点
    ManualMoving, ///< 调试位移中（命令 3 / 11 / 12）
    ManualHold,   ///< 调试保持中（命令 4）
    FaultHold,    ///< 故障但仍在保持位置，仅原先已使能时有效
    Fault         ///< 故障且已断驱动，必须外部支承
  };

  // 故障位，可叠加；锁存后不自动恢复。
  enum Fault : uint32_t
  {
    NoFault = 0,
    Configuration = 1, ///< 参数非法或驱动初始化失败
    Feedback = 2,      ///< 反馈无效、超时或跳变
    Controller = 4,    ///< 控制器判故障（数值/状态不一致）
    Timing = 8,        ///< 应用周期超出 2~15 ms
    Timeout = 16,      ///< 阶段等待超时
    Travel = 32,       ///< 高度或目标越界
    CanTransmit = 64,  ///< 上次 CAN 发送结果非 Ok
    OperatorHold = 128 ///< 操作员请求的 SoftStop
  };

  /** @name 命令入口（只入队，返回 false 说明邮箱被上一条占着，不代表动作失败） */
  // 真正执行在下一拍 Update()：ConsumeRequest() 取出来交给 Process()。
  // 动作成没成，看 catch_debug.result / reason / faults。
  /** @{ */
  /**
   * @brief 下发参数，并初始化驱动与位置环。
   * @note 要求电机已停、机构被可靠支承。配好之后仍未使能，得再调 ArmAtOrigin()。
   */
  bool Configure(const Config &config);
  /**
   * @brief 把当前静止位置记成 0 m，并按行程划出软限位。
   * @note 调它等于声明"机构现在确实被托住了"，否则零点就是个悬空位置。
   */
  bool ArmAtOrigin();
  /**
   * @brief 闭合夹爪，开始取块流程。
   * @note 要求原点已稳定 settle_ms；没稳住会被拒，并计入 rejected_ 计数。
   */
  bool CloseGrip();
  /**
   * @brief 人确认夹牢了，上升到行程顶端。
   * @note 调它即视为"我已确认夹紧/吸牢"；另要求两路继电器此刻都为高。
   */
  bool ConfirmGripAndLift();
  /**
   * @brief 人许可投放。
   * @note 调它即视为"投放区已隔离"。要求顶端已稳定，且下一拍复核通过才真进 Release。
   */
  bool PermitDrop();
  /**
   * @brief 人确认物块已离开，受控降回原点。
   * @note 不靠延时推定，必须由人确认。
   */
  bool ConfirmObjectClearAndReturn();
  /**
   * @brief 软急停：冻结当前动作，锁在当前位置。
   * @note 不是断电急停，仍然靠驱动器托着负载；没有外部支承时别指望它防坠。
   */
  bool SoftStop();
  /**
   * @brief 清故障、回到未配置。
   * @note 要求机构有支承。这是唯一能清掉驱动层锁存（超速）的路径；
   *       反馈过期不需要它，帧回来自动就消。
   */
  bool ResetWhileSupported();
  /**
   * @brief 调试：走到相对原点 height_m 米处并停住。
   * @note 只能从 OriginHold / ManualHold 出发，目标必须落在 0..travel_m。
   */
  bool MoveToHeight(float height_m);
  /**
   * @brief 调试：抬升到行程顶端。
   * @note 等价于 MoveToHeight(travel_m)，但目标在 Process 里才取，不会用到过期参数。
   */
  bool LiftToTop();
  /**
   * @brief 调试：受控降回原点（等价于 MoveToHeight(0)）。
   */
  bool DescendToOrigin();
  /**
   * @brief 调试：以当前反馈位置为目标停住。
   * @note 用来中断一次 MoveToHeight，但不清故障。
   */
  bool HoldHere();
  /** @} */

  /** @name 单步动作：直接操作两路继电器，供 Keil 逐个验证 */
  /** @{ */
  /**
   * @brief 强制气泵通 / 断。
   * @note 只是改 catch_pump_on，下一拍由驱动层写到 PE12。不看状态也不看配置；
   *       流程进行中调用会破坏 SuctionOn / GripVerify 的判据，只用于单步调试。
   */
  void SetPump(bool on);
  /**
   * @brief 强制电磁阀吸合 / 松开（true = PE6 高 = 夹爪闭合）。
   */
  void SetValve(bool on);
  /** @} */
private:
  // 内部命令码。对外只暴露上面那组方法，命令码不出本类。
  enum class Command : uint8_t
  {
    None,
    Configure,
    ArmAtOrigin,
    CloseGrip,
    ConfirmGripAndLift,
    PermitDrop,
    ConfirmObjectClearAndReturn,
    SoftStop,
    ResetWhileSupported,
    MoveToHeight,
    LiftToTop,
    DescendToOrigin,
    HoldHere
  };

  // 命令被拒绝的原因。只用于调试窗口，不参与任何控制逻辑。
  enum RejectReason : uint8_t
  {
    ReasonNone = 0,  ///< 没被拒绝
    ReasonThreshold, ///< 业务阈值不自洽（Validate 失败）
    ReasonHardware,  ///< 硬件参数非法（编号/符号/惯量/带宽/限流/接线）
    ReasonState,     ///< 当前状态不接受这条命令
    ReasonTarget,    ///< 目标高度是 NaN/Inf 或超出 0..travel_m
    ReasonNotReady,  ///< 未使能、反馈不健康；具体哪条看 catch_debug.blockers
    ReasonEnabled,   ///< 电机仍在输出，此命令要求先撤力（仅配置）
    ReasonMoving     ///< 机构还在动，速度高于 settled_speed_m_s（标零专用）
  };

  // "现在为什么不能出力"。只用于调试窗口，不参与控制逻辑。
  // 按因果顺序取第一条不满足的，所以它永远只有一个答案（健康位可能几位叠加）：
  //   开关 1 → 配置 2 → 硬件 16~2048 → 标零 4 → 状态 8
  // 健康位放在标零前面：标零本身依赖反馈，CAN 没通时该报 64，不该报 4。
  enum Blocker : uint16_t
  {
    BlockerNone = 0,             ///< 可以出力，或者已经在出力
    BlockerMotorSwitch = 1,      ///< catch_motor_enable 还是 0
    BlockerNotConfigured = 2,    ///< 还没配置成功（command = 1）
    BlockerNoOrigin = 4,         ///< 还没标零（command = 2）
    BlockerState = 8,            ///< 当前状态不在出力白名单里
    BlockerCtrlNotReady = 16,    ///< 控制器没配好（config 没成，或 ADRC 没真正 Init）
    BlockerFaultLatched = 32,    ///< 有锁存的软件故障，看 motor_faults（只有超速和配置失败会真的置位）
    BlockerNoFeedback = 64,      ///< 一帧反馈都没收到：CAN 没通、编号不对、电调没上电
    BlockerFeedbackInvalid = 128,///< 收到了帧，但还在库的 offset 初始化窗口里，累计角还没开始算
    BlockerFeedbackFault = 256,  ///< 保留位，恒 0：回退后的库不再产生帧级故障码
    BlockerFeedbackStale = 512,  ///< 最近 feedback_timeout_us 内没有新帧（**不锁存，帧回来自动消**）
    BlockerAdrcInvalid = 1024,   ///< ADRC 内部参数无效（Init 没跑起来）
    BlockerTransmitFailed = 2048 ///< 保留位，恒 0：BSP::CAN::Transmit 返回 void，发送结果不可观测
  };

  CatchBlockDriver driver_; ///< @brief 机构驱动层：电机 + 两路继电器 + 零点 + 换算
  Config config_;           ///< @brief 生效参数（策略阈值用），只在 Configure 成功时赋值

  Command pending_command_ = Command::None; ///< @brief 邮箱里的命令码
  float pending_height_m_ = 0;              ///< @brief 邮箱里的目标高度
  Config pending_config_;                   ///< @brief 邮箱里的参数副本

  bool started_ = false;        ///< @brief 已启动
  bool protection_ack_ = false; ///< @brief 已确认机构有支承（标零时置 1，配置/复位时清 0）
  State state_ = State::Unconfigured;

  uint32_t faults_ = 0;       ///< @brief 应用故障位
  uint32_t rejected_ = 0;     ///< @brief 被拒绝的命令数
  uint8_t last_reject_ = ReasonNone; ///< @brief 最近一次拒绝的原因，每条命令进来时先清零
  uint16_t blockers_ = BlockerNone;  ///< @brief 本拍为什么不能出力，ApplyOutputs() 里重算
  uint32_t phase_start_ms_ = 0; ///< @brief 当前阶段起点

  uint64_t previous_tick_us_ = 0; ///< @brief 上一拍时刻
  uint64_t settled_since_us_ = 0; ///< @brief 到位起始时刻
  float manual_target_m_ = 0;     ///< @brief 调试位移的目标高度

  // 三个重载共用一个邮箱：命令码 + 可选的目标高度 / 参数副本，由上面的公开方法调用。
  // 命令最后写，保证载荷先就绪；邮箱非空就返回 false，不排队也不覆盖。
  bool Enqueue(Command command);
  bool Enqueue(Command command, float height_m);
  bool Enqueue(Command command, const Config &config);

  /// @brief 记下这次为什么被拒（只写 last_reject_，计数统一在 Process() 末尾做）
  void Reject(uint8_t reason);

  bool StateAllowsEnable() const;                ///< @brief 这个状态允许电机出力吗（10 个状态的白名单）
  bool Settled(float target_m, uint64_t now_us); ///< @brief 到目标了并且稳住够久？
  bool ApplyHeight(float metres, uint32_t now_ms); ///< @brief 下高度目标；失败报 Travel 故障

  bool ApplyConfigure(const Config &config, uint32_t now_ms); ///< @brief 配置：查四类参数，然后初始化驱动
  bool DoArmAtOrigin(uint32_t now_ms);                        ///< @brief 标零：采零点、设软限位

  void Process(Command command, const Config &config, float height_m, uint32_t now_ms); ///< @brief 主分发：拿到命令后决定转状态还是拒绝
  void Transition(State state, uint32_t now_ms);  ///< @brief 换状态，并把阶段计时清零
  void Fail(uint32_t fault, bool can_hold, uint32_t now_ms); ///< @brief 记故障；can_hold 决定托住还是撤驱动
  void CheckMotion(uint64_t now_us, uint32_t now_ms, uint64_t elapsed_us); ///< @brief 使能期间每拍体检：周期、CAN、控制器、越界
  void ConsumeRequest(uint32_t now_ms);      ///< @brief 从邮箱取业务代码下的命令
  void ConsumeDebugCommand(uint32_t now_ms); ///< @brief 从邮箱取 Keil 窗口写的命令
  void Advance(uint64_t now_us, uint32_t now_ms); ///< @brief 自动流程往前走一格，再查阶段超时
  void ApplyOutputs();                       ///< @brief 写继电器，并按当前状态决定使能还是撤力
  void Publish();                            ///< @brief 把内部量刷到 catch_debug
};
