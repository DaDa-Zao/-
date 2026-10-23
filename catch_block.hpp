#pragma once
#include "System.hpp"
#include "catch_block_driver.hpp"

/**
 * @file catch_block.hpp
 * @brief 取块机构（夹取 → 提升 → 顶端保持 → 主动投放 → 受控回程）的业务层。
 *
 * 本层只决定"什么时候做、做错了怎么办"，不认识寄存器、不认识 MotorDJI：
 * 状态机 · 人工确认 · 故障策略 · 到位判据 · 阶段超时 · 命令邮箱 · 调试窗口。
 * 一切硬件动作都经 CatchBlockDriver 完成，单位一律是米 / 米每秒 / 毫秒。
 *
 * 坐标与单位：驱动层内部用"转子累计角度 rad"，本层对外用米，
 * 换算系数约 2.6037 mm/rad（0.025 m 带轮 × 2 ÷ 19.2032 减速比）。
 * 运行上下文：Update() 在 App 任务（名义 5 ms）跑状态机与继电器；
 * 位置环在 Control 任务（1 kHz）由 MotorDJI::Control() 独立执行。
 * 安全边界：没有失电保持、没有夹取/位置传感器；故障动作是"撤驱动"而非"安全停止"，
 * 必须依赖外部支承与防坠。
 */

/**
 * @brief Keil Watch 的调试窗口
 * @note command 可写：1=配置（用下面的 config）2=当前位置标零 3=移动到 target_height_m
 *       4=保持当前位置 5=清故障并回到未配置 6=软急停（保持式停车）；写后由下一拍消费并自动清零。
 *       result 回读：0=未执行 1=已接受 2=被拒绝 3=执行后处于故障。
 *       其余字段只读，由 Publish() 每拍刷新。
 */
class CatchBlockDebug
{
public:
  CatchBlockConfig config;              ///< @brief 命令 1 使用的参数面板，可直接在 Watch 里逐项填
  volatile uint8_t command = 0;         ///< @brief 调试命令号，写后自动清 0
  volatile uint8_t result = 0;          ///< @brief 上一条命令的执行结果
  volatile float target_height_m = 0;   ///< @brief 命令 3 的目标高度 (m)
  volatile float actual_height_m = 0;   ///< @brief 实际高度回读 (m)
  volatile uint32_t faults = 0;         ///< @brief 应用故障 | 驱动故障
  volatile uint8_t state = 0;           ///< @brief State 枚举值
  volatile uint8_t feedback_valid = 0;  ///< @brief 1 = 反馈有效
  volatile uint8_t motor_enabled = 0;   ///< @brief 1 = 驱动正在输出
  volatile uint8_t pump_on = 0;         ///< @brief 1 = PE6 实际为高
  volatile uint8_t valve_closed = 0;    ///< @brief 1 = PE12 实际为高

  // 调参用的控制器观测量，与 CatchBlockDriver 的公有诊断量一一对应。
  volatile float target_position_rad = 0;      ///< @brief 位置环目标 (rad)
  volatile float measured_position_rad = 0;    ///< @brief 位置环反馈 (rad)
  volatile float reference_position_rad = 0;   ///< @brief TD 参考位置 v1 (rad)
  volatile float reference_velocity_rad_s = 0; ///< @brief TD 参考速度 v2 (rad/s)
  volatile float eso_position_rad = 0;         ///< @brief ESO z1 (rad)
  volatile float eso_velocity_rad_s = 0;       ///< @brief ESO z2 (rad/s)
  volatile float disturbance_rad_s2 = 0;       ///< @brief ESO z3 (rad/s²)
  volatile float raw_current_A = 0;            ///< @brief ADRC 原始输出 (A)
  volatile float limited_current_A = 0;        ///< @brief 限幅后 (A)
  volatile float final_current_A = 0;          ///< @brief 真正下发的电流 (A)
  volatile float accepted_command_A = 0;       ///< @brief CAN 邮箱已接受，不是实测
  volatile int16_t feedback_current_raw = 0;   ///< @brief 反馈原始电流码，未标定
  volatile uint8_t feedback_current_A_valid = 0; ///< @brief 1 = raw 可当安培用
  volatile uint8_t temperature_C = 0;          ///< @brief 电调温度 (°C)
};

/// @brief 全局唯一的调试窗口实例，Keil Watch 里展开即可读到上面所有字段
extern CatchBlockDebug catch_debug;

// 仅用于有外部防坠保护的调试；任何电控指令都不能保证失电防坠。
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
    OriginHold,   ///< 原点保持：可关夹爪、可点动
    GripClose,    ///< 夹爪闭合动作中，等 relay_dwell_ms
    SuctionOn,    ///< 气泵建立中，等 dwell 且两路标志为高
    GripVerify,   ///< 等待操作员确认夹取成功
    Lifting,      ///< 位置环上升中
    TargetHold,   ///< 顶端保持：等操作员许可投放
    ReleaseReady, ///< 许可已给：再校验一拍后进入 Release
    Release,      ///< 已允许投放：等确认物块离开
    Descending,   ///< 受控下降回原点
    ManualMoving, ///< 调试位移中（命令 3）
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

  /** @name 命令入口：只负责入队，返回 false 表示邮箱已被占用，不代表动作失败 */
  /** @{ */
  /**
   * @brief 下发参数并初始化驱动与位置环
   * @note 前置条件：电机已停、机构被可靠支承。成功后仍不使能，必须再 ArmAtOrigin()。
   */
  bool Configure(const Config &config);
  /**
   * @brief 以当前静止位置为零点并设软限位
   * @note 调用即表示"机构当前确实被可靠支承"，否则零点可能是悬空位置。
   */
  bool ArmAtOrigin();
  /**
   * @brief 原点稳定后进入夹取流程
   * @note 需要原点已稳定 settle_ms，未稳定时命令被拒绝并计入 rejected 计数。
   */
  bool CloseGrip();
  /**
   * @brief 人工确认夹牢后上升到行程顶端
   * @note 调用即表示已确认夹紧/吸牢；还要求两路继电器标志此时都为高。
   */
  bool ConfirmGripAndLift();
  /**
   * @brief 人工许可投放
   * @note 调用即表示投放区已隔离；要求顶端已稳定，且下一拍复核通过才真正进入 Release。
   */
  bool PermitDrop();
  /**
   * @brief 人工确认物块已离开后受控回原点
   * @note 调用即表示物块确已离开，不靠延时推定。
   */
  bool ConfirmObjectClearAndReturn();
  /**
   * @brief 软急停：冻结动作并锁在当前位置
   * @note 不是断电急停，仍然靠驱动器托住负载；没有外部支承时不要依赖它。
   */
  bool SoftStop();
  /**
   * @brief 有支承时清故障、清反馈参考并回到未配置
   * @note 调用即表示机构被可靠支承；这是唯一能清掉反馈跳变锁存的路径。
   */
  bool ResetWhileSupported();
  /**
   * @brief 调试：移动到相对原点的高度
   * @note 只允许从原点保持或手动保持出发，目标须落在 0..travel_m。
   */
  bool MoveToHeight(float height_m);
  /**
   * @brief 调试：以当前反馈位置为目标停住
   * @note 用于中断一次 MoveToHeight 而不清故障。
   */
  bool HoldHere();
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
    HoldHere
  };

  CatchBlockDriver driver_; ///< @brief 机构驱动层：电机 + 两路继电器 + 零点 + 换算
  Config config_;           ///< @brief 生效参数（策略阈值用），只在 Configure 成功时赋值

  Command pending_command_ = Command::None; ///< @brief 邮箱里的命令码
  float pending_height_m_ = 0;              ///< @brief 邮箱里的目标高度
  Config pending_config_;                   ///< @brief 邮箱里的参数副本

  bool started_ = false;        ///< @brief 已启动
  bool protection_ack_ = false; ///< @brief 本次已确认外部保护
  State state_ = State::Unconfigured;

  uint32_t faults_ = 0;       ///< @brief 应用故障位
  uint32_t rejected_ = 0;     ///< @brief 被拒绝的命令数
  uint32_t phase_start_ms_ = 0; ///< @brief 当前阶段起点

  uint64_t previous_tick_us_ = 0; ///< @brief 上一拍时刻
  uint64_t settled_since_us_ = 0; ///< @brief 到位起始时刻
  float manual_target_m_ = 0;     ///< @brief 调试位移的目标高度

  bool Enqueue(Command command);
  bool Enqueue(Command command, float height_m);
  bool Enqueue(Command command, const Config &config);

  bool StateAllowsEnable() const; ///< @brief 使能白名单（10 个状态）
  bool Settled(float target_m, uint64_t now_us); ///< @brief 到位且稳定判定
  bool ApplyHeight(float metres, uint32_t now_ms); ///< @brief 高度目标 + 失败报 Travel

  bool ApplyConfigure(const Config &config, uint32_t now_ms); ///< @brief 初始化驱动与控制器
  bool DoArmAtOrigin(uint32_t now_ms);                        ///< @brief 采零点、设软限位

  void Process(Command command, const Config &config, float height_m, uint32_t now_ms); ///< @brief 命令分发
  void Transition(State state, uint32_t now_ms);  ///< @brief 切状态并复位本阶段计时
  void Fail(uint32_t fault, bool can_hold, uint32_t now_ms); ///< @brief 锁存故障并选择保持或断驱动
  void CheckMotion(uint64_t now_us, uint32_t now_ms, uint64_t elapsed_us); ///< @brief 运行中检查
  void ConsumeRequest(uint32_t now_ms);      ///< @brief 取出邮箱命令
  void ConsumeDebugCommand(uint32_t now_ms); ///< @brief 取出 Keil 命令
  void Advance(uint64_t now_us, uint32_t now_ms); ///< @brief 自动推进与超时
  void ApplyOutputs();                       ///< @brief 写继电器、切换使能
  void Publish();                            ///< @brief 刷新调试窗口
};
