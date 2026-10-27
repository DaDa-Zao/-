#pragma once
#include "catch_block_config.hpp"
#include "motor_dji.hpp"
#include "bsp_gpio.hpp"

/**
 * @file catch_block_driver.hpp
 * @brief 取块机构的硬件适配层：单位换算、硬件配置、采样、输出。
 *
 * 这层只干四件事：单位换算、硬件配置、采样、输出。
 * 它不认识状态机 —— 那是 catch_block.hpp 的事；它也不改电机库 —— Extensions70 只读。
 *
 * 【和电机库的分工】（库已回退到最初版本，这是本层存在的理由）
 *   库负责：CAN 收发、编码器解码、ADRC 闭环、电流爬坡与限幅、帧计数、在线计时。
 *   本层负责：SI 单位换算、零点与软行程、健康判据、锁存故障、超速保护、诊断映射。
 *
 *   回退前，健康判据（`HealthTerms()`）、锁存故障（`_fault` / `ResetFault()`）、
 *   反馈快照（`GetFeedbackSnapshot()` / `FeedbackFresh()`）、帧级守卫
 *   （`ConfigureFeedbackGuard()`）、发送结果（`LastTransmitResult()`）、
 *   电流爬坡（`CurrentSlew()`）、极性（`CurrentPolarity()`）这些东西长在库里。
 *   现在库里一个都没有了，全部由本层用**公开成员**重建，重建的取舍写在文件里每一处。
 *   唯一没有重建的是帧级守卫 —— 它在库里没有"是否使能"的前置条件，判中一次就永久
 *   冻结帧流，是"电机怎么都使不上"的直接原因。本层改成可恢复的超速锁存，理由见 Sample()。
 *
 * 运行上下文：所有方法只在 App 任务（5 ms 一拍）里调用，内部不加临界区；
 *            唯一例外是微秒时钟那几行关中断，它和 1 kHz 的 CAN 接收中断抢同一个 CYCCNT。
 */

// Keil 手动强制输出：写非 0 为开，写 0 为关。写 0 是"撤力/松开"，不是刹车。
// 引脚按实际布线：PE6 = 电磁阀（夹爪），PE12 = 气泵（吸盘）。
extern "C" volatile uint8_t catch_pump_on;      ///< @brief PE12 气泵：非 0 = 吸附
extern "C" volatile uint8_t catch_valve_closed; ///< @brief PE6 电磁阀：非 0 = 夹爪闭合
extern "C" volatile uint8_t catch_motor_enable; ///< @brief 非 0 = 允许输出，还要过业务层那一关

class CatchBlockDriver
{
public:
  /**
   * @brief Configure() 的三种结果。
   * @note 前两种必须分开：Rejected 只算这次没配成，HardwareFault 要报故障。
   */
  enum class ConfigResult : uint8_t
  {
    Rejected,     ///< 前置条件不满足，什么都还没动
    Applied,      ///< 配好了，下一步可以标零
    HardwareFault ///< 前置过了，但控制器没真正初始化起来
  };

  /**
   * @brief 健康位：`HealthTerms()` 逐位说明"为什么不能出力"，0 = 全过。
   * @note 这 8 条判据和 `blockers = health << 4` 的位序与回退前完全一致，
   *       所以《状态位速查.md》里 ÷16 那张表照旧能用。
   *       但数据来源全换了：库里已经没有任何一条能查，全部由本层自己记账。
   */
  enum HealthBit : uint32_t
  {
    HealthCtrlNotConfigured = 1,  ///< 控制器没配好（config 没成功，或 ADRC 没真正 Init）
    HealthFaultLatched = 2,       ///< 本层锁存了故障，要 ResetFault() 才清
    HealthFeedbackNeverSeen = 4,  ///< 一帧反馈都没收到：CAN 没通、编号不对、电调没上电
    HealthFeedbackInvalid = 8,    ///< 收到帧了，但还在库的 offset 初始化窗口里，累计角还没开始算
    HealthFeedbackFault = 16,     ///< 保留。回退后的库不再产生帧级故障码，此位恒 0
    HealthFeedbackStale = 32,     ///< 最近 feedback_timeout_us 内没有新帧（**不锁存，帧回来就自动消**）
    HealthAdrcInvalid = 64,       ///< ADRC 内部参数无效（Init 没跑起来）
    HealthTransmitFailed = 128    ///< 保留。BSP::CAN::Transmit 返回 void，拿不到发送结果，此位恒 0
  };

  /**
   * @brief 本层锁存的故障原码。
   * @note 位定义沿用回退前 `MotorDJI::FaultBits` 的布局，这样 `catch_debug.motor_faults`
   *       在 Keil 里读到的位号不变、文档不变。
   *       标"保留"的位本层不会置位 —— 对应的信号回退后确实拿不到了，写在这里是为了位号对齐。
   */
  enum DriverFault : uint32_t
  {
    DFaultNone = 0,
    DFaultConfigure = 1,   ///< 控制器没真正初始化起来（注册失败 / Apply() 后半路返回）
    DFaultFeedback = 2,    ///< 反馈数据非法：编码器角度码 > 8191（原库有这条检查，回退后没了）
    DFaultTiming = 4,      ///< 保留。1 kHz 控制拍的时序检查在库的 Control() 里，回退后拿不到
    DFaultNumeric = 8,     ///< 命令链出现 NaN / Inf（库回退后不再自查，由本层补）
    DFaultTravel = 16,     ///< 保留。行程越界由上层 Fail(Travel) 记；本层的软行程在 SetHeightTarget() 里挡
    DFaultTransmit = 32,   ///< 保留。库改用 void 的 BSP::CAN::Transmit，发送结果不可观测
    DFaultOverspeed = 64   ///< 使能期间转速超过 guard_rpm（连判 3 拍才锁），要 command = 5 才解
  };

  /**
   * @brief ArmAtCurrentPosition() 的四种结果。
   * @note 原来只返回 bool，三种失败在状态机里被压成同一个"未就绪"，
   *       调参时看不出到底是 CAN 没通还是机构在晃，所以拆开。
   */
  enum class ArmResult : uint8_t
  {
    Ok,            ///< 零点采好了
    NotConfigured, ///< 还没配置成功
    Unhealthy,     ///< 反馈或控制器不健康，具体哪条看 HealthTerms()
    Moving         ///< 机构还在动，速度高于 settled_speed_m_s
  };

  /** @name 配置与标零 */
  /** @{ */
  /**
   * @brief 把 CAN、控制器、软行程按参数配好。
   * @note 配好之后仍然不输出，要等上层使能。要求电机已经停下、两路 GPIO 有效。
   *       顺带把本层的故障锁存、帧间隔峰值全部清零 —— 一次完整重配就是一次干净开始。
   */
  ConfigResult Configure(const CatchBlockConfig &config);

  /**
   * @brief 把现在的位置记成 0 m，并让控制器以当前位置为保持目标。
   * @note 机构还在动就不记，免得把半路的位置当成原点。
   */
  ArmResult ArmAtCurrentPosition();

  /**
   * @brief 丢掉零点和"已配置"标记，回到不能动的状态。
   * @note 不清故障码，那个由 ResetFault() 单独负责。
   */
  void ResetConfiguration();
  /** @} */

  /** @name 采样与查询 */
  /** @{ */
  /**
   * @brief 把编码器、电流、温度、帧年龄读进下面的公有成员，并当场判超速。
   * @note 必须在健康判定和越界判定之前调用，否则检查用的是上一拍的数据。
   *       本拍的时间戳由它采集，之后用 SampledUs() 取。
   */
  void Sample();

  uint64_t SampledUs() const;       ///< @brief Sample() 那一刻的单调微秒，本拍的统一时间基准
  bool Ready() const;               ///< @brief 参数和控制器配好了没有
  bool OriginValid() const;         ///< @brief 零点采了没有（采了才能把角度换成米）
  bool Healthy() const;             ///< @brief 反馈和控制器有没有故障
  uint32_t HealthTerms() const;     ///< @brief Healthy() 没过的那几条，逐位；0 = 全过
  bool FeedbackSeen() const;        ///< @brief 收到过反馈帧没有，false 就是 CAN 一帧都没进来
  bool Enabled() const;             ///< @brief 驱动现在是不是正在输出电流
  bool LastTransmitOk() const;      ///< @brief 恒 true。库改回 void 之后发送结果不可观测，见实现里的注释
  uint32_t FaultCode() const;       ///< @brief 本层锁存的故障位（DriverFault）
  float TravelMetres() const;       ///< @brief 行程多少米
  float TravelMarginMetres() const; ///< @brief 越界判定在两端各放宽多少米
  /** @} */

  /** @name 输出 */
  /** @{ */
  /**
   * @brief 把两个继电器开关写到引脚上；顺带看总开关，为 0 就撤力。
   * @note 继电器写在最前面，不受任何使能条件影响，方便单独做硬件通断测试。
   *       返回 false 表示"这一拍别再去尝试使能了"。
   */
  bool ApplyManualOverrides();

  /**
   * @brief 以当前位置为保持目标，然后开始输出。
   * @note 不会沿用上一次的目标，避免突然冲出去；同时把 TD/ESO 一起重置成"原地不动"，
   *       等于从当前位置无扰接管。已经使能时直接返回，不重置控制器。
   *       要"在已经使能的状态下原地停住"请用 HoldHere()，那个不重置观测器。
   */
  bool Enable();

  void Disable();                     ///< @brief 停止输出。重复调用没关系
  bool SetHeightTarget(float metres); ///< @brief 把"多少米"换成编码值目标下发；没标零或超行程会拒绝
  /**
   * @brief 把保持目标改成当前位置，等于原地停住。
   * @note 只改运动参考（目标 + TD 的 v1/v2/v3），**不重置 ESO**：
   *       重力扰动估计 z3、输出电流、积分项全部保留。这是回退前 `MotorDJI::HoldHere()`
   *       的原始语义，用公开成员复刻 —— 不能拿 `Uneutral()` 图省事，那会把 z3 清零。
   */
  bool HoldHere();
  void ResetFault();                  ///< @brief 清本层锁存的故障位。要求已经撤力，带故障出力是最危险的状态
  void Neutral();                     ///< @brief 退出闭环，不再控制
  void Uneutral();                    ///< @brief 重新进入位置闭环
  /** @} */

  /** @name 标志位查询 */
  /** @{ */
  bool MotorEnableRequested() const; ///< @brief Keil 那个总开关是不是被打开了
  bool PumpRequested() const;        ///< @brief 气泵是不是被要求打开（是意图，不是引脚电平）
  bool ValveRequested() const;       ///< @brief 夹爪是不是被要求闭合
  bool PumpPinLevel() const;         ///< @brief PE12 现在实际是高还是低，只看不判断
  bool ValvePinLevel() const;        ///< @brief PE6 现在实际是高还是低
  /** @} */

  /** @name 采样结果（Sample() 刷新，外部只读） */
  /** @{ */
  float height_m = 0;          ///< @brief 相对原点的高度，m
  float velocity_m_s = 0;      ///< @brief 高度方向的速度，m/s
  float rotor_rad = 0;         ///< @brief 编码器累计的转子角度，rad
  bool feedback_valid = false; ///< @brief 反馈过了库的初始化窗口，且没超过 feedback_timeout_us
  uint32_t feedback_age_ms = 0; ///< @brief 距上一帧反馈过了多少 ms。超过 config.feedback_timeout_us/1000 就会判 Stale
  uint32_t feedback_age_peak_ms = 0; ///< @brief 自上次 command = 1 以来见过的最长帧间隔，ms。判"到底有没有断过帧"
  uint16_t feedback_hz = 0;     ///< @brief 库统计的反馈帧率，Hz。正常 C620 是 1000；掉一半说明总线有丢帧或被别的报文挤占
  /** @} */

  /** @name 控制器诊断（Sample() 刷新，外部只读，调 ADRC 时看这些） */
  /** @{ */
  float target_position_rad = 0;       ///< @brief 位置环想要到达的角度，rad
  float measured_position_rad = 0;     ///< @brief 编码器实测的角度，rad
  float reference_position_rad = 0;    ///< @brief TD 平滑后的参考位置 v1，rad
  float reference_velocity_rad_s = 0;  ///< @brief TD 平滑后的参考速度 v2，rad/s
  float eso_position_rad = 0;          ///< @brief ESO 估的位置 z1，rad
  float eso_velocity_rad_s = 0;        ///< @brief ESO 估的速度 z2，rad/s
  float disturbance_rad_s2 = 0;        ///< @brief ESO 估的总扰动 z3，rad/s²
  float raw_current_A = 0;             ///< @brief ADRC 算出来的电流，A。库在内部就限幅了，所以与下一项相等；撤力时为 0
  float limited_current_A = 0;         ///< @brief 限幅之后的电流，A。与上一项相同，撤力时为 0
  float final_current_A = 0;           ///< @brief 真正发出去的电流（已过库内爬坡），A
  float accepted_command_A = 0;        ///< @brief 与 final 相同。库不再回报邮箱结果，没有第二个来源
  int16_t feedback_current_raw = 0;    ///< @brief 电调回报的电流原始码，没标定
  bool feedback_current_valid = false; ///< @brief false 时上面这个原始码不能当安培用
  uint8_t temperature_C = 0;           ///< @brief 电调温度，°C
  /** @} */

private:
  MotorDJI motor_;                      ///< @brief 电机控制层：驱动 + ADRC + 电流链路
  BSP::GPIO::Inst pump_{Pin{'E', 12}};  ///< @brief PE12 气泵继电器（吸盘，标签 XiPan）
  BSP::GPIO::Inst valve_{Pin{'E', 6}};  ///< @brief PE6 电磁阀继电器（夹爪，标签 DianCiFa）

  CatchBlockConfig config_;        ///< @brief 生效的硬件参数，只在 Configure() 里赋值
  float origin_rad_ = 0;           ///< @brief 零点对应的转子角度，rad
  float metres_per_rotor_rad_ = 0; ///< @brief 转子转 1 弧度走多少米，1:1 传动约 1.3019 mm/rad，全工程只算这一处
  /**
   * @brief 当前 motor_ 这个对象在库的接收表里占的是哪个电调编号，0 = 还没注册过
   * @note  必须和 configured_ 分开记：ResetConfiguration() 会把 configured_ 清掉，
   *        但库那边的槽位注册**不会**跟着取消 —— 那个对象一旦在 slot 上就撤不回来了。
   *        回退后的 MotorDJI_Driver::Init() 换 ID 注册时会直接 return（"already registered
   *        at slot N"），而 can_inst.can_id 仍然留着上一次的旧句柄，查不出失败。
   *        所以换 ID 必须在这一层拒掉，否则会拿着旧 ID 的收发通道冒充新 ID 跑起来。
   */
  uint8_t registered_esc_id_ = 0;
  bool configured_ = false;        ///< @brief 控制器已配置（含 Apply() 后置校验通过）
  bool origin_valid_ = false;      ///< @brief 零点有效
  float guard_rpm_ = 0;            ///< @brief 超速门限，转子转速 rpm
  uint32_t faults_ = 0;            ///< @brief 本层锁存的 DriverFault 位
  uint32_t overspeed_run_ = 0;     ///< @brief 连续超标拍数，用来滤掉单拍毛刺

  uint32_t last_msg_cnt_ = 0;      ///< @brief 上一拍看到的库内帧计数，用来判"这一拍有没有新帧"
  uint64_t last_frame_us_ = 0;     ///< @brief 最后一帧到达时刻，µs
  uint64_t sample_us_ = 0;         ///< @brief 本拍 Sample() 的时间戳，µs

  uint32_t clock_last_cnt_ = 0;    ///< @brief 微秒时钟的上一次 CYCCNT
  uint64_t clock_acc_cnt_ = 0;     ///< @brief 微秒时钟的 64 位累加计数（CYCCNT 差值）

  uint64_t AdvanceClock();         ///< @brief 读 CYCCNT 并推进累加器，返回单调微秒
  uint32_t FeedbackAgeUs() const;  ///< @brief 距上一帧的精确微秒；从没收到过返回 60 s
  bool LibraryConfigured() const;  ///< @brief Apply() 到底成没成（反查 ADRC 的公开量）
  bool AdrcConfigured() const;     ///< @brief ADRC 的 Init 跑过没有，供健康位用
  void Latch(uint32_t fault);      ///< @brief 置锁存位，只增不减
};
