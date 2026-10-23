#include "catch_block.hpp"
#include "bsp_hardware.hpp"
#include "bsp_dwt.hpp"
#include <cmath>

// 三个硬件开关与八个调试量的定义（声明见头文件），上电全为 0：
// 泵停、阀开、电机不输出、无待执行命令。
extern "C"
{
volatile uint8_t catch_pump_on = 0;
volatile uint8_t catch_valve_closed = 0;
volatile uint8_t catch_motor_enable = 0;
volatile uint8_t catch_debug_command = 0;
volatile uint8_t catch_debug_result = 0;
volatile float catch_debug_height_m = 0;
volatile float catch_debug_actual_height_m = 0;
volatile uint32_t catch_debug_faults = 0;
volatile uint8_t catch_debug_state = 0;
volatile uint8_t catch_debug_feedback_valid = 0;
volatile uint8_t catch_debug_motor_enabled = 0;
}

// Keil 里逐项填写的参数副本，默认值即结构体默认值。
CatchBlock::Config catch_debug_config;

namespace
{
// 关中断的短临界区，退出时恢复进入前的中断掩码。
struct Critical
{
  uint32_t mask;
  Critical()
    : mask(__get_PRIMASK()) // 先记录原状态，再关中断
  {
    __disable_irq();
    __DMB();
  }
  ~Critical()
  {
    __DMB();                // 保证写入对其他上下文可见后再恢复中断
    __set_PRIMASK(mask);
  }
};
// 有限正数判定，用于拒绝 NaN / Inf / 0。
bool Positive(float x)
{
  return std::isfinite(x) && x > 0;
}
// 编码器 1 count 对应的转子角度：2π/8192 ≈ 7.67e-4 rad。
constexpr float RadPerCount = 6.28318530718f / 8192.0f;
}

// 框架注册应用时调用一次：只置位并发布初始状态，不改写任何硬件。
void CatchBlock::Start()
{
  // 重复注册不重置运行中的状态，也不动阀门。
  if (started_)
  {
    return;
  }
  started_ = true;
  Publish(); // PE6/PE12 的上电电平由 CubeMX 初始化，本应用此处不写。
}
// 单槽邮箱：已有未处理命令时拒绝，保证命令不排队、不覆盖。
bool CatchBlock::Submit(const Request &request)
{
  Critical guard;
  if (pending_valid_ || request.command == Command::None)
  {
    return false;
  }
  pending_ = request;
  pending_valid_ = true;
  return true;
}
// 调试用：替调用者补齐"有支承/有外部保护"两条确认，其余校验仍由 Configure() 完成。
bool CatchBlock::ConfigureForTest(const Config &config)
{
  Request request;
  request.command = Command::Configure;
  request.config = config;
  request.supported_origin_confirmed = true;
  request.external_protection_confirmed = true;
  return Submit(request);
}
// 调试用：把"当前就有支承"作为输入，零点采样本身仍走 ArmAtOrigin()。
bool CatchBlock::SetOriginForTest()
{
  Request request;
  request.command = Command::ArmAtSupportedOrigin;
  request.supported_origin_confirmed = true;
  request.external_protection_confirmed = true;
  return Submit(request);
}
// 调试用：只负责入队；目标范围与状态检查在 Process() 里做。
bool CatchBlock::MoveToHeight(float height_m)
{
  Request request;
  request.command = Command::MoveToHeight;
  request.target_height_m = height_m;
  return Submit(request);
}
// 调试用：只负责入队；保持目标由控制任务里的 HoldHere() 设置。
bool CatchBlock::HoldPosition()
{
  Request request;
  request.command = Command::HoldCurrent;
  return Submit(request);
}
// 关中断拷贝，避免读到 Publish() 写到一半的结构体。
CatchBlock::Status CatchBlock::GetSnapshot() const
{
  Critical guard;
  return status_;
}
// 参数总检查：编号、两个方向符号、四个实测标志必须在，其余阈值必须自洽。
bool CatchBlock::Validate(const Config &c) const
{
  // 顺序：编号与方向符号 → 四个"已实测"标志 → 模型/带宽/限幅/量程的自洽性。
  return c.esc_id >= 1 && c.esc_id <= 8 &&
      (c.lift_encoder_sign == 1 || c.lift_encoder_sign == -1) &&
      (c.current_encoder_sign == 1 || c.current_encoder_sign == -1) &&
      c.direction_verified && c.model_verified && c.current_limits_verified && c.motion_limits_verified &&
      Positive(c.inertia_kg_m2) && Positive(c.torque_constant_Nm_A) &&
      std::isfinite(c.viscous_Nm_s_rad) && c.viscous_Nm_s_rad >= 0 &&
      Positive(c.max_current_A) && c.max_current_A <= 20 && Positive(c.current_slew_A_s) &&
      Positive(c.observer_Hz) && Positive(c.controller_Hz) && c.controller_Hz < c.observer_Hz &&
      Positive(c.travel_m) && Positive(c.pulley_radius_m) && Positive(c.external_speed_ratio) &&
      Positive(c.gearbox_ratio) && Positive(c.speed_m_s) && Positive(c.acceleration_m_s2) &&
      Positive(c.overspeed_m_s) && c.overspeed_m_s > c.speed_m_s &&
      Positive(c.position_tolerance_m) && c.position_tolerance_m < c.travel_m / 10 &&
      Positive(c.settled_speed_m_s) && c.settled_speed_m_s < c.speed_m_s &&
      Positive(c.travel_fault_margin_m) && c.travel_fault_margin_m < c.travel_m / 10 &&
      c.feedback_timeout_us >= 2000 && c.feedback_timeout_us <= 20000 &&
      c.settle_ms >= 100 && c.settle_ms <= 10000 && c.relay_dwell_ms <= 5000 &&
      c.phase_timeout_ms >= 1000 && c.phase_timeout_ms <= 600000;
}
// 切状态时同步复位阶段计时与"到位"计时，避免沿用上一阶段的判断。
void CatchBlock::Transition(State state, uint32_t now_ms)
{
  state_ = state;
  phase_start_ms_ = now_ms;
  settled_since_us_ = 0;
}
// 故障只有两种结局：还能闭环就锁在当前位置（FaultHold），否则断驱动（Fault）。
// 两者都不保证托住负载；泵阀保持原状，仍由 Keil 标志位直接控制。
void CatchBlock::Fail(uint32_t fault, bool can_hold, uint32_t now_ms)
{
  faults_ |= fault;
  if (can_hold && motor_.Healthy() && motor_.HoldHere())
  {
    Transition(State::FaultHold, now_ms);
  }
  else
  {
    motor_.driver.Disable(); // 驱动不可用时，载荷只能由外部支承承担。
    Transition(State::Fault, now_ms);
  }
}
// 唯一的高度→位置入口：把米换算成转子目标角；越界或未标零时返回 false 且不写目标。
bool CatchBlock::SetHeight(float metres)
{
  return origin_valid_ && std::isfinite(metres) && metres >= 0 && metres <= config_.travel_m &&
      motor_.SetPosRad(origin_rad_ + config_.lift_encoder_sign * metres / metres_per_rotor_rad_);
}
// 到位判定：使能中 + 位置误差够小 + 速度够小，并持续 settle_ms 才算成立。
bool CatchBlock::Settled(float target_m, uint64_t now_us)
{
  if (!motor_.driver.enabled || std::fabs(height_m_ - target_m) > config_.position_tolerance_m ||
      std::fabs(velocity_m_s_) > config_.settled_speed_m_s || !motor_.Healthy())
  {
    settled_since_us_ = 0;
    return false;
  }
  if (!settled_since_us_)
  {
    settled_since_us_ = now_us; // 第一次满足条件，开始计时
  }
  return now_us >= settled_since_us_ && now_us - settled_since_us_ >= uint64_t(config_.settle_ms) * 1000;
}
// 初始化驱动与位置环：要求电机禁用、有支承、参数合法；成功后仍不使能。
bool CatchBlock::Configure(const Request &r, uint32_t now_ms)
{
  // 前置条件：未使能、两条人工确认、参数合法、两路 GPIO 有效、不换 ESC 编号。
  if (motor_.driver.enabled || !r.supported_origin_confirmed ||
      !r.external_protection_confirmed || !Validate(r.config) ||
      !pump_.IsValid() || !valve_.IsValid() ||
      (configured_ && r.config.esc_id != config_.esc_id))
  {
    return false;
  }

  // 高度换算系数：带轮半径 × 齿轮增速 ÷ 减速比（单位 m/rad，转子侧）。
  const float metres_per_rad = r.config.pulley_radius_m * r.config.external_speed_ratio /
      r.config.gearbox_ratio;
  if (!Positive(metres_per_rad))
  {
    return false;
  }

  config_ = r.config;
  metres_per_rotor_rad_ = metres_per_rad;
  motor_.Init(Hardware::hcan_main, config_.esc_id, DJI_C620);
  if (!motor_.driver.Initialized())
  {
    Fail(Configuration, false, now_ms);
    return true;
  }

  const float rad_s = config_.speed_m_s / metres_per_rotor_rad_; // 规划限速 (rad/s)
  const float guard_rpm = config_.overspeed_m_s / metres_per_rotor_rad_ * 60.0f / 6.28318530718f;
  // 同一门限两用：帧级跳变/速度检查，以及控制级超速保护。超速即锁存，调参时要留余量。
  if (!motor_.driver.ConfigureFeedbackGuard(guard_rpm, config_.feedback_timeout_us))
  {
    Fail(Configuration, false, now_ms);
    return true;
  }

  motor_.ResetFault();
  // 链式配置：单位在此换算成 SI，驱动内部只在发包前做一次 A→code。
  motor_.SetDt(0.001f)
      .ConfigADRC()
      .AsPosC()
      .ADRC_Womega(config_.observer_Hz, config_.controller_Hz)
      .ADRC_Physic(config_.inertia_kg_m2, config_.torque_constant_Nm_A, config_.viscous_Nm_s_rad)
      .ADRC_Limit(config_.max_current_A)
      .ADRC_MaxPlannedVel(rad_s * 60.0f / 6.28318530718f)
      .CurrentSlew(config_.current_slew_A_s)
      .CurrentPolarity(config_.current_encoder_sign)
      .FeedbackTimeout(config_.feedback_timeout_us)
      .SpdLimit(guard_rpm)
      .MotionLimits(rad_s, config_.acceleration_m_s2 / metres_per_rotor_rad_)
      .Apply();
  if (!motor_.IsConfigured())
  {
    Fail(Configuration, false, now_ms);
    return true;
  }

  // 配置成功后清零故障与零点，必须重新 Arm 才能运动。
  configured_ = true;
  origin_valid_ = false;
  protection_ack_ = false;
  faults_ = 0;
  Transition(State::Idle, now_ms);
  return true;
}
// 在有支承的静止原点采样零点并设软限位；是否使能由 catch_motor_enable 决定。
bool CatchBlock::ArmAtOrigin(const Request &r, uint32_t now_ms)
{
  if (state_ != State::Idle || !configured_ || !r.supported_origin_confirmed ||
      !r.external_protection_confirmed || !motor_.Healthy())
  {
    return false;
  }

  // 运动中标零会污染零点，要求高度方向速度低于到位门限。
  const auto fb = motor_.driver.GetFeedbackSnapshot();
  const float measured_speed = std::fabs(float(fb.measure.speed_rpm)) *
      6.28318530718f / 60 * metres_per_rotor_rad_;
  if (measured_speed > config_.settled_speed_m_s)
  {
    return false;
  }

  origin_rad_ = float(fb.measure.total_angle) * RadPerCount;
  const float end = origin_rad_ + config_.lift_encoder_sign * config_.travel_m / metres_per_rotor_rad_;
  const float margin = config_.travel_fault_margin_m / metres_per_rotor_rad_;
  // 软限位 = [原点, 顶端] 两端各留 margin；它挡不住失电下坠。
  motor_.PositionLimits(fminf(origin_rad_, end) - margin, fmaxf(origin_rad_, end) + margin);
  origin_valid_ = true;
  protection_ack_ = true;
  motor_.Uneutral(PosC);
  if (!SetHeight(0))
  {
    Fail(Configuration, false, now_ms);
    return true;
  }

  Transition(State::OriginHold, now_ms);
  return true;
}
// 命令分发：每个 case 自己判断"当前状态 + 确认位 + 物理条件"，不满足就落到末尾计拒绝。
void CatchBlock::Process(const Request &r, uint32_t now_ms)
{
  switch (r.command)
  {
    case Command::Configure: // 一次性初始化，成功后直接返回
      if (Configure(r, now_ms))
      {
        return;
      }
      break;
    case Command::ArmAtSupportedOrigin: // 一次性标零，成功后直接返回
      if (ArmAtOrigin(r, now_ms))
      {
        return;
      }
      break;
    case Command::CloseGrip: // 原点稳住后才允许走夹取流程
      if (state_ != State::OriginHold || !r.external_protection_confirmed ||
          !Settled(0, previous_tick_us_))
      {
        break;
      }
      Transition(State::GripClose, now_ms);
      return;
    case Command::ConfirmGripAndLift: // 必须人工确认，且两路继电器确已为高
      if (state_ != State::GripVerify || !r.grip_confirmed || !r.external_protection_confirmed ||
          catch_pump_on == 0 || catch_valve_closed == 0 || !motor_.driver.enabled)
      {
        break;
      }
      if (!SetHeight(config_.travel_m))
      {
        Fail(Travel, true, now_ms);
        return;
      }
      Transition(State::Lifting, now_ms);
      return;
    case Command::PermitDrop: // 顶端稳住且投放区已隔离才许可
      if (state_ != State::TargetHold || !r.drop_zone_clear_confirmed ||
          !r.external_protection_confirmed || !Settled(config_.travel_m, previous_tick_us_))
      {
        break;
      }
      Transition(State::ReleaseReady, now_ms);
      return;
    case Command::ConfirmObjectClearAndReturn: // 不靠延时假定物块已离开
      if (state_ != State::Release || !r.object_clear_confirmed || !r.external_protection_confirmed ||
          now_ms - phase_start_ms_ < config_.relay_dwell_ms)
      {
        break;
      }
      if (!SetHeight(0))
      {
        Fail(Travel, true, now_ms);
        return;
      }
      Transition(State::Descending, now_ms);
      return;
    case Command::Hold: // 软急停：转 FaultHold 冻结动作
      if (!motor_.driver.enabled)
      {
        break;
      }
      Fail(OperatorHold, true, now_ms);
      return;
    case Command::MoveToHeight: // 调试位移：只允许从原点/手动保持出发
      if ((state_ != State::OriginHold && state_ != State::ManualHold) ||
          !motor_.driver.enabled || !motor_.Healthy() ||
          !std::isfinite(r.target_height_m) ||
          r.target_height_m < 0 || r.target_height_m > config_.travel_m)
      {
        break;
      }
      if (!SetHeight(r.target_height_m))
      {
        Fail(Travel, true, now_ms);
        return;
      }
      manual_target_m_ = r.target_height_m;
      Transition(State::ManualMoving, now_ms);
      return;
    case Command::HoldCurrent: // 以当前位置为目标，停在原地
      if ((state_ != State::OriginHold && state_ != State::ManualMoving &&
           state_ != State::ManualHold) || !motor_.driver.enabled ||
          !motor_.Healthy() || !motor_.HoldHere())
      {
        break;
      }
      manual_target_m_ = height_m_;
      Transition(State::ManualHold, now_ms);
      return;
    case Command::ResetWhileSupported: // 唯一能清反馈锁存的路径
      if (!r.supported_origin_confirmed || !r.external_protection_confirmed)
      {
        break;
      }
      motor_.driver.Disable();
      motor_.Neutral();
      motor_.driver.ResetFeedbackReference();
      motor_.ResetFault();
      configured_ = false;
      origin_valid_ = false;
      protection_ack_ = false;
      faults_ = 0;
      Transition(State::Unconfigured, now_ms); // 必须重新配置、标零、使能。
      return;
    default:
      break;
  }
  ++rejected_; // 入队成功不等于转换获准，这里统计被拒次数。
}
// 运行中检查：只在驱动使能时进行；任何一项异常都升级为故障。
void CatchBlock::CheckMotion(uint64_t now_us, uint32_t now_ms, uint64_t elapsed_us)
{
  if (!motor_.driver.enabled)
  {
    return;
  }
  // 应用周期异常（名义 5 ms，允许 2~15 ms）；注意这里量的是 App 任务，不是控制环。
  if (now_us < previous_tick_us_ || elapsed_us < 2000 || elapsed_us > 15000)
  {
    Fail(Timing, false, now_ms);
  }
  // 上一次 CAN 发送未被邮箱接受：电调可能仍在执行旧指令。
  if (motor_.driver.enabled && motor_.driver.LastTransmitResult() != MotorDJI_Driver::Result::Ok)
  {
    Fail(CanTransmit, false, now_ms);
  }
  if (!motor_.driver.enabled)
  {
    return;
  }
  // 控制器/反馈不健康，或高度越出 [0, travel] ± 余量。
  if (!motor_.Healthy())
  {
    Fail(Controller | Feedback, false, now_ms);
  }
  else if (!protection_ack_ || !origin_valid_ || !std::isfinite(height_m_) ||
           height_m_ < -config_.travel_fault_margin_m ||
           height_m_ > config_.travel_m + config_.travel_fault_margin_m)
  {
    Fail(Travel, false, now_ms);
  }
}
// 反馈 → 高度/速度：三个条件（已配置、已标零、反馈有效）同时成立才更新，否则保留旧值。
void CatchBlock::ReadFeedback()
{
  const auto feedback = motor_.driver.GetFeedbackSnapshot();
  if (configured_ && origin_valid_ && feedback.valid)
  {
    height_m_ = (float(feedback.measure.total_angle) * RadPerCount - origin_rad_) *
        config_.lift_encoder_sign * metres_per_rotor_rad_;
    velocity_m_s_ = float(feedback.measure.speed_rpm) * 6.28318530718f / 60 *
        config_.lift_encoder_sign * metres_per_rotor_rad_;
  }
}
// 从邮箱取一条命令：临界区只做"取走并清标志"，Process() 在临界区外执行。
void CatchBlock::ConsumeRequest(uint32_t now_ms)
{
  Request request;
  bool have_request = false;
  {
    Critical guard;
    have_request = pending_valid_;
    if (have_request)
    {
      request = pending_;
      pending_valid_ = false;
    }
  }
  if (have_request)
  {
    Process(request, now_ms);
  }
}
// 读取一条 Keil 命令：先清命令号保证只生效一次，再走与 Submit() 相同的 Process()。
void CatchBlock::ConsumeDebugCommand(uint32_t now_ms)
{
  const uint8_t command = catch_debug_command;
  if (command == 0)
  {
    return;
  }
  catch_debug_command = 0;
  Request request;
  switch (command)
  {
    case 1: // 配置：参数整体取自 catch_debug_config
      request.command = Command::Configure;
      request.config = catch_debug_config;
      request.supported_origin_confirmed = true;
      request.external_protection_confirmed = true;
      break;
    case 2: // 当前位置标零
      request.command = Command::ArmAtSupportedOrigin;
      request.supported_origin_confirmed = true;
      request.external_protection_confirmed = true;
      break;
    case 3: // 移动到 catch_debug_height_m
      request.command = Command::MoveToHeight;
      request.target_height_m = catch_debug_height_m;
      break;
    case 4: // 保持当前位置
      request.command = Command::HoldCurrent;
      break;
    default:
      catch_debug_result = 2; // 未知命令号，直接记"被拒绝"
      return;
  }
  // 用 rejected_ 的增量判断是否被拒：Process 里被拒必然自增。
  const uint32_t rejected_before = rejected_;
  Process(request, now_ms);
  catch_debug_result = rejected_ != rejected_before ? 2 :
      (state_ == State::Fault || state_ == State::FaultHold ? 3 : 1);
}
// 自动推进：稳定判定、流程串联、手动位移完成、阶段超时；每拍最多前进一个状态。
void CatchBlock::Advance(uint64_t now_us, uint32_t now_ms)
{
  // 两个"保持态"需要持续刷新到位计时，供后续命令判断。
  if (state_ == State::OriginHold)
  {
    Settled(0, now_us);
  }
  if (state_ == State::TargetHold)
  {
    Settled(config_.travel_m, now_us);
  }
  if (state_ == State::GripClose && now_ms - phase_start_ms_ >= config_.relay_dwell_ms)
  {
    Transition(State::SuctionOn, now_ms);
  }
  else if (state_ == State::SuctionOn && now_ms - phase_start_ms_ >= config_.relay_dwell_ms &&
           catch_pump_on != 0 && catch_valve_closed != 0) // 两路确已为高才继续
  {
    Transition(State::GripVerify, now_ms); // 之后只能由人工确认推进
  }
  else if (state_ == State::Lifting && Settled(config_.travel_m, now_us))
  {
    Transition(State::TargetHold, now_ms);
  }
  else if (state_ == State::ReleaseReady && now_ms != phase_start_ms_)
  {
    // 投放前再校验一拍：健康、位置、速度都满足才真正进入 Release。
    if (!motor_.Healthy() || std::fabs(height_m_ - config_.travel_m) > config_.position_tolerance_m ||
        std::fabs(velocity_m_s_) > config_.settled_speed_m_s)
    {
      Fail(Controller, true, now_ms);
    }
    else
    {
      Transition(State::Release, now_ms);
    }
  }
  else if (state_ == State::Descending && Settled(0, now_us))
  {
    Transition(State::OriginHold, now_ms); // 不自动重复循环，也不自动断电流。
  }
  else if (state_ == State::ManualMoving && Settled(manual_target_m_, now_us))
  {
    Transition(State::ManualHold, now_ms);
  }
  // 超时检查放在最后，避免刚转换就被判超时；等待人工确认的态也计入。
  const bool phase_timed = state_ == State::GripClose || state_ == State::SuctionOn ||
      state_ == State::GripVerify || state_ == State::Lifting || state_ == State::TargetHold ||
      state_ == State::Release || state_ == State::Descending || state_ == State::ManualMoving;
  if (phase_timed && now_ms - phase_start_ms_ > config_.phase_timeout_ms)
  {
    Fail(Timeout, true, now_ms);
  }
}
// 框架 App 任务入口（名义 5 ms）：顺序固定，先读反馈查故障，再收命令推状态，最后写输出。
// 电流环由 ControlAllMotors() 在另一个任务里按 1 kHz 独立执行。
void CatchBlock::Update()
{
  if (!started_)
  {
    return;
  }
  uint64_t now_us;
  {
    Critical guard;
    now_us = DWT_GetTimeline_USec();
  }
  const uint32_t now_ms = uint32_t(now_us / 1000);
  // 首拍没有上一拍时间，按名义 5 ms 处理。
  const uint64_t elapsed_us = previous_tick_us_ && now_us >= previous_tick_us_ ?
      now_us - previous_tick_us_ : 5000;
  // 使能标志为 0 时立即撤力，不等状态机。
  if (catch_motor_enable == 0 && motor_.driver.enabled)
  {
    motor_.driver.Disable();
  }
  ReadFeedback();
  CheckMotion(now_us, now_ms, elapsed_us);
  previous_tick_us_ = now_us;
  ConsumeRequest(now_ms);
  ConsumeDebugCommand(now_ms);
  Advance(now_us, now_ms);
  ApplyDebugOutputs();
  Publish();
}
// 输出层：继电器无条件跟随标志位；电机仅在"已配置 + 已标零 + 状态允许 + 健康"时使能。
void CatchBlock::ApplyDebugOutputs()
{
  pump_.Write(catch_pump_on != 0);
  valve_.Write(catch_valve_closed != 0);

  if (catch_motor_enable == 0)
  {
    motor_.driver.Disable();
    return;
  }
  // 已使能则不再重复使能；状态不在白名单或控制器不健康则保持现状。
  if (motor_.driver.enabled || !configured_ || !origin_valid_ ||
      (state_ != State::OriginHold && state_ != State::GripClose &&
       state_ != State::SuctionOn && state_ != State::GripVerify &&
       state_ != State::Lifting && state_ != State::TargetHold &&
       state_ != State::ReleaseReady && state_ != State::Release &&
       state_ != State::Descending && state_ != State::ManualMoving &&
       state_ != State::ManualHold) || !motor_.Healthy())
  {
    return;
  }
  // 使能瞬间以当前位置为保持目标，避免续用旧的运动目标。
  if (motor_.HoldHere())
  {
    motor_.driver.Enable();
    // 升降/手动位移途中重新使能，视为人工中断，转故障保持。
    if (state_ == State::Lifting || state_ == State::Descending ||
        state_ == State::ManualMoving)
    {
      faults_ |= OperatorHold;
      Transition(State::FaultHold, uint32_t(previous_tick_us_ / 1000));
    }
  }
}
// 汇总结论到局部变量后一次性替换 status_，再同步 Keil 可见的镜像量。
void CatchBlock::Publish()
{
  Status next;
  next.state = state_;
  next.faults = faults_;
  next.motor_faults = motor_.FaultCode();
  next.rejected_requests = rejected_;
  next.height_m = height_m_;
  next.velocity_m_s = velocity_m_s_;
  next.target_position_rad = motor_.targ_pos_rad;
  next.measured_position_rad = motor_.position_rad;
  next.reference_position_rad = motor_.motor_adrc.input_nltd_3rd.v1;
  next.reference_velocity_rad_s = motor_.motor_adrc.input_nltd_3rd.v2;
  next.eso_position_rad = motor_.motor_adrc.eso.z1;
  next.eso_velocity_rad_s = motor_.motor_adrc.eso.z2;
  next.disturbance_rad_s2 = motor_.motor_adrc.eso.z3;
  next.raw_current_A = motor_.raw_current_A;
  next.limited_current_A = motor_.limited_current_A;
  next.final_current_A = motor_.final_current_A;
  next.accepted_command_A = motor_.driver.LastAcceptedCommandAmp();
  const auto fb = motor_.driver.GetFeedbackSnapshot();
  next.feedback_current_raw = fb.measure.raw_current;
  next.feedback_current_A_valid = fb.measure.current_amp_valid;
  next.feedback_valid = fb.valid;
  next.temperature_C = fb.measure.temprature;
  next.pump_on = pump_.Read();
  next.valve_closed = valve_.Read();
  next.motor_enabled = motor_.driver.enabled;
  Critical guard;
  status_ = next; // 只有赋值这一段关中断，前面拼装不关
  catch_debug_actual_height_m = next.height_m;
  catch_debug_faults = next.faults | next.motor_faults;
  catch_debug_state = static_cast<uint8_t>(next.state);
  catch_debug_feedback_valid = next.feedback_valid ? 1 : 0;
  catch_debug_motor_enabled = next.motor_enabled ? 1 : 0;
}
