#include "catch_block.hpp"
#include "bsp_dwt.hpp"
#include <cmath>

// 全局唯一的调试窗口实例。上电全 0：泵停、阀开、电机不输出、无待执行命令。
CatchBlockDebug catch_debug;

namespace
{
/**
 * @brief 关中断的短临界区
 * @note 退出时恢复进入前的中断掩码，不做成"无条件开中断"。
 */
class Critical
{
public:
  Critical()
    : mask_(__get_PRIMASK()) // 先记录原状态，再关中断
  {
    __disable_irq();
    __DMB();
  }
  ~Critical()
  {
    __DMB(); // 保证写入对其他上下文可见后再恢复中断
    __set_PRIMASK(mask_);
  }

private:
  uint32_t mask_;
};
}

/** @name 命令入口 */
/** @{ */
bool CatchBlock::Configure(const Config &config)
{
  return Enqueue(Command::Configure, config);
}

bool CatchBlock::ArmAtOrigin()
{
  return Enqueue(Command::ArmAtOrigin);
}

bool CatchBlock::CloseGrip()
{
  return Enqueue(Command::CloseGrip);
}

bool CatchBlock::ConfirmGripAndLift()
{
  return Enqueue(Command::ConfirmGripAndLift);
}

bool CatchBlock::PermitDrop()
{
  return Enqueue(Command::PermitDrop);
}

bool CatchBlock::ConfirmObjectClearAndReturn()
{
  return Enqueue(Command::ConfirmObjectClearAndReturn);
}

bool CatchBlock::SoftStop()
{
  return Enqueue(Command::SoftStop);
}

bool CatchBlock::ResetWhileSupported()
{
  return Enqueue(Command::ResetWhileSupported);
}

bool CatchBlock::MoveToHeight(float height_m)
{
  return Enqueue(Command::MoveToHeight, height_m);
}

bool CatchBlock::HoldHere()
{
  return Enqueue(Command::HoldHere);
}
/** @} */

/** @name 邮箱 */
/** @{ */
bool CatchBlock::Enqueue(Command command)
{
  Critical guard;
  if (pending_command_ != Command::None || command == Command::None)
  {
    return false;
  }
  pending_command_ = command;
  return true;
}

bool CatchBlock::Enqueue(Command command, float height_m)
{
  Critical guard;
  if (pending_command_ != Command::None)
  {
    return false;
  }
  pending_height_m_ = height_m;
  pending_command_ = command; // 命令最后写，保证载荷先就绪
  return true;
}

bool CatchBlock::Enqueue(Command command, const Config &config)
{
  Critical guard;
  if (pending_command_ != Command::None)
  {
    return false;
  }
  pending_config_ = config;
  pending_command_ = command;
  return true;
}

/**
 * @brief 从邮箱取一条命令
 * @note 临界区里只做"拷走并清标志"，Process() 在临界区外执行，避免长时间关中断。
 */
void CatchBlock::ConsumeRequest(uint32_t now_ms)
{
  Command command = Command::None;
  Config config;
  float height_m = 0;
  {
    Critical guard;
    command = pending_command_;
    if (command != Command::None)
    {
      config = pending_config_;
      height_m = pending_height_m_;
      pending_command_ = Command::None;
    }
  }
  if (command == Command::None)
  {
    return;
  }
  Process(command, config, height_m, now_ms);
}

/**
 * @brief 读取一条 Keil 命令
 * @note 先清命令号保证只生效一次，再走与公开命令完全相同的 Process() 路径。
 */
void CatchBlock::ConsumeDebugCommand(uint32_t now_ms)
{
  const uint8_t code = catch_debug.command;
  if (code == 0)
  {
    return;
  }
  catch_debug.command = 0;

  Command command = Command::None;
  Config config;
  float height_m = 0;
  switch (code)
  {
    case 1: // 配置：参数整体取自 catch_debug.config
      command = Command::Configure;
      config = catch_debug.config;
      break;
    case 2: // 当前位置标零
      command = Command::ArmAtOrigin;
      break;
    case 3: // 移动到 catch_debug.target_height_m
      command = Command::MoveToHeight;
      height_m = catch_debug.target_height_m;
      break;
    case 4: // 保持当前位置
      command = Command::HoldHere;
      break;
    case 5: // 清故障、清反馈锁存，回到未配置（须有支承，否则零点会丢）
      command = Command::ResetWhileSupported;
      break;
    case 6: // 软急停：冻结动作并锁在当前位置，比直接撤力安全
      command = Command::SoftStop;
      break;
    default:
      catch_debug.result = 2; // 未知命令号，直接记"被拒绝"
      return;
  }

  // 用 rejected_ 的增量判断是否被拒：Process 里被拒必然自增。
  const uint32_t rejected_before = rejected_;
  Process(command, config, height_m, now_ms);
  catch_debug.result = rejected_ != rejected_before ? 2 :
      (state_ == State::Fault || state_ == State::FaultHold ? 3 : 1);
}
/** @} */

/** @name 框架回调 */
/** @{ */
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

/**
 * @brief 框架 App 任务入口（名义 5 ms）
 * @note 顺序固定：先撤力与采样，再查故障，最后收命令、推状态、写输出。
 *       采样必须早于 CheckMotion，否则检查会用上一拍的数据。
 */
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
  if (!driver_.MotorEnableRequested() && driver_.Enabled())
  {
    driver_.Disable();
  }

  driver_.Sample();
  CheckMotion(now_us, now_ms, elapsed_us);
  previous_tick_us_ = now_us;

  ConsumeRequest(now_ms);
  ConsumeDebugCommand(now_ms);
  Advance(now_us, now_ms);
  ApplyOutputs();
  Publish();
}
/** @} */

/** @name 状态机 */
/** @{ */
/**
 * @brief 命令分发
 * @note 每个 case 自己判断"当前状态 + 物理条件"，不满足就落到末尾计一次拒绝。
 *       人工确认已经写进命令名里，所以这里不再逐条检查确认位。
 */
void CatchBlock::Process(Command command, const Config &config, float height_m, uint32_t now_ms)
{
  switch (command)
  {
    case Command::Configure:
      if (ApplyConfigure(config, now_ms))
      {
        return;
      }
      break;
    case Command::ArmAtOrigin:
      if (DoArmAtOrigin(now_ms))
      {
        return;
      }
      break;
    case Command::CloseGrip: // 原点稳住后才允许走夹取流程
      if (state_ != State::OriginHold || !Settled(0, previous_tick_us_))
      {
        break;
      }
      Transition(State::GripClose, now_ms);
      return;
    case Command::ConfirmGripAndLift: // 两路继电器确已为高才允许上升
      if (state_ != State::GripVerify || !driver_.PumpRequested() ||
          !driver_.ValveRequested() || !driver_.Enabled())
      {
        break;
      }
      if (!ApplyHeight(driver_.TravelMetres(), now_ms))
      {
        return;
      }
      Transition(State::Lifting, now_ms);
      return;
    case Command::PermitDrop: // 顶端稳住才许可
      if (state_ != State::TargetHold || !Settled(driver_.TravelMetres(), previous_tick_us_))
      {
        break;
      }
      Transition(State::ReleaseReady, now_ms);
      return;
    case Command::ConfirmObjectClearAndReturn: // 转 Release 后至少等够一个动作间隔
      if (state_ != State::Release || now_ms - phase_start_ms_ < config_.relay_dwell_ms)
      {
        break;
      }
      if (!ApplyHeight(0, now_ms))
      {
        return;
      }
      Transition(State::Descending, now_ms);
      return;
    case Command::SoftStop: // 软急停：转 FaultHold 冻结动作
      if (!driver_.Enabled())
      {
        break;
      }
      Fail(OperatorHold, true, now_ms);
      return;
    case Command::MoveToHeight: // 调试位移：只允许从原点/手动保持出发
      if ((state_ != State::OriginHold && state_ != State::ManualHold) ||
          !driver_.Enabled() || !driver_.Healthy() || !std::isfinite(height_m) ||
          height_m < 0 || height_m > driver_.TravelMetres())
      {
        break;
      }
      if (!ApplyHeight(height_m, now_ms))
      {
        return;
      }
      manual_target_m_ = height_m;
      Transition(State::ManualMoving, now_ms);
      return;
    case Command::HoldHere: // 以当前位置为目标，停在原地
      if ((state_ != State::OriginHold && state_ != State::ManualMoving &&
           state_ != State::ManualHold) || !driver_.Enabled() ||
          !driver_.Healthy() || !driver_.HoldHere())
      {
        break;
      }
      manual_target_m_ = driver_.height_m;
      Transition(State::ManualHold, now_ms);
      return;
    case Command::ResetWhileSupported: // 唯一能清反馈锁存的路径
      driver_.Disable();
      driver_.Neutral();
      driver_.ResetFeedbackReference();
      driver_.ResetFault();
      driver_.ResetConfiguration();
      protection_ack_ = false;
      faults_ = 0;
      Transition(State::Unconfigured, now_ms); // 必须重新配置、标零、使能。
      return;
    default:
      break;
  }
  ++rejected_; // 入队成功不等于转换获准，这里统计被拒次数。
}

/**
 * @brief 下发参数并初始化驱动
 * @note 前置条件：电机已停、业务阈值与硬件参数都合法；配置成功后必须重新标零。
 */
bool CatchBlock::ApplyConfigure(const Config &config, uint32_t now_ms)
{
  if (driver_.Enabled() || !config.Validate() || !config.ValidateHardware())
  {
    return false;
  }

  const CatchBlockDriver::ConfigResult result = driver_.Configure(config);
  if (result == CatchBlockDriver::ConfigResult::Rejected)
  {
    return false;
  }

  config_ = config;
  if (result == CatchBlockDriver::ConfigResult::HardwareFault)
  {
    Fail(Configuration, false, now_ms);
    return true;
  }

  protection_ack_ = false;
  faults_ = 0;
  Transition(State::Idle, now_ms);
  return true;
}

/**
 * @brief 在有支承的静止原点采样零点并设软限位
 * @note 是否使能由 catch_motor_enable 决定，标零本身不会让机构动。
 */
bool CatchBlock::DoArmAtOrigin(uint32_t now_ms)
{
  if (state_ != State::Idle || !driver_.ArmAtCurrentPosition())
  {
    return false;
  }

  protection_ack_ = true;
  if (!driver_.SetHeightTarget(0))
  {
    Fail(Configuration, false, now_ms);
    return true;
  }

  Transition(State::OriginHold, now_ms);
  return true;
}

/**
 * @brief 切状态并复位本阶段计时
 * @note 同时清掉"到位计时"，避免沿用上一阶段的判断。
 */
void CatchBlock::Transition(State state, uint32_t now_ms)
{
  state_ = state;
  phase_start_ms_ = now_ms;
  settled_since_us_ = 0;
}

/**
 * @brief 锁存故障并选择"保持"还是"断驱动"
 * @note 还能闭环就锁在当前位置（FaultHold），否则断驱动（Fault）。两者都不保证托住负载。
 *       "能不能保持"是安全策略，只能在本层决定，不能下放给驱动层。
 */
void CatchBlock::Fail(uint32_t fault, bool can_hold, uint32_t now_ms)
{
  faults_ |= fault;
  if (can_hold && driver_.Healthy() && driver_.HoldHere())
  {
    Transition(State::FaultHold, now_ms);
  }
  else
  {
    driver_.Disable(); // 驱动不可用时，载荷只能由外部支承承担。
    Transition(State::Fault, now_ms);
  }
}

/**
 * @brief 自动推进与阶段超时
 * @note 每拍最多前进一个状态；超时检查放在最后，避免刚转换就被判超时。
 */
void CatchBlock::Advance(uint64_t now_us, uint32_t now_ms)
{
  // 两个"保持态"需要持续刷新到位计时，供后续命令判断。
  if (state_ == State::OriginHold)
  {
    Settled(0, now_us);
  }
  if (state_ == State::TargetHold)
  {
    Settled(driver_.TravelMetres(), now_us);
  }

  if (state_ == State::GripClose && now_ms - phase_start_ms_ >= config_.relay_dwell_ms)
  {
    Transition(State::SuctionOn, now_ms);
  }
  else if (state_ == State::SuctionOn && now_ms - phase_start_ms_ >= config_.relay_dwell_ms &&
           driver_.PumpRequested() && driver_.ValveRequested()) // 两路确已为高才继续
  {
    Transition(State::GripVerify, now_ms); // 之后只能由人工确认推进
  }
  else if (state_ == State::Lifting && Settled(driver_.TravelMetres(), now_us))
  {
    Transition(State::TargetHold, now_ms);
  }
  else if (state_ == State::ReleaseReady && now_ms != phase_start_ms_)
  {
    // 投放前再校验一拍：健康、位置、速度都满足才真正进入 Release。
    if (!driver_.Healthy() ||
        std::fabs(driver_.height_m - driver_.TravelMetres()) > config_.position_tolerance_m ||
        std::fabs(driver_.velocity_m_s) > config_.settled_speed_m_s)
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

  // 等待人工确认的态也计入超时。
  const bool phase_timed = state_ == State::GripClose || state_ == State::SuctionOn ||
      state_ == State::GripVerify || state_ == State::Lifting || state_ == State::TargetHold ||
      state_ == State::Release || state_ == State::Descending || state_ == State::ManualMoving;
  if (phase_timed && now_ms - phase_start_ms_ > config_.phase_timeout_ms)
  {
    Fail(Timeout, true, now_ms);
  }
}
/** @} */

/** @name 判据与检查 */
/** @{ */
/**
 * @brief 使能白名单
 * @note 除 Unconfigured / Idle / FaultHold / Fault 外，其余状态都允许电机输出。
 */
bool CatchBlock::StateAllowsEnable() const
{
  return state_ == State::OriginHold || state_ == State::GripClose ||
      state_ == State::SuctionOn || state_ == State::GripVerify ||
      state_ == State::Lifting || state_ == State::TargetHold ||
      state_ == State::ReleaseReady || state_ == State::Release ||
      state_ == State::Descending || state_ == State::ManualMoving ||
      state_ == State::ManualHold;
}

/**
 * @brief 到位判定
 * @note 使能中 + 位置误差够小 + 速度够小，并持续 settle_ms 才算成立。
 */
bool CatchBlock::Settled(float target_m, uint64_t now_us)
{
  if (!driver_.Enabled() || std::fabs(driver_.height_m - target_m) > config_.position_tolerance_m ||
      std::fabs(driver_.velocity_m_s) > config_.settled_speed_m_s || !driver_.Healthy())
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

/**
 * @brief 高度目标下发 + 失败上报
 * @note 与拆分前一致：所有失败一律记 Travel 故障，不区分"参数没配好"和"真的越界"。
 */
bool CatchBlock::ApplyHeight(float metres, uint32_t now_ms)
{
  if (driver_.SetHeightTarget(metres))
  {
    return true;
  }
  Fail(Travel, true, now_ms);
  return false;
}

/**
 * @brief 运行中检查
 * @note 只在驱动使能时进行：应用周期、CAN 发送结果、控制器健康、高度越界，任何一项异常都升级为故障。
 */
void CatchBlock::CheckMotion(uint64_t now_us, uint32_t now_ms, uint64_t elapsed_us)
{
  if (!driver_.Enabled())
  {
    return;
  }
  // 应用周期异常（名义 5 ms，允许 2~15 ms）；注意这里量的是 App 任务，不是控制环。
  if (now_us < previous_tick_us_ || elapsed_us < 2000 || elapsed_us > 15000)
  {
    Fail(Timing, false, now_ms);
  }
  // 上一次 CAN 发送未被邮箱接受：电调可能仍在执行旧指令。
  if (driver_.Enabled() && !driver_.LastTransmitOk())
  {
    Fail(CanTransmit, false, now_ms);
  }
  if (!driver_.Enabled())
  {
    return;
  }
  // 控制器/反馈不健康，或高度越出 [0, travel] ± 余量。
  if (!driver_.Healthy())
  {
    Fail(Controller | Feedback, false, now_ms);
  }
  else if (!protection_ack_ || !driver_.OriginValid() || !std::isfinite(driver_.height_m) ||
           driver_.height_m < -driver_.TravelMarginMetres() ||
           driver_.height_m > driver_.TravelMetres() + driver_.TravelMarginMetres())
  {
    Fail(Travel, false, now_ms);
  }
}
/** @} */

/** @name 输出与调试窗口 */
/** @{ */
/**
 * @brief 输出层
 * @note 继电器与撤力交给驱动层；本层只决定"这个状态该不该使能"。
 */
void CatchBlock::ApplyOutputs()
{
  // 驱动层已完成两路继电器写入，并在 catch_motor_enable 为 0 时撤力。
  if (!driver_.ApplyManualOverrides())
  {
    return;
  }
  // 已使能则不再重复使能；状态不在白名单或控制器不健康则保持现状。
  if (driver_.Enabled() || !driver_.Ready() || !driver_.OriginValid() ||
      !StateAllowsEnable() || !driver_.Healthy())
  {
    return;
  }
  if (driver_.Enable())
  {
    // 升降/手动位移途中重新使能，视为人工中断，转故障保持。
    if (state_ == State::Lifting || state_ == State::Descending ||
        state_ == State::ManualMoving)
    {
      faults_ |= OperatorHold;
      Transition(State::FaultHold, uint32_t(previous_tick_us_ / 1000));
    }
  }
}

/**
 * @brief 刷新调试窗口
 * @note 只写只读类字段，不动 catch_debug.config / command，避免覆盖操作员正在填的内容。
 */
void CatchBlock::Publish()
{
  Critical guard;
  catch_debug.actual_height_m = driver_.height_m;
  catch_debug.faults = faults_ | driver_.FaultCode();
  catch_debug.state = static_cast<uint8_t>(state_);
  catch_debug.feedback_valid = driver_.feedback_valid ? 1 : 0;
  catch_debug.motor_enabled = driver_.Enabled() ? 1 : 0;
  catch_debug.pump_on = driver_.PumpPinLevel() ? 1 : 0;
  catch_debug.valve_closed = driver_.ValvePinLevel() ? 1 : 0;

  catch_debug.target_position_rad = driver_.target_position_rad;
  catch_debug.measured_position_rad = driver_.measured_position_rad;
  catch_debug.reference_position_rad = driver_.reference_position_rad;
  catch_debug.reference_velocity_rad_s = driver_.reference_velocity_rad_s;
  catch_debug.eso_position_rad = driver_.eso_position_rad;
  catch_debug.eso_velocity_rad_s = driver_.eso_velocity_rad_s;
  catch_debug.disturbance_rad_s2 = driver_.disturbance_rad_s2;
  catch_debug.raw_current_A = driver_.raw_current_A;
  catch_debug.limited_current_A = driver_.limited_current_A;
  catch_debug.final_current_A = driver_.final_current_A;
  catch_debug.accepted_command_A = driver_.accepted_command_A;
  catch_debug.feedback_current_raw = driver_.feedback_current_raw;
  catch_debug.feedback_current_A_valid = driver_.feedback_current_valid ? 1 : 0;
  catch_debug.temperature_C = driver_.temperature_C;
}
/** @} */
