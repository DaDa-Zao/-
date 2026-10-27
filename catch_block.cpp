#include "catch_block.hpp"
#include "keys.hpp"
#include <cmath>

// 全局唯一的调试窗口实例。上电全 0：气泵不吸、夹爪松开、电机不输出、没有待执行命令。
CatchBlockDebug catch_debug;

namespace
{
/**
 * @brief 关中断的短临界区
 * @note 退出时恢复进入前的掩码，不做成"无条件开中断"。
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

// 时间基准不在这里做了：单调微秒由 CatchBlockDriver 提供（见 AdvanceClock 的注释），
// 因为帧年龄要它、周期检查也要它，两份时钟迟早对不上。这里只留一个 Publish/Enqueue
// 用的短临界区。
}

/** @name 命令入口 */
// 这 12 个方法都只做一件事：换个命令码塞进邮箱就返回。
// 判断状态、动硬件全在下一拍 Update() → Process() 里，所以这里看不到任何逻辑。
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

bool CatchBlock::LiftToTop()
{
  return Enqueue(Command::LiftToTop);
}

bool CatchBlock::DescendToOrigin()
{
  return Enqueue(Command::DescendToOrigin);
}

bool CatchBlock::HoldHere()
{
  return Enqueue(Command::HoldHere);
}
/** @} */

/** @name 单步动作 */
/** @{ */
/**
 * @brief 两路继电器的单步动作
 * @note 只改标志位，真正写引脚在 ApplyManualOverrides()，所以最多晚一拍（≤5 ms）。
 */
void CatchBlock::SetPump(bool on)
{
  catch_pump_on = on ? 1 : 0;
}

void CatchBlock::SetValve(bool on)
{
  catch_valve_closed = on ? 1 : 0;
}
/** @} */

/** @name 邮箱 */
/** @{ */
// 三个重载是同一个邮箱的三种载荷。关键约定：命令码最后写，
// 保证取命令时载荷一定已经就绪；取命令的一方先拷走载荷再清标志。
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
 * @brief 从邮箱取一条命令交给 Process()
 * @note 临界区里只做"拷走载荷 + 清标志"，Process() 放到临界区外，避免长时间关中断。
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
 * @brief 从邮箱取一条 Keil 窗口写的命令
 * @note 先把命令号清 0 保证只生效一次，再走与公开命令完全相同的 Process()。
 *       7~10 是继电器直出，不经过 Process，所以这里要提前把上一条的拒绝原因清掉。
 */
void CatchBlock::ConsumeDebugCommand(uint32_t now_ms)
{
  const uint8_t code = catch_debug.command;
  if (code == 0)
  {
    return;
  }
  catch_debug.command = 0;
  last_reject_ = ReasonNone; // 7~10 不走 Process，不先清就会读到上一条的原因

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
    case 7: // 单步：气泵开（PE12 高），硬件直出，不看状态与配置
      SetPump(true);
      catch_debug.result = 1;
      return;
    case 8: // 单步：气泵关（松开吸附）
      SetPump(false);
      catch_debug.result = 1;
      return;
    case 9: // 单步：电磁阀开（PE6 高 = 夹爪闭合）
      SetValve(true);
      catch_debug.result = 1;
      return;
    case 10: // 单步：电磁阀关（夹爪松开）
      SetValve(false);
      catch_debug.result = 1;
      return;
    case 11: // 单步：抬升到行程顶端（需已配置 + 已标零 + 已使能）
      command = Command::LiftToTop;
      break;
    case 12: // 单步：下降到原点（需已配置 + 已标零 + 已使能）
      command = Command::DescendToOrigin;
      break;
    case 13: // 自动流程：原点稳定后进入夹取（此时应已开阀、开泵）
      command = Command::CloseGrip;
      break;
    case 14: // 自动流程：人工确认夹牢，上升到行程顶端
      command = Command::ConfirmGripAndLift;
      break;
    case 15: // 自动流程：人工许可投放，下一拍复核后进入 Release
      command = Command::PermitDrop;
      break;
    case 16: // 自动流程：人工确认物块已离开，受控下降回原点
      command = Command::ConfirmObjectClearAndReturn;
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
/**
 * @brief 框架启动时调一次，重复注册不会重置正在跑的状态
 * @note PE6/PE12 的上电电平由 CubeMX 初始化，本应用在这里不写引脚。
 */
void CatchBlock::Start()
{
  if (started_)
  {
    return;
  }
  started_ = true;
  PanelKeys::GetInstance().Init(); // 面板按键：只在上电配一次引脚
  Publish();
}

/**
 * @brief 框架 App 任务入口（5 ms 一拍，200 Hz）
 * @note 顺序不能乱：先撤力与采样，再查故障，最后收命令、推状态、写输出。
 *       采样必须早于 CheckMotion，否则检查用的是上一拍的数据。
 */
void CatchBlock::Update()
{
  if (!started_)
  {
    return;
  }

  // 面板按键先扫：它可能改 catch_motor_enable，必须早于下面那行"立即撤力"的判断，
  // 否则按下失能键要等下一拍才生效。它也可能写 catch_debug.command，
  // 走的是和 Keil 手写完全相同的 ConsumeDebugCommand() 路径。
  PanelKeys::GetInstance().Update();

  // 使能标志为 0 时立即撤力，不等状态机。
  if (!driver_.MotorEnableRequested() && driver_.Enabled())
  {
    driver_.Disable();
  }

  // 采样顺带采集本拍的时间戳，之后所有判断都引用同一个数，不再各自读表。
  // 时间基准是驱动层维护的单调微秒（CYCCNT 差值累加）：软件时间轴那套
  // 秒/毫秒/微秒分开写、会被 CAN 接收中断拼坏的问题在这里不存在。
  driver_.Sample();
  const uint64_t now_us = driver_.SampledUs();
  const uint32_t now_ms = uint32_t(now_us / 1000);
  // 首拍没有上一拍时间，用 5 ms 占位；只影响 2~15 ms 的窗口判断，不影响控制。
  const uint64_t elapsed_us = previous_tick_us_ ? now_us - previous_tick_us_ : 5000;

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
 * @brief 主分发：拿到一条命令，决定转状态还是记一次拒绝
 * @note 调用者只有两个：ConsumeRequest()（业务代码下的命令）和 ConsumeDebugCommand()（Keil 写的命令）。
 *       每个 case 自己判断"当前状态 + 物理条件"，不满足就 break 落到末尾统一计一次拒绝。
 *       人工确认已经写进命令名里了，这里不再逐条检查确认位。
 */
void CatchBlock::Process(Command command, const Config &config, float height_m, uint32_t now_ms)
{
  last_reject_ = ReasonNone; // 每条命令重新记原因，避免读到上一条的
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
    case Command::MoveToHeight:    // 调试位移：目标由操作员给
    case Command::LiftToTop:       // 抬升：目标取行程顶端
    case Command::DescendToOrigin: // 下降：目标取原点
      // 三条命令共用同一条运动路径，只有目标来源不同；顶端/原点在此取值，保证用的是生效参数。
      if (command == Command::LiftToTop)
      {
        height_m = driver_.TravelMetres();
      }
      else if (command == Command::DescendToOrigin)
      {
        height_m = 0;
      }
      if (state_ != State::OriginHold && state_ != State::ManualHold)
      {
        Reject(ReasonState); // 只能从原点保持或手动保持出发
        break;
      }
      if (!driver_.Enabled() || !driver_.Healthy())
      {
        Reject(ReasonNotReady); // 最常见：catch_motor_enable 还没置 1
        break;
      }
      if (!std::isfinite(height_m) || height_m < 0 || height_m > driver_.TravelMetres())
      {
        Reject(ReasonTarget); // 含 NaN / Inf，以及超出 0..travel_m
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
    case Command::ResetWhileSupported: // 撤力 + 清锁存 + 退回未配置
      // 注意这条路径**不是**清锁存的唯一途径：撤力状态下 command = 1 也会经
      // driver_.Configure() 里那行 faults_ = 0 清掉本层锁存，而且直接落到 Idle。
      // 它唯一不可替代的作用是"带着使能也能退"——FaultHold 下电机还在托着，
      // ApplyConfigure() 一进门就被 `driver_.Enabled()` 拒掉，只能先从这里撤力。
      driver_.Disable();
      driver_.Neutral();
      driver_.ResetFault();
      driver_.ResetConfiguration();
      protection_ack_ = false;
      faults_ = 0;
      Transition(State::Unconfigured, now_ms); // 必须重新配置、标零、使能。
      return;
    default:
      break;
  }
  // 走到这里说明没有一条 case 提前 return，本次命令被拒。
  // 具体原因由各 case 自己写；没写的一律归为"当前状态不接受"。
  if (last_reject_ == ReasonNone)
  {
    last_reject_ = ReasonState;
  }
  ++rejected_; // 入队成功不等于转换获准，这里统计被拒次数。
}

/**
 * @brief 记一次拒绝原因
 * @note 只记录，不计数；计数统一在 Process() 末尾，避免"先记原因再 break"的路径被算两次。
 */
void CatchBlock::Reject(uint8_t reason)
{
  last_reject_ = reason;
}

/**
 * @brief 配置：先查三类参数，再让驱动初始化
 * @note 前置条件：电机已停、阈值与硬件参数都合法。配好后必须重新标零。
 */
bool CatchBlock::ApplyConfigure(const Config &config, uint32_t now_ms)
{
  // 三类拒绝原因分开报：它们对应的处理动作完全不同，共用一个 result=2 排查起来很费劲。
  if (driver_.Enabled())
  {
    Reject(ReasonEnabled); // 反过来：这里是"还在输出"，要先撤力，不是"没使能"
    return false;
  }
  if (!config.Validate())
  {
    Reject(ReasonThreshold);
    return false;
  }
  if (!config.ValidateHardware())
  {
    Reject(ReasonHardware);
    return false;
  }

  const CatchBlockDriver::ConfigResult result = driver_.Configure(config);
  if (result == CatchBlockDriver::ConfigResult::Rejected)
  {
    // 驱动侧拒绝：换 ESC 编号、GPIO 无效、换算系数为 0 或 Inf。
    Reject(ReasonHardware);
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
 * @brief 在静止的原点采零点、设软限位
 * @note 标零本身不会让机构动，动不动由 catch_motor_enable 决定。
 */
bool CatchBlock::DoArmAtOrigin(uint32_t now_ms)
{
  if (state_ != State::Idle)
  {
    Reject(ReasonState); // 必须先配置成功（state=Idle）才能标零
    return false;
  }
  const CatchBlockDriver::ArmResult arm = driver_.ArmAtCurrentPosition();
  if (arm != CatchBlockDriver::ArmResult::Ok)
  {
    // 三种失败分开报，调参时才能分清是 CAN 没通、状态不对，还是机构在晃。
    Reject(arm == CatchBlockDriver::ArmResult::Unhealthy ? ReasonNotReady :
        arm == CatchBlockDriver::ArmResult::Moving ? ReasonMoving : ReasonState);
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
 * @brief 换状态，并把本阶段计时清零
 * @note 顺手清掉"到位计时"，免得沿用上一阶段的判断。
 */
void CatchBlock::Transition(State state, uint32_t now_ms)
{
  state_ = state;
  phase_start_ms_ = now_ms;
  settled_since_us_ = 0;
}

/**
 * @brief 记故障，并决定"托住"还是"撤驱动"
 * @note can_hold=true 且驱动还能闭环 → FaultHold（锁在当前位置）；否则 Disable → Fault。
 *       两者都不保证托住负载。"能不能保持"是安全策略，只能在本层决定，不能下放给驱动层。
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
 * @brief 自动流程往前走一格，然后查阶段超时
 * @note 每拍最多前进一格；超时检查放在最后，避免刚转换就被判超时。
 */
void CatchBlock::Advance(uint64_t now_us, uint32_t now_ms)
{
  // 配好了却一帧反馈都没收到，说明 CAN 没通。2 s 后直接报故障：
  // 否则状态机会一直停在 Idle，command = 2 只回一个"未就绪"，看不出根因。
  if (state_ == State::Idle && !driver_.FeedbackSeen() && now_ms - phase_start_ms_ > 2000)
  {
    Fail(Feedback, false, now_ms);
    return;
  }

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
 * @brief 这个状态能不能让电机出力（白名单）
 * @note 只有 Unconfigured / Idle / FaultHold / Fault 四个状态不在名单里。
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
 * @brief 到位了没有：使能中 + 位置误差够小 + 速度够小，并且持续 settle_ms
 * @note 三个条件任一不满足就清零重新计时，所以"抖一下"会重新开始算。
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
 * @brief 下发高度目标，失败就当 Travel 故障处理
 * @note 失败一律记 Travel，不区分"参数没配好"和"真的越界"。
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
 * @brief 使能期间每拍体检
 * @note 查四件事：应用周期、上次 CAN 发送、控制器/反馈健康、高度越界。任一项异常就升级为故障。
 *       没使能时直接返回，因为没输出就没有这些风险。
 */
void CatchBlock::CheckMotion(uint64_t now_us, uint32_t now_ms, uint64_t elapsed_us)
{
  if (!driver_.Enabled())
  {
    return;
  }
  // 应用周期异常（名义 5 ms，允许 2~15 ms）；注意这里量的是 App 任务，不是控制环。
  // 在 Keil 里下断点就会超过 15 ms，所以使能状态下单步调试必掉故障，见 README。
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
 * @brief 写输出：继电器交给驱动层，本层只决定"这个状态该不该使能"
 * @note 已经使能就不重复使能；状态不在白名单、控制器不健康、零点无效，都保持现状。
 */
void CatchBlock::ApplyOutputs()
{
  // 驱动层已完成两路继电器写入，并在 catch_motor_enable 为 0 时撤力。
  if (!driver_.ApplyManualOverrides())
  {
    blockers_ = BlockerMotorSwitch;
    return;
  }

  if (driver_.Enabled())
  {
    blockers_ = BlockerNone; // 已经在出力，不算被挡
    return;
  }

  // 按"因果顺序"取第一条不满足的，所以永远只有一个答案，不用做位运算：
  //   开关 → 配置 → 硬件（CAN/反馈/ADRC）→ 标零 → 状态
  // 健康位放在标零前面，因为标零本身依赖反馈；CAN 没通时应该报 64，而不是报 4。
  if (!driver_.Ready())
  {
    blockers_ = BlockerNotConfigured;
  }
  else
  {
    const uint32_t health = driver_.HealthTerms();
    if (health != 0)
    {
      blockers_ = uint16_t(health << 4); // HealthBit × 16
    }
    else if (!driver_.OriginValid())
    {
      blockers_ = BlockerNoOrigin;
    }
    else if (!StateAllowsEnable())
    {
      blockers_ = BlockerState;
    }
    else
    {
      blockers_ = BlockerNone;
    }
  }
  if (blockers_ != BlockerNone)
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
 * @brief 把驱动层的量搬进 catch_debug
 * @note 只写只读类字段，不动 catch_debug.config / command，避免覆盖操作员正在填的内容。
 */
void CatchBlock::Publish()
{
  Critical guard;
  catch_debug.actual_height_m = driver_.height_m;
  catch_debug.faults = faults_ | driver_.FaultCode();
  catch_debug.motor_faults = driver_.FaultCode();
  catch_debug.feedback_hz = driver_.feedback_hz;
  catch_debug.state = static_cast<uint8_t>(state_);
  catch_debug.reason = last_reject_;
  catch_debug.feedback_valid = driver_.feedback_valid ? 1 : 0;
  catch_debug.feedback_age_ms = uint16_t(driver_.feedback_age_ms > 60000 ? 60000 : driver_.feedback_age_ms);
  catch_debug.feedback_age_peak_ms = uint16_t(driver_.feedback_age_peak_ms > 60000 ? 60000 : driver_.feedback_age_peak_ms);
  catch_debug.motor_enabled = driver_.Enabled() ? 1 : 0;
  catch_debug.pump_on = driver_.PumpPinLevel() ? 1 : 0;
  catch_debug.valve_closed = driver_.ValvePinLevel() ? 1 : 0;
  // 面板按键的两级证据：keys_raw 变了说明引脚识别到了，keys_last 变了说明动作真执行了。
  catch_debug.keys_raw = PanelKeys::GetInstance().RawBitmap();
  catch_debug.keys_last = PanelKeys::GetInstance().LastKey();
  catch_debug.blockers = blockers_;

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
