#include "catch_block_driver.hpp"
#include "bsp_hardware.hpp"
#include "bsp_dwt.hpp"
#include <cmath>

// 三个硬件开关的定义（声明在头文件）。上电全为 0：气泵不吸、夹爪松开、电机不输出。
extern "C"
{
volatile uint8_t catch_pump_on = 0;
volatile uint8_t catch_valve_closed = 0;
volatile uint8_t catch_motor_enable = 0;
}

namespace
{
/// @brief 编码器 1 个计数对应多少弧度：2π/8192 ≈ 7.67e-4
constexpr float kRadPerCount = 6.28318530718f / 8192.0f;
/// @brief 1 弧度对应多少编码器计数，与上面互为倒数。下发目标时用
constexpr float kCountsPerRad = 8192.0f / 6.28318530718f;
/// @brief C620 的安培满量程。库在 Init(DJI_C620) 里把 _max_amp 定成 20 A，
///        _code_per_amp 因此是 16384/20。_code_per_amp 是 private，只能在这里复算一遍。
constexpr float kC620FullScaleA = 20.0f;
/// @brief 每安培多少编码值（C620）。把库里 code 域的目标值换回安培时用
constexpr float kCodePerAmp = MotorDJI_Driver::MAX_CURRENT_CODE_C620 / kC620FullScaleA;

/**
 * @brief 库的 offset 初始化窗口有多长
 * @note  MotorDji_RxCallback 里是 `msg_cnt++ <= 50` 才走 _MotorDJI_DecodeInitOffset()，
 *        那一支只写 angle / offset_angle，round_cnt 和 total_angle 还停在初值。
 *        所以 msg_cnt 到 52 才算"累计角开始有效"，这之前的角度和高度都不能当数用。
 */
constexpr uint32_t kOffsetInitFrames = 51u;

/// @brief 超速要连判几拍才锁存。4 ms 一拍，3 拍 = 12 ms，滤掉单拍毛刺
constexpr uint32_t kOverspeedSamples = 3u;

/**
 * @brief 是不是有限的正数。
 * @note 换算系数可能因为参数写错变成 0 或 Inf，这里单独挡一次。
 */
bool Positive(float x)
{
  return std::isfinite(x) && x > 0;
}

/// @brief 相对比较，用来判断"库里抄进来的值和我配的是不是同一个"
bool Near(float a, float b)
{
  return std::fabs(a - b) <= 1e-6f * std::fmax(1.0f, std::fabs(a) + std::fabs(b));
}

/// @brief 1 µs 等于多少个 CYCCNT 计数（A 板 180 MHz → 180）
uint32_t TicksPerUs()
{
  const uint32_t ticks = SystemCoreClock / 1000000u;
  return ticks ? ticks : 180u; // SystemCoreClock 异常时按 A 板额定值兜底
}

float ClampAbs(float x, float limit)
{
  if (x > limit) return limit;
  if (x < -limit) return -limit;
  return x;
}
}

/**
 * @brief 单调微秒时钟
 * @note  只累加 CYCCNT 差值：32 位无符号减法天然处理回绕，圈数由 64 位累加器补回来。
 *        不用 bsp_dwt 的 DWT_GetTimeline_USec()：那个函数把秒 / 毫秒 / 微秒三个字段分开写，
 *        中间隔着三次 64 位除法且没有临界区。CAN 接收中断（1 kHz）在这期间插进来，
 *        就会把"新的秒 + 旧的毫秒"拼在一起，算出 ±1 s 的假值；上层 2~15 ms 的周期检查
 *        会因此误判、直接撤力。这里没有那个窗口。
 *        调用者只有 App 任务一个，本来就没有并发；外面套临界区是为了防御将来的误用。
 */
uint64_t CatchBlockDriver::AdvanceClock()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint32_t cnt = DWT->CYCCNT;
  clock_acc_cnt_ += uint64_t(uint32_t(cnt - clock_last_cnt_)); // 首次调用等于 DWT_Init 至今的计数
  clock_last_cnt_ = cnt;
  const uint64_t us = clock_acc_cnt_ / uint64_t(TicksPerUs());
  __set_PRIMASK(primask);
  return us;
}

uint64_t CatchBlockDriver::SampledUs() const
{
  return sample_us_;
}

/**
 * @brief 距上一帧反馈过了多少微秒
 * @note  判"超时"必须用微秒，不能用 `feedback_age_ms`：那个是给人看的整数毫秒，
 *        `age_us / 1000` 会把 10.9 ms 截成 10，于是 10 ms 的超时窗口实际放宽到近 11 ms，
 *        比回退前（库内直接比 `now_us - timestamp_us`）松了将近 10%。
 *        这里用同一只单调时钟的原始差值，判据和当年一模一样。
 */
uint32_t CatchBlockDriver::FeedbackAgeUs() const
{
  if (motor_.driver.measure.msg_cnt == 0 || last_frame_us_ == 0)
  {
    return 60000000u; // 一帧都没收到过
  }
  const uint64_t age_us = sample_us_ - last_frame_us_; // 同一只单调时钟，不会倒流
  return age_us > 60000000ull ? 60000000u : uint32_t(age_us);
}

/**
 * @brief 按参数把 CAN、控制器、软行程配好。
 * @note 失败的两种情况都不改 configured_，靠返回值区分"拒绝"和"硬件故障"。
 */
CatchBlockDriver::ConfigResult CatchBlockDriver::Configure(const CatchBlockConfig &config)
{
  if (motor_.driver.enabled || !config.ValidateHardware() ||
      !pump_.IsValid() || !valve_.IsValid())
  {
    return ConfigResult::Rejected;
  }

  // 换电调编号要重新上电，不能在运行中换。
  // 回退后的 MotorDJI_Driver::Init() 是"同一对象换 slot 就 return"，而且
  // can_inst.can_id 还留着上一次注册成功的旧句柄 —— 也就是说换 ID 失败**查不出来**。
  // 若放过去，本层会以为配成了，实际收发还挂在旧 ID 上：命令发到旧槽位、反馈从旧报文来，
  // 两个都在骗人。宁可在这里拒掉，让上层报 reason = 2。
  // registered_esc_id_ 和 configured_ 分开记，因为 command = 5 只清后者，库的槽位不会退。
  if (registered_esc_id_ != 0 && config.esc_id != registered_esc_id_)
  {
    return ConfigResult::Rejected;
  }

  // 回退后的库没有 CurrentPolarity() 这种接口，电流方向只能由接线决定：
  // 给正电流转子往哪个方向转，是硬件属性，软件改不了。
  // 若实测与编码器方向相反，必须调换电机任意两根相线 —— 在软件里翻符号等于改库，不允许。
  if (config.current_encoder_sign != 1)
  {
    return ConfigResult::Rejected;
  }

  // 高度换算系数：带轮半径 × 齿轮增速 ÷ 减速比（m/rad，转子侧）。全工程只算这一处。
  const float metres_per_rad = config.pulley_radius_m * config.external_speed_ratio / config.gearbox_ratio;
  if (!Positive(metres_per_rad))
  {
    return ConfigResult::Rejected;
  }

  // 速度和加速度必须是正的，而且这一条必须在驱动层再挡一次，不能只靠 Validate()。
  // 原因在库的 NonlinearTD 里：fclamp(val, limit) 遇到 limit <= 0 时**直接返回 val**，
  // 也就是 max_v = 0 不是"限到 0"，而是"完全不限速" —— TD 会用任意速度冲向目标。
  // 而 r = 0 的后果相反但同样糟：fhan 恒返回 0，v1/v2 永不更新，目标改了机构也不动，
  // 而且没有任何状态位提示。两种都是填 0 造成的，这里一次挡掉。
  if (!Positive(config.speed_m_s) || !Positive(config.acceleration_m_s2))
  {
    return ConfigResult::Rejected;
  }

  config_ = config;
  metres_per_rotor_rad_ = metres_per_rad;
  motor_.Init(Hardware::hcan_main, config_.esc_id, DJI_C620);

  // 回退后的 Init() 既没有返回值也没有 Initialized()。注册成没成只剩一个痕迹：
  // BSP::CAN::Regist() 会把 can 写进 can_inst.can_id；ID 非法或槽位被别的实例占了，
  // Init() 会提前 return，can_id 就还是空。查不到它就等于"这个电机根本没登记进接收表"，
  // 之后再怎么配都不会有反馈 —— 必须现在就报硬件故障，别让它拖到 2 s 后变成 reasons=5。
  if (motor_.driver.can_inst.can_id == nullptr)
  {
    return ConfigResult::HardwareFault;
  }
  // 到这里才算真的在库里占到槽位了。之后这个对象的编号就钉死，换 ID 只能重新上电。
  registered_esc_id_ = config_.esc_id;

  // 一次完整重配就是一次干净开始：清本层账本，别把上一次的锁存带进来。
  faults_ = 0;
  overspeed_run_ = 0;
  feedback_age_peak_ms = 0;
  last_msg_cnt_ = motor_.driver.measure.msg_cnt; // 别把"重配"当成"刚来了一帧"

  const float rad_s = config_.speed_m_s / metres_per_rotor_rad_; // 规划限速 (rad/s)
  // 超速门限换算成转子转速：同一个数既当规划速度的快照，也当本层超速保护的门槛。
  // 注意它是"转子 rpm"，即减速箱之前、电调反馈的语义，2π/60 的换算只做一次。
  guard_rpm_ = config_.overspeed_m_s / metres_per_rotor_rad_ * 60.0f / 6.28318530718f;

  // 链式配置：单位在这里换算成 SI，驱动内部只在发包前做一次 A→code。
  //
  // 与回退前的差别（这四项库不再提供，不要去找）：
  //   CurrentSlew()      —— 爬坡率在库里写死 1098.63 A/s，是 private 且没有 setter。
  //                         config.current_slew_A_s 因此**当前不生效**，只作为期望值留档。
  //   CurrentPolarity()  —— 见上面的接线要求。
  //   FeedbackTimeout()  —— 本层用 config.feedback_timeout_us 直接判，不进库。
  //   MotionLimits()     —— 速度上限仍由 ADRC_MaxPlannedVel() 下发；加速度上限库里
  //                         从 (Kt × max_current - B) / J 自行推导，没有单独入口。
  motor_.SetDt(0.001f)
      .ConfigADRC()
      .AsPosC()
      .ADRC_Womega(config_.observer_Hz, config_.controller_Hz)
      .ADRC_Physic(config_.inertia_kg_m2, config_.torque_constant_Nm_A, config_.viscous_Nm_s_rad)
      .ADRC_Limit(config_.max_current_A)
      .ADRC_MaxPlannedVel(rad_s * 60.0f / 6.28318530718f)
      .Apply();

  // Apply() 是 void，也没有 IsConfigured() 可查。它内部有四五处 BspLog_LogError 之后直接 return，
  // 一旦从那里返回，库的 Control() 会永远返回 0 —— 电机不动，却没有任何状态位能看出来。
  // 所以这里必须自己验一遍：验不过就当硬件故障报上去，宁可 state 变 14 也不要不声不响。
  if (!LibraryConfigured())
  {
    return ConfigResult::HardwareFault;
  }

  // ── 把 MotionLimits() 的加速度上限补回来 ──────────────────────────────────
  // 回退后的 ADRC 没有 SetMotionLimits()，但 NonlinearTD 的 r / max_v 是 public，
  // 而当年那个 SetMotionLimits() 内部也只做一件事：
  //     input_nltd_3rd.max_v = 最大角速度；input_nltd_3rd.r = 最大角加速度；
  // 所以这里用同样的公开成员原样复刻。
  //
  // 为什么必须补：Apply() → ADRC::Init() 会把 r 算成 (Kt×max_current − B)/J × 0.67，
  // 按现在的参数是 **5583 rad/s²**；而配置要的是 0.04 m/s² ÷ 1.3019 mm/rad ≈ **30.7 rad/s²**，
  // 差 180 倍。r 是 TD 的最大加速度，r 一大 TD 就退化成阶跃发生器：
  // 目标一变，参考位置瞬间跳到终点，位置环立刻打满电流 —— 机构是被"砸"出去的，不是走过去的。
  // 只覆盖 r 就够：速度上限已经由 ADRC_MaxPlannedVel() 设成同一个 rad_s 了，这里一起写明更保险。
  //
  // ⚠ 这行和库里的 ADRC::ApplyJ() 是冲突的：ApplyJ() 在 Thr_Ord 下会调
  //    input_nltd_3rd.ResetR(r_maxacc × 0.67)，也就是把 r 又算回 5583 那一档。
  //    现在安全，只因为 ADRC::UpdateJ() 在整个工程里**没有任何调用点**，
  //    _j_update_pending 永远是 false，StepPendingJUpdate() 一进门就 return。
  //    将来谁要用惯量在线更新（UpdateJ），必须先解决这个冲突，否则机构会在
  //    某次 J 更新之后突然从"走过去"变成"砸过去" —— 而且没有任何状态位会提示。
  motor_.motor_adrc.input_nltd_3rd.r = config_.acceleration_m_s2 / metres_per_rotor_rad_;
  motor_.motor_adrc.input_nltd_3rd.max_v = rad_s;

  configured_ = true;
  origin_valid_ = false;
  return ConfigResult::Applied;
}

/**
 * @brief Apply() 到底成没成
 * @note  能查的只有 ADRC 的公开量：ADRC::Init() 会把 J / Kt / max_current 从配置里原样抄进来，
 *        mode 由 AsPosC() 置成 PosC，use_adrc 由 ConfigADRC() 置位。
 *        这几个值都对上，才说明配置链真的走完到 Init 之后。
 */
bool CatchBlockDriver::LibraryConfigured() const
{
  const ADRC &adrc = motor_.motor_adrc;
  const float amp = (config_.max_current_A < kC620FullScaleA) ? config_.max_current_A : kC620FullScaleA;
  return motor_.use_adrc && motor_.mode == PosC && Near(adrc.J, config_.inertia_kg_m2) &&
      Near(adrc.Kt, config_.torque_constant_Nm_A) && Near(adrc.max_current, amp);
}

/**
 * @brief ADRC 的 Init 跑过没有（供健康位用）
 * @note  库把 _inited 藏成了 private，没有 IsValid()。能查的还是那三个公开量。
 */
bool CatchBlockDriver::AdrcConfigured() const
{
  return motor_.motor_adrc.J > 0.0f && motor_.motor_adrc.Kt > 0.0f &&
      motor_.motor_adrc.max_current > 0.0f;
}

/// @brief 置锁存位，只增不减。唯一的清除路径是 ResetFault()
void CatchBlockDriver::Latch(uint32_t fault)
{
  faults_ |= fault;
}

/**
 * @brief 把现在的位置记成 0 m，并让控制器就地接管。
 * @note  软行程不在这里设：SetHeightTarget() 自己会挡 0..travel_m，不存在第二个限位来源。
 *        三种失败分开返回，状态机据此报不同的 reason，调参时才分得清是 CAN 没通还是机构在晃。
 */
CatchBlockDriver::ArmResult CatchBlockDriver::ArmAtCurrentPosition()
{
  if (!configured_)
  {
    return ArmResult::NotConfigured;
  }
  // 不健康时不再往下算：速度和角度都不可信，采到的零点会是错的。
  if (HealthTerms() != 0)
  {
    return ArmResult::Unhealthy;
  }

  const moto_measure_t &measure = motor_.driver.measure;
  // 运动中标零会污染零点，要求高度方向速度低于到位门限。
  const float measured_speed = std::fabs(float(measure.speed_rpm)) *
      6.28318530718f / 60.0f * metres_per_rotor_rad_;
  if (measured_speed > config_.settled_speed_m_s)
  {
    return ArmResult::Moving;
  }

  origin_rad_ = float(measure.total_angle) * kRadPerCount;
  origin_valid_ = true;
  // 以当前位置为保持目标，并把 TD/ESO 就地重置：落地那一刻误差和控制量都从 0 起算，不会跳。
  motor_.Uneutral(PosC);
  return ArmResult::Ok;
}

/**
 * @brief 丢掉"已配置"和零点标记，回到不能动的状态
 * @note 不清故障码，那个由 ResetFault() 单独负责。
 *       也**不清** registered_esc_id_：库里那次槽位注册撤不回来，这个编号要一直跟着对象，
 *       才能在下一次 Configure() 里挡住"换 ID"这种做不到的请求。
 */
void CatchBlockDriver::ResetConfiguration()
{
  configured_ = false;
  origin_valid_ = false;
}

/**
 * @brief 把编码器、电流、温度、帧年龄读进公有成员，并当场判超速。
 * @note  高度和速度要"已配置 + 已标零 + 反馈有效"三条都成立才更新，否则保留旧值。
 *        控制器的诊断量无条件刷新 —— 撤力状态下也要能看 ESO 在估什么。
 */
void CatchBlockDriver::Sample()
{
  const uint64_t now_us = AdvanceClock();
  sample_us_ = now_us;

  moto_measure_t &measure = motor_.driver.measure;

  // ── 帧年龄 ──────────────────────────────────────────────────────────────
  // 库只给了一个只增不减的 msg_cnt，没有任何时间戳（回退前那套 FeedbackSnapshot /
  // FeedbackFresh 已经不存在了）。所以"这一拍有没有新帧"用计数边沿判，
  // "距上一帧多久"用本层自己的单调微秒算。
  const uint32_t frames = measure.msg_cnt;
  if (frames != last_msg_cnt_)
  {
    last_msg_cnt_ = frames;
    last_frame_us_ = now_us;
    feedback_age_ms = 0;
  }
  else if (frames == 0)
  {
    feedback_age_ms = 60000; // 一帧都没收到过，别拿"开机多久"冒充帧间隔
  }
  else
  {
    const uint64_t age_us = now_us - last_frame_us_;
    feedback_age_ms = age_us > 60000000ull ? 60000u : uint32_t(age_us / 1000ull);
  }
  if (feedback_age_ms > feedback_age_peak_ms)
  {
    feedback_age_peak_ms = feedback_age_ms;
  }

  // 库自己统计的帧率。正常 C620 是 1 kHz；掉到几百说明总线丢帧或被别的报文挤占。
  // 它只在收帧时被刷新，断线后会一直停在最后一个好值，所以超过 1 s 没有新帧就直接报 0 ——
  // 这个数唯一的用途就是回答"总线还活着吗"，报一个历史值等于骗人。
  const float hz = motor_.driver._recv_quality._recv_freq;
  feedback_hz = (feedback_age_ms <= 1000u && hz > 0.0f)
      ? uint16_t((hz > 65535.0f) ? 65535.0f : hz + 0.5f)
      : 0;

  const bool decoded = frames > kOffsetInitFrames;
  feedback_valid = decoded && FeedbackAgeUs() <= config_.feedback_timeout_us;
  feedback_current_valid = decoded;
  feedback_current_raw = measure.raw_current;
  temperature_C = measure.temprature;

  // ── 数值与坏帧 ──────────────────────────────────────────────────────────
  // 回退前的 Control() 在这两处都有守卫，回退后全都消失了，只能在本层补。
  //
  // ① 命令链出现 NaN / Inf：ADRC 算出 NaN 会一路走到 `(int16_t)targ_current`，
  //    转成什么值由编译器决定，等于往电调里发随机电流。targ_current / targ_pos_rad 都是公开量，
  //    直接查。只能晚一拍（4 ms）发现，但发现就撤力，比放任它发下去强得多。
  // ② 编码器角度码必须是 13 位（0..8191）。超了说明这一帧根本不是合法电调反馈，
  //    而它会直接污染库里的 round_cnt / total_angle —— 累计角一旦跳，零点和行程全废。
  if (!std::isfinite(motor_.targ_current) || !std::isfinite(motor_.targ_pos_rad))
  {
    Latch(DFaultNumeric);
  }
  if (measure.angle > 8191u)
  {
    Latch(DFaultFeedback);
  }

  // ── 超速锁存 ────────────────────────────────────────────────────────────
  // 只在"真的在出力"时判：撤力状态下手拉判方向、掉电下沉都不算驱动故障，
  // 否则调一次方向就锁一次，又回到"只能整车断电"的老路。
  // 连判 3 拍（12 ms）才锁，滤掉单拍毛刺。
  //
  // 这是回退前帧级守卫（ConfigureFeedbackGuard）唯一被保留下来的功能。原来那份实现
  // 没有"是否使能"的前置条件，判中一次就冻结帧流、时间戳不再更新，
  // 于是每一拍都判"反馈过期"、使能永远发不出去 —— 那正是台架上使不上力的直接原因。
  // 这里改成：只看使能期间、可恢复、锁存后靠 command = 5 清。
  if (motor_.driver.enabled && guard_rpm_ > 0.0f && std::fabs(float(measure.speed_rpm)) > guard_rpm_)
  {
    if (++overspeed_run_ >= kOverspeedSamples)
    {
      Latch(DFaultOverspeed);
    }
  }
  else
  {
    overspeed_run_ = 0;
  }

  // ── 控制器诊断 ──────────────────────────────────────────────────────────
  // 全部读库的公开成员，没有一个私有字段被碰过。
  rotor_rad = float(measure.total_angle) * kRadPerCount;
  target_position_rad = motor_.targ_pos_rad;
  measured_position_rad = rotor_rad;
  reference_position_rad = motor_.motor_adrc.input_nltd_3rd.v1;
  reference_velocity_rad_s = motor_.motor_adrc.input_nltd_3rd.v2;
  eso_position_rad = motor_.motor_adrc.eso.z1;
  eso_velocity_rad_s = motor_.motor_adrc.eso.z2;
  disturbance_rad_s2 = motor_.motor_adrc.eso.z3;
  // 库在 ADRC::CalcPos() 里就把 i_des 限幅到 max_current 了，所以"原始"和"限幅后"是同一个数。
  // 真正的第二段行程是库内部的电流爬坡（1098.63 A/s），体现为 final_current_A。
  //
  // 撤力时必须显式报 0：库只在 Control() 里刷 targ_current（撤力那支直接写 0），
  // 而 i_des 只在 CalcPos() 里写 —— 撤力之后它会一直停在上一次的数值。
  // 不处理的话读数会变成"raw = 3.2 A、final = 0 A"，看起来像控制器还在使劲。
  // 回退前的 Control() 在 `!enabled || NoneC` 那一支里是把 raw/limited/final 一起清 0 的，
  // 这里照抄那个语义。
  const bool outputting = motor_.driver.enabled && motor_.mode != NoneC;
  raw_current_A = outputting ? motor_.motor_adrc.i_des : 0.0f;
  limited_current_A = ClampAbs(raw_current_A, motor_.motor_adrc.max_current);
  final_current_A = motor_.targ_current / kCodePerAmp;
  accepted_command_A = final_current_A; // 库不回报邮箱结果，没有第二个来源

  if (!configured_ || !origin_valid_ || !feedback_valid)
  {
    return;
  }
  height_m = (rotor_rad - origin_rad_) * config_.lift_encoder_sign * metres_per_rotor_rad_;
  velocity_m_s = float(measure.speed_rpm) * 6.28318530718f / 60.0f *
      config_.lift_encoder_sign * metres_per_rotor_rad_;
}

/**
 * @brief 把"多少米"换成编码值目标下发。
 * @note  全工程只有这一个高度下发入口，零点和软行程都在这里把关。
 *        回退前这一步是 SetPosRad() + PositionLimits()，现在合成一处：范围检查在这里，
 *        单位换算也在这里，库只收编码值。
 */
bool CatchBlockDriver::SetHeightTarget(float metres)
{
  if (!configured_)
  {
    return false;
  }
  if (!origin_valid_)
  {
    return false;
  }
  if (!std::isfinite(metres))
  {
    return false;
  }
  if (metres < 0 || metres > config_.travel_m)
  {
    return false;
  }
  // 库的 SetPos() 只在 PosC 下生效，其它模式静默丢弃 —— 这里先挡一次，免得"下发了但没反应"。
  if (motor_.mode != PosC)
  {
    return false;
  }

  const float rad = origin_rad_ + config_.lift_encoder_sign * metres / metres_per_rotor_rad_;
  motor_.SetPos(rad * kCountsPerRad);
  return true;
}

/**
 * @brief 把两个继电器开关写到引脚，顺带看总开关、为 0 就撤力。
 * @note 继电器先写，不受任何使能条件影响，方便单独测硬件通断。
 */
bool CatchBlockDriver::ApplyManualOverrides()
{
  pump_.Write(catch_pump_on != 0);
  valve_.Write(catch_valve_closed != 0);

  if (catch_motor_enable == 0)
  {
    motor_.driver.Disable();
    return false;
  }
  return true;
}

/**
 * @brief 以当前位置为保持目标，然后开始输出。
 * @note  库没有单独的 HoldHere()。Uneutral(PosC) 做的正是这件事：目标 = 当前累计角，
 *        同时清 PID、重置 TD/ESO，从当前位置无扰接管。
 *        已经使能时直接返回 —— 每 4 ms 重置一次控制器，误差永远建立不起来，等于没有闭环。
 */
bool CatchBlockDriver::Enable()
{
  if (!configured_ || !origin_valid_)
  {
    return false;
  }
  if (motor_.driver.enabled)
  {
    return true;
  }
  motor_.Uneutral(PosC);
  motor_.driver.Enable();
  return true;
}

void CatchBlockDriver::Disable()
{
  motor_.driver.Disable();
  overspeed_run_ = 0; // 撤力了就别继续攒超标拍数，重新使能时从头算
}

bool CatchBlockDriver::Ready() const
{
  return configured_;
}

bool CatchBlockDriver::OriginValid() const
{
  return origin_valid_;
}

bool CatchBlockDriver::Healthy() const
{
  return HealthTerms() == 0;
}

/**
 * @brief 逐位列出"为什么不能出力"，0 = 全过
 * @note  位序与回退前一致，`blockers = health << 4` 的换算不用改。
 *        但每条判据的来源全换了：库不再提供任何一条，全部是本层自己记的账。
 *        每条都只是读，不改变库的任何状态。
 */
uint32_t CatchBlockDriver::HealthTerms() const
{
  uint32_t terms = 0;
  const uint32_t frames = motor_.driver.measure.msg_cnt;

  if (!configured_)
  {
    terms |= HealthCtrlNotConfigured;
  }
  if (faults_ != 0)
  {
    terms |= HealthFaultLatched;
  }

  // 三个反馈位互斥：一帧没收到只管 bit4，还在库的初始化窗口里只管 bit8，
  // 收到了但过期才管 bit32。否则三个位一起亮，读数没法看。
  if (frames == 0)
  {
    terms |= HealthFeedbackNeverSeen;
  }
  else if (frames <= kOffsetInitFrames)
  {
    terms |= HealthFeedbackInvalid;
  }
  else if (FeedbackAgeUs() > config_.feedback_timeout_us)
  {
    // 注意：这一位**不锁存**。帧一回来自动就消，不需要任何复位动作。
    // 回退前这里对应的是 `_fault = FeedbackFault` 加一整套 ResetFault/ResetFeedbackReference
    // 才能解的锁存环，而 ResetFault() 又要求未使能 —— 一旦锁上就永远解不开。
    terms |= HealthFeedbackStale;
  }

  if (configured_ && motor_.use_adrc && !AdrcConfigured())
  {
    terms |= HealthAdrcInvalid;
  }
  return terms;
}

bool CatchBlockDriver::FeedbackSeen() const
{
  return motor_.driver.measure.msg_cnt > 0;
}

bool CatchBlockDriver::Enabled() const
{
  return motor_.driver.enabled;
}

/**
 * @brief 上次 CAN 发送，邮箱收下了没有
 * @note  恒 true。回退后的 MotorDJI_Driver 没有 Result / LastTransmitResult()，
 *        而 BSP::CAN::Transmit 本身就返回 void —— 发送有没有被接受，这个工程里
 *        没有任何一路可观测信号，原来那个 HealthTransmitFailed 位也就无从判断。
 *
 *        所以上层的 Fail(CanTransmit) 永远不会触发。真断总线会走另一条路：
 *        帧不再进来 → 帧年龄超时 → HealthFeedbackStale → 上层 Fail(Feedback)，
 *        那条链是通的，而且比"邮箱满"更早、更准。
 *
 *        保留这个函数和它的调用点，是为了将来 BSP 真的回报发送结果时只改这一行。
 */
bool CatchBlockDriver::LastTransmitOk() const
{
  return true;
}

uint32_t CatchBlockDriver::FaultCode() const
{
  return faults_;
}

float CatchBlockDriver::TravelMetres() const
{
  return config_.travel_m;
}

float CatchBlockDriver::TravelMarginMetres() const
{
  return config_.travel_fault_margin_m;
}

/**
 * @brief 把保持目标改成当前位置，不动使能状态，也不重置控制器
 * @note  这里**不能**图省事用 `Uneutral(PosC)`：它会把 ADRC 一起 Reset 掉，
 *        ESO 已经收敛的重力估计 z3 归零，于是"原地保持"反而先塌一下再爬回来。
 *
 *        回退前的 `MotorDJI::HoldHere()` 是怎么做的：置一个 `_hold_requested` 标志，
 *        在 `Control()` 里只改三行 ——
 *            input_nltd_3rd.v1/v2/v3 = 当前位置 / 当前速度 / 0
 *        注释写得很清楚："只修改运动参考；TD 向当前位置减速时，保留重力扰动估计、
 *        输出电流和 PID 历史状态。"那一整套现在散了，但 `input_nltd_3rd` 和它的
 *        v1/v2/v3 都是公开成员，所以在应用层原样复刻：换参考、不碰观测器。
 *
 *        效果：目标变成"此刻的位置和速度"，TD 从这里开始平滑减速到停，
 *        ESO / 输出电流 / 积分项全都不动，和回退前完全一致。
 */
bool CatchBlockDriver::HoldHere()
{
  if (!configured_ || !origin_valid_)
  {
    return false;
  }
  if (motor_.mode != PosC) // 不在位置环下改参考没有意义，库的 SetPos 也会静默丢弃
  {
    return false;
  }

  const moto_measure_t &measure = motor_.driver.measure;
  const float code = float(measure.total_angle);

  motor_.SetPos(code); // 目标 := 当前位置（库收编码值，只在 PosC 下生效）
  motor_.motor_adrc.input_nltd_3rd.v1 = code * kRadPerCount;
  motor_.motor_adrc.input_nltd_3rd.v2 = float(measure.speed_rpm) * 6.28318530718f / 60.0f;
  motor_.motor_adrc.input_nltd_3rd.v3 = 0.0f;
  return true;
}

/**
 * @brief 清本层锁存的故障位
 * @note  要求已经撤力：带着故障继续出力是最危险的状态，先关掉再来清。
 *        回退前同一个名字的函数开头是 `if (driver.enabled) return false;`，
 *        但那时故障会让 Enable() 永远发不出去，形成死锁；现在故障只挡输出，
 *        上层 Fail() 第一件事就是 Disable()，所以这条前置条件一定满足。
 */
void CatchBlockDriver::ResetFault()
{
  if (motor_.driver.enabled)
  {
    return;
  }
  faults_ = 0;
  overspeed_run_ = 0;
}

void CatchBlockDriver::Neutral()
{
  motor_.Neutral();
}

void CatchBlockDriver::Uneutral()
{
  motor_.Uneutral(PosC);
}

bool CatchBlockDriver::MotorEnableRequested() const
{
  return catch_motor_enable != 0;
}

bool CatchBlockDriver::PumpRequested() const
{
  return catch_pump_on != 0;
}

bool CatchBlockDriver::ValveRequested() const
{
  return catch_valve_closed != 0;
}

bool CatchBlockDriver::PumpPinLevel() const
{
  return pump_.Read();
}

bool CatchBlockDriver::ValvePinLevel() const
{
  return valve_.Read();
}
