#include "catch_block_driver.hpp"
#include "bsp_hardware.hpp"
#include <cmath>

// 三个硬件开关的定义（声明见头文件），上电全为 0：泵停、阀开、电机不输出。
extern "C"
{
volatile uint8_t catch_pump_on = 0;
volatile uint8_t catch_valve_closed = 0;
volatile uint8_t catch_motor_enable = 0;
}

namespace
{
/// @brief 编码器 1 count 对应的转子角度：2π/8192 ≈ 7.67e-4 rad
constexpr float RadPerCount = 6.28318530718f / 8192.0f;

/**
 * @brief 有限正数判定
 * @note 换算系数可能因为参数写错变成 0 或 Inf，这里单独判一次。
 */
bool Positive(float x)
{
  return std::isfinite(x) && x > 0;
}
}

/**
 * @brief 初始化 CAN、反馈守卫与 ADRC 位置环
 * @note 失败路径都不改 configured_，由返回值区分"拒绝"与"硬件故障"。
 */
CatchBlockDriver::ConfigResult CatchBlockDriver::Configure(const CatchBlockConfig &config)
{
  if (motor_.driver.enabled || !config.ValidateHardware() ||
      !pump_.IsValid() || !valve_.IsValid() ||
      (configured_ && config.esc_id != config_.esc_id))
  {
    return ConfigResult::Rejected;
  }

  // 高度换算系数：带轮半径 × 齿轮增速 ÷ 减速比（m/rad，转子侧）。全工程只算这一处。
  const float metres_per_rad = config.pulley_radius_m * config.external_speed_ratio / config.gearbox_ratio;
  if (!Positive(metres_per_rad))
  {
    return ConfigResult::Rejected;
  }

  config_ = config;
  metres_per_rotor_rad_ = metres_per_rad;
  motor_.Init(Hardware::hcan_main, config_.esc_id, DJI_C620);
  if (!motor_.driver.Initialized())
  {
    return ConfigResult::HardwareFault;
  }

  const float rad_s = config_.speed_m_s / metres_per_rotor_rad_; // 规划限速 (rad/s)
  const float guard_rpm = config_.overspeed_m_s / metres_per_rotor_rad_ * 60.0f / 6.28318530718f;
  // 同一门限两用：帧级跳变/速度检查，以及控制级超速保护。超速即锁存，调参时要留余量。
  if (!motor_.driver.ConfigureFeedbackGuard(guard_rpm, config_.feedback_timeout_us))
  {
    return ConfigResult::HardwareFault;
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
    return ConfigResult::HardwareFault;
  }

  // 配置成功：零点作废，必须重新标零才能运动。
  configured_ = true;
  origin_valid_ = false;
  return ConfigResult::Applied;
}

/**
 * @brief 以当前静止位置为零点并设软限位
 * @note 软限位 = [原点, 顶端] 两端各留 travel_fault_margin_m；它挡不住失电下坠。
 */
bool CatchBlockDriver::ArmAtCurrentPosition()
{
  if (!configured_ || !motor_.Healthy())
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
  motor_.PositionLimits(fminf(origin_rad_, end) - margin, fmaxf(origin_rad_, end) + margin);
  origin_valid_ = true;
  motor_.Uneutral(PosC);
  return true;
}

void CatchBlockDriver::ResetConfiguration()
{
  configured_ = false;
  origin_valid_ = false;
}

/**
 * @brief 刷新所有公有采样量与诊断量
 * @note 高度/速度只在"已配置 + 已标零 + 反馈有效"三个条件同时成立时更新，否则保留旧值。
 */
void CatchBlockDriver::Sample()
{
  const auto fb = motor_.driver.GetFeedbackSnapshot();
  feedback_valid = fb.valid;
  feedback_current_raw = fb.measure.raw_current;
  feedback_current_valid = fb.measure.current_amp_valid;
  temperature_C = fb.measure.temprature;

  // 控制器内部量：直接映射，供 Keil 调参时对照 TD / ESO / 电流三段。
  target_position_rad = motor_.targ_pos_rad;
  measured_position_rad = motor_.position_rad;
  reference_position_rad = motor_.motor_adrc.input_nltd_3rd.v1;
  reference_velocity_rad_s = motor_.motor_adrc.input_nltd_3rd.v2;
  eso_position_rad = motor_.motor_adrc.eso.z1;
  eso_velocity_rad_s = motor_.motor_adrc.eso.z2;
  disturbance_rad_s2 = motor_.motor_adrc.eso.z3;
  raw_current_A = motor_.raw_current_A;
  limited_current_A = motor_.limited_current_A;
  final_current_A = motor_.final_current_A;
  accepted_command_A = motor_.driver.LastAcceptedCommandAmp();

  if (!configured_ || !origin_valid_ || !fb.valid)
  {
    return;
  }
  rotor_rad = float(fb.measure.total_angle) * RadPerCount;
  height_m = (rotor_rad - origin_rad_) * config_.lift_encoder_sign * metres_per_rotor_rad_;
  velocity_m_s = float(fb.measure.speed_rpm) * 6.28318530718f / 60 *
      config_.lift_encoder_sign * metres_per_rotor_rad_;
}

/**
 * @brief 米 → 转子目标角
 * @note 唯一的高度下发入口，零点与行程范围都在这里把关。
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
  return motor_.SetPosRad(origin_rad_ + config_.lift_encoder_sign * metres / metres_per_rotor_rad_);
}

/**
 * @brief 两路继电器跟随标志位，并在 catch_motor_enable 为 0 时撤力
 * @note 继电器写在最前面，不受任何使能条件影响，便于单独做硬件通断测试。
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

bool CatchBlockDriver::Enable()
{
  if (!motor_.HoldHere())
  {
    return false;
  }
  motor_.driver.Enable();
  return true;
}

void CatchBlockDriver::Disable()
{
  motor_.driver.Disable();
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
  return motor_.Healthy();
}

bool CatchBlockDriver::Enabled() const
{
  return motor_.driver.enabled;
}

bool CatchBlockDriver::LastTransmitOk() const
{
  return motor_.driver.LastTransmitResult() == MotorDJI_Driver::Result::Ok;
}

uint32_t CatchBlockDriver::FaultCode() const
{
  return motor_.FaultCode();
}

float CatchBlockDriver::TravelMetres() const
{
  return config_.travel_m;
}

float CatchBlockDriver::TravelMarginMetres() const
{
  return config_.travel_fault_margin_m;
}

bool CatchBlockDriver::HoldHere()
{
  return motor_.HoldHere();
}

void CatchBlockDriver::ResetFault()
{
  motor_.ResetFault();
}

void CatchBlockDriver::ResetFeedbackReference()
{
  motor_.driver.ResetFeedbackReference();
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
