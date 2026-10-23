#include "catch_block_config.hpp"
#include <cmath>

namespace
{
/**
 * @brief 有限正数判定
 * @note 拒绝 NaN / Inf / 0，避免它们被当作"看起来合法"的阈值传进控制器。
 */
bool Positive(float x)
{
  return std::isfinite(x) && x > 0;
}
}

/**
 * @brief 校验业务阈值
 * @note 顺序固定：四个实测标志 → 速度/加速度自洽 → 到位与余量 → 时间参数范围。
 */
bool CatchBlockConfig::Validate() const
{
  return direction_verified && model_verified && current_limits_verified && motion_limits_verified &&
      Positive(travel_m) && Positive(speed_m_s) && Positive(acceleration_m_s2) &&
      Positive(overspeed_m_s) && overspeed_m_s > speed_m_s &&
      Positive(position_tolerance_m) && position_tolerance_m < travel_m / 10 &&
      Positive(settled_speed_m_s) && settled_speed_m_s < speed_m_s &&
      Positive(travel_fault_margin_m) && travel_fault_margin_m < travel_m / 10 &&
      settle_ms >= 100 && settle_ms <= 10000 && relay_dwell_ms <= 5000 &&
      phase_timeout_ms >= 1000 && phase_timeout_ms <= 600000;
}

/**
 * @brief 校验硬件参数
 * @note max_current_A 上限取 20 A，是 C620 的额定上限，不是本机构的可用值。
 */
bool CatchBlockConfig::ValidateHardware() const
{
  return esc_id >= 1 && esc_id <= 8 &&
      (lift_encoder_sign == 1 || lift_encoder_sign == -1) &&
      (current_encoder_sign == 1 || current_encoder_sign == -1) &&
      Positive(inertia_kg_m2) && Positive(torque_constant_Nm_A) &&
      std::isfinite(viscous_Nm_s_rad) && viscous_Nm_s_rad >= 0 &&
      Positive(max_current_A) && max_current_A <= 20 && Positive(current_slew_A_s) &&
      Positive(observer_Hz) && Positive(controller_Hz) && controller_Hz < observer_Hz &&
      Positive(pulley_radius_m) && Positive(external_speed_ratio) && Positive(gearbox_ratio) &&
      feedback_timeout_us >= 2000 && feedback_timeout_us <= 20000;
}
