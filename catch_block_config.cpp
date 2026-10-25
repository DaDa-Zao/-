#include "catch_block_config.hpp"
#include <cmath>

namespace
{
/**
 * @brief 是不是有限的正数。
 * @note  挡掉 NaN、Inf 和 0 —— 这三个看起来都"填了"，但都不能当阈值用。
 */
bool Positive(float x)
{
  return std::isfinite(x) && x > 0;
}
}

/**
 * @brief 检查工艺阈值有没有自相矛盾。
 * @note 顺序是：先看速度和加速度的关系，再看容差和余量，最后看时间参数在不在合理范围。
 */
bool CatchBlockConfig::Validate() const
{
  return Positive(travel_m) && Positive(speed_m_s) && Positive(acceleration_m_s2) &&
      Positive(overspeed_m_s) && overspeed_m_s > speed_m_s &&
      Positive(position_tolerance_m) && position_tolerance_m < travel_m / 10 &&
      Positive(settled_speed_m_s) && settled_speed_m_s < speed_m_s &&
      Positive(travel_fault_margin_m) && travel_fault_margin_m < travel_m / 10 &&
      settle_ms >= 100 && settle_ms <= 10000 && relay_dwell_ms <= 5000 &&
      phase_timeout_ms >= 1000 && phase_timeout_ms <= 600000;
}

/**
 * @brief 检查电机参数和量程对不对。
 * @note 电流上限卡在 20 A，那是 C620 的额定值，不是这台机构能用的值。
 *       电流方向符号只接受 +1：电机库回退后没有极性反转接口，反向电机必须靠调相线解决，
 *       详情见头文件里"电流方向"那一段。这里挡掉，比让它使能后飞车再查要安全得多。
 */
bool CatchBlockConfig::ValidateHardware() const
{
  return esc_id >= 1 && esc_id <= 8 &&
      (lift_encoder_sign == 1 || lift_encoder_sign == -1) &&
      current_encoder_sign == 1 &&
      Positive(inertia_kg_m2) && Positive(torque_constant_Nm_A) &&
      std::isfinite(viscous_Nm_s_rad) && viscous_Nm_s_rad >= 0 &&
      Positive(max_current_A) && max_current_A <= 20 && Positive(current_slew_A_s) &&
      Positive(observer_Hz) && Positive(controller_Hz) && controller_Hz < observer_Hz &&
      Positive(pulley_radius_m) && Positive(external_speed_ratio) && Positive(gearbox_ratio) &&
      feedback_timeout_us >= 2000 && feedback_timeout_us <= 20000;
}
