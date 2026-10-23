#pragma once
#include <cstdint>

/**
 * @brief 取块机构的全部可调参数
 * @note 28 个字段平铺在一起，便于在 Keil Watch 里一次性填完，不做嵌套。
 *       业务阈值（行程、容差、超时）与硬件参数（编号、惯量、限流）混在同一张表里，
 *       但校验分成两段：Validate() 管工艺阈值，ValidateHardware() 管电机与量程，
 *       分别由业务层和机构驱动层调用，两者同时为真才允许下发。
 *       本类被两层共用，因此单独成文件，避免 catch_block.hpp 与 driver 互相包含。
 */
class CatchBlockConfig
{
public:
  uint8_t esc_id = 1;                      ///< @brief C620 编号 1..8，反馈 ID = 0x200 + id（CAN1 应为 0x201）
  int8_t lift_encoder_sign = 0;            ///< @brief ±1：高度上升时编码器累计值的方向
  int8_t current_encoder_sign = 0;         ///< @brief ±1：正 CAN 电流使转子加速的方向
  bool direction_verified = false;         ///< @brief 两个方向都已实测
  bool model_verified = false;             ///< @brief J / Kt 已确认或辨识
  bool current_limits_verified = false;    ///< @brief 电流上限与斜率已按温升、机械确认
  bool motion_limits_verified = false;     ///< @brief 速度/加速度/超速门限已实测
  float inertia_kg_m2 = 0;                 ///< @brief kg·m²，折算到转子轴的整机惯量
  float torque_constant_Nm_A = 0;          ///< @brief N·m/A，转子域（手册 0.3 是输出轴值）
  float viscous_Nm_s_rad = 0;              ///< @brief N·m·s/rad，0 表示交给 ESO 吸收
  float max_current_A = 0;                 ///< @brief A，必须大于静态承载电流
  float current_slew_A_s = 0;              ///< @brief A/s，太小托不住，太大冲击
  float observer_Hz = 0;                   ///< @brief ESO 带宽，须大于 controller_Hz
  float controller_Hz = 0;                 ///< @brief 位置环带宽
  float travel_m = 0.395f;                 ///< @brief m，原点至顶端，仍须核对两端余量
  float pulley_radius_m = 0.025f;          ///< @brief m，同步带轮有效节圆半径
  float external_speed_ratio = 2.0f;       ///< @brief 大/小齿轮转速比（带轮是输出的 2 倍）
  float gearbox_ratio = 3591.0f / 187.0f;  ///< @brief 减速箱比，来自 M3508 手册
  float speed_m_s = 0.020f;                ///< @brief m/s，TD 规划限速（候选值）
  float acceleration_m_s2 = 0.040f;        ///< @brief m/s²，TD 规划限加速度（候选值）
  float overspeed_m_s = 0.030f;            ///< @brief m/s，超速门限，应留 1.2~1.5 倍余量
  float position_tolerance_m = 0.001f;     ///< @brief m，到位判定
  float settled_speed_m_s = 0.001f;        ///< @brief m/s，到位判定的速度门限
  float travel_fault_margin_m = 0.003f;    ///< @brief m，软限位与越界判定的余量
  uint32_t feedback_timeout_us = 10000;    ///< @brief µs，反馈超时
  uint32_t settle_ms = 500;                ///< @brief ms，到位需持续稳定的时间
  uint32_t relay_dwell_ms = 300;           ///< @brief ms，动作间隔，不证明动作成功
  uint32_t phase_timeout_ms = 45000;       ///< @brief ms，单阶段超时

  /**
   * @brief 校验业务阈值
   * @note 只看"四个实测标志 + 速度/容差/余量/超时是否自洽"，通过不代表参数已调好，
   *       只代表取值不可能让状态机进入自相矛盾的判据。
   */
  bool Validate() const;

  /**
   * @brief 校验硬件参数
   * @note 编号范围、两个方向符号、惯量与转矩常数、带宽关系、限流上限、换算量程、反馈超时窗口。
   *       与 Validate() 同时为真才允许下发到驱动。
   */
  bool ValidateHardware() const;
};
