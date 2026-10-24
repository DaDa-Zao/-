#pragma once
#include <cstdint>

/**
 * @brief 取块机构的全部参数，一张表填完。
 * @note 24 个字段平铺，不做嵌套，方便在 Keil Watch 里从上往下填。
 *       校验分两段：Validate() 查工艺阈值，ValidateHardware() 查电机与量程。
 *       业务层和驱动层都要用这张表，所以单独成文件，避免两个头文件互相包含。
 *       这里没有"已核实"之类的签字位：能配通只说明数值合法，不说明方向测过。
 *       方向、惯量、限流、限速该实测的，按 README 的阶段 1~3 走一遍即可。
 */
class CatchBlockConfig
{
public:
  uint8_t esc_id = 1;                      ///< @brief 电调编号 1~8，反馈 ID = 0x200 + 编号
  int8_t lift_encoder_sign = 1;            ///< @brief 只能填 +1 或 -1：往上走时编码器读数往哪边变
  int8_t current_encoder_sign = 1;         ///< @brief 只能填 +1 或 -1：给正电流时转子往哪边转
  float inertia_kg_m2 = 1.5e-5;            ///< @brief 折算到电机转子上的整机惯量，kg·m²。1:1 传动约 1.5e-5
  float torque_constant_Nm_A = 0.3;        ///< @brief 每安培出多少扭矩，N·m/A。要填转子值，手册 0.3 是输出轴值
  float viscous_Nm_s_rad = 0;              ///< @brief 粘性阻尼，N·m·s/rad。填 0 表示让 ESO 自己去估
  float max_current_A = 8;                 ///< @brief 允许输出的最大电流，A。必须大于静态托举电流才托得住
  float current_slew_A_s = 2000;           ///< @brief 电流每秒最多涨多少 A。太小托不住，太大有冲击
  float observer_Hz = 20;                  ///< @brief ESO 观测器带宽，Hz。必须大于 controller_Hz
  float controller_Hz = 4;                 ///< @brief 位置环带宽，Hz
  float travel_m = 0.395f;                 ///< @brief 原点往上能走多远，m
  float pulley_radius_m = 0.025f;          ///< @brief 同步带轮有效半径，m
  float external_speed_ratio = 1.0f;       ///< @brief 电机齿轮与带轮齿轮的转速比，本项目 1:1 直连。改它所有换算都变
  float gearbox_ratio = 3591.0f / 187.0f;  ///< @brief M3508 减速比，约 19.2，别改
  float speed_m_s = 0.020f;                ///< @brief 规划的最高速度，m/s
  float acceleration_m_s2 = 0.040f;        ///< @brief 规划的加速度，m/s²
  float overspeed_m_s = 0.030f;            ///< @brief 超速门限，m/s。要留 1.2~1.5 倍余量，别贴着 speed 设
  float position_tolerance_m = 0.001f;     ///< @brief 离目标多近算到位，m
  float settled_speed_m_s = 0.001f;        ///< @brief 速度低于多少算停稳，m/s
  float travel_fault_margin_m = 0.003f;    ///< @brief 两端各再放宽多少算越界，m
  uint32_t feedback_timeout_us = 10000;    ///< @brief 多久收不到反馈就判掉线，µs，范围 2000~20000
  uint32_t settle_ms = 500;                ///< @brief 到位后要稳住多久才算真到位，ms
  uint32_t relay_dwell_ms = 300;           ///< @brief 夹爪/气泵两次动作之间至少隔多久，ms
  uint32_t phase_timeout_ms = 45000;       ///< @brief 每个流程阶段最长等多久，ms

  /**
   * @brief 检查工艺阈值有没有自相矛盾。
   * @note 通过不代表调好了，只代表这些数不会让判据互相打架。
   */
  bool Validate() const;

  /**
   * @brief 检查电机参数和量程对不对。
   * @note 编号范围、两个方向符号、惯量、带宽大小关系、电流上限、换算量程、反馈超时窗口。
   */
  bool ValidateHardware() const;
};
