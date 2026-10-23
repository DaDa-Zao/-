#pragma once
#include "catch_block_config.hpp"
#include "motor_dji.hpp"
#include "bsp_gpio.hpp"

/**
 * @file catch_block_driver.hpp
 * @brief 取块机构的机构驱动层：一台带减速比的升降轴 + 两路继电器。
 *
 * 层次关系（三层都叫 driver，容易混，这里一次写清）：
 *   CatchBlockDriver          机构轴：米 / 转子 rad / 两路继电器 / 零点 / 软限位
 *     └ MotorDJI              控制层：ADRC + 电流链路
 *         └ MotorDJI_Driver   CAN 收发：电流码
 *
 * 本层只做四件事：单位换算、硬件配置、采样、执行器输出；不认识状态机，也不做安全策略。
 * 一旦本文件 include 了 catch_block.hpp，就说明有东西放错了层。
 *
 * 命名约定：公有采样量与诊断量沿用 MotorDJI 的做法直接公开（无下划线）供 Watch 展开；
 *           私有成员一律带下划线。公有量由 Sample() 每拍刷新，外部只读。
 *
 * 运行上下文：所有方法只在 App 任务（名义 5 ms）里调用，内部不加临界区。
 */

// Keil 手动强制输出：写非 0 为开，写 0 为关。写 0 是"撤力/松开"，不是刹车。
extern "C" volatile uint8_t catch_pump_on;      ///< @brief PE6 气泵：非 0 = 吸附
extern "C" volatile uint8_t catch_valve_closed; ///< @brief PE12 电磁阀：非 0 = 夹爪闭合
extern "C" volatile uint8_t catch_motor_enable; ///< @brief 非 0 = 允许输出，仍受业务层门控

class CatchBlockDriver
{
public:
  /**
   * @brief Configure() 的三种结局
   * @note 必须区分 Rejected 与 HardwareFault：前者只记一次拒绝，后者要锁存 Configuration 故障。
   */
  enum class ConfigResult : uint8_t
  {
    Rejected,      ///< 前置条件不满足，未下发任何配置
    Applied,       ///< 配置成功，可以标零
    HardwareFault  ///< 前置通过但 Init / 反馈守卫 / 链式配置失败
  };

  /** @name 一次性配置 */
  /** @{ */
  /**
   * @brief 初始化 CAN、反馈守卫与 ADRC 位置环
   * @note 要求电机已禁用、两路 GPIO 有效、硬件参数合法、不换 ESC 编号；成功后仍不使能。
   */
  ConfigResult Configure(const CatchBlockConfig &config);
  /**
   * @brief 以当前静止位置为零点并设软限位
   * @note 速度超过 settled_speed_m_s 时拒绝采零，避免把运动中的位置当原点。
   */
  bool ArmAtCurrentPosition();
  /**
   * @brief 丢弃零点与"已配置"标记
   * @note 不清驱动故障码，那由 ResetFault() / ResetFeedbackReference() 分别负责。
   */
  void ResetConfiguration();
  /** @} */

  /** @name 采样与查询 */
  /** @{ */
  /**
   * @brief 刷新下面所有公有采样量与诊断量
   * @note 必须在健康判定与越界判定之前调用，否则检查会用上一拍的数据。
   */
  void Sample();
  bool Ready() const;                ///< @brief 参数与控制器已配置完成
  bool OriginValid() const;          ///< @brief 零点已采，高度可换算
  bool Healthy() const;              ///< @brief 数据与控制器健康，不代表机构有支承
  bool Enabled() const;              ///< @brief 驱动是否正在输出
  bool LastTransmitOk() const;       ///< @brief 上次 CAN 发送是否被邮箱接受
  uint32_t FaultCode() const;        ///< @brief 驱动锁存的故障位
  float TravelMetres() const;        ///< @brief 行程 (m)
  float TravelMarginMetres() const;  ///< @brief 越界余量 (m)
  /** @} */

  /** @name 执行器输出 */
  /** @{ */
  /**
   * @brief 两路继电器跟随标志位，并在 catch_motor_enable 为 0 时撤力
   * @note 返回 false 表示上层不要再尝试使能，与拆分前的 return 时机一致。
   */
  bool ApplyManualOverrides();
  /**
   * @brief 以当前位置为保持目标后使能
   * @note 不会自动选目标，避免续用上一次运动的旧目标导致突然冲出去。
   */
  bool Enable();
  void Disable();                        ///< @brief 幂等，重复调用无副作用
  bool SetHeightTarget(float metres);    ///< @brief 米 → 转子目标角，含零点与范围检查
  bool HoldHere();                       ///< @brief 把保持目标改成当前位置
  void ResetFault();
  void ResetFeedbackReference();
  void Neutral();
  void Uneutral();
  /** @} */

  /** @name 标志位查询 */
  /** @{ */
  bool MotorEnableRequested() const;  ///< @brief catch_motor_enable != 0
  bool PumpRequested() const;         ///< @brief catch_pump_on != 0（意图，不是引脚电平）
  bool ValveRequested() const;        ///< @brief catch_valve_closed != 0
  bool PumpPinLevel() const;          ///< @brief 实际引脚电平，仅供诊断
  bool ValvePinLevel() const;         ///< @brief 实际引脚电平，仅供诊断
  /** @} */

  /** @name 采样结果（Sample() 刷新，外部只读） */
  /** @{ */
  float height_m = 0;        ///< @brief 相对原点的高度 (m)
  float velocity_m_s = 0;    ///< @brief 高度方向速度 (m/s)
  float rotor_rad = 0;       ///< @brief 实测转子累计角 (rad)
  bool feedback_valid = false; ///< @brief 反馈已建立且无故障
  /** @} */

  /** @name 控制器诊断（Sample() 刷新，外部只读） */
  /** @{ */
  float target_position_rad = 0;       ///< @brief 位置环目标 (rad)
  float measured_position_rad = 0;     ///< @brief 位置环反馈 (rad)
  float reference_position_rad = 0;    ///< @brief TD 参考位置 v1 (rad)
  float reference_velocity_rad_s = 0;  ///< @brief TD 参考速度 v2 (rad/s)
  float eso_position_rad = 0;          ///< @brief ESO z1：估计位置 (rad)
  float eso_velocity_rad_s = 0;        ///< @brief ESO z2：估计速度 (rad/s)
  float disturbance_rad_s2 = 0;        ///< @brief ESO z3：估计总扰动加速度 (rad/s²)
  float raw_current_A = 0;             ///< @brief ADRC 原始输出 (A)
  float limited_current_A = 0;         ///< @brief 限幅后 (A)
  float final_current_A = 0;           ///< @brief 量化后真正下发的物理电流 (A)
  float accepted_command_A = 0;        ///< @brief CAN 邮箱已接受，不是实测电流
  int16_t feedback_current_raw = 0;    ///< @brief 反馈原始电流码，未标定
  bool feedback_current_valid = false; ///< @brief false 时 raw 不能当安培用
  uint8_t temperature_C = 0;           ///< @brief 电调温度 (°C)
  /** @} */

private:
  MotorDJI motor_;  ///< @brief 电机控制层：驱动 + ADRC + 电流链路
  BSP::GPIO::Inst pump_{Pin{'E', 6}};   ///< @brief PE6 气泵继电器
  BSP::GPIO::Inst valve_{Pin{'E', 12}}; ///< @brief PE12 电磁阀继电器

  CatchBlockConfig config_;         ///< @brief 生效的硬件参数副本，只在 Configure() 赋值
  float origin_rad_ = 0;            ///< @brief 零点转子角 (rad)
  float metres_per_rotor_rad_ = 0;  ///< @brief 高度换算系数（约 2.6037 mm/rad），全工程只算这一处
  bool configured_ = false;         ///< @brief 控制器已配置
  bool origin_valid_ = false;       ///< @brief 零点有效
};
