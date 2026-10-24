#pragma once
#include "catch_block_config.hpp"
#include "motor_dji.hpp"
#include "bsp_gpio.hpp"

/**
 * @file catch_block_driver.hpp
 * @brief
 *
 * 这层只干四件事：单位换算、硬件配置、采样、输出。
 * 它不认识状态机，也不做安全策略 —— 那是 catch_block.hpp 的事。
 *
 * 运行上下文：所有方法只在 App 任务（4 ms 一拍）里调用，内部不加临界区。
 */

// Keil 手动强制输出：写非 0 为开，写 0 为关。写 0 是"撤力/松开"，不是刹车。
// 引脚按实际布线：PE6 = 电磁阀（夹爪），PE12 = 气泵（吸盘）。
extern "C" volatile uint8_t catch_pump_on;      ///< @brief PE12 气泵：非 0 = 吸附
extern "C" volatile uint8_t catch_valve_closed; ///< @brief PE6 电磁阀：非 0 = 夹爪闭合
extern "C" volatile uint8_t catch_motor_enable; ///< @brief 非 0 = 允许输出，还要过业务层那一关

class CatchBlockDriver
{
public:
  /**
   * @brief Configure() 的三种结果。
   * @note 前两种必须分开：Rejected 只算这次没配成，HardwareFault 要报故障。
   */
  enum class ConfigResult : uint8_t
  {
    Rejected,     ///< 前置条件不满足，什么都还没动
    Applied,      ///< 配好了，下一步可以标零
    HardwareFault ///< 前置过了，但初始化 / 反馈守卫 / 参数下发失败
  };

  /** @name 配置与标零 */
  /** @{ */
  /**
   * @brief 把 CAN、反馈守卫、位置环按参数配好。
   * @note 配好之后仍然不输出，要等上层使能。要求电机已经停下、两路 GPIO 有效。
   */
  ConfigResult Configure(const CatchBlockConfig &config);

  /**
   * @brief 把现在的位置记成 0 m，并按行程划出运动范围。
   * @note 机构还在动就不记，免得把半路的位置当成原点。
   */
  bool ArmAtCurrentPosition();

  /**
   * @brief 丢掉零点和"已配置"标记，回到不能动的状态。
   * @note 不清故障码，那个由 ResetFault() 单独负责。
   */
  void ResetConfiguration();
  /** @} */

  /** @name 采样与查询 */
  /** @{ */
  /**
   * @brief 把编码器、电流、温度读进下面的公有成员。
   * @note 必须在健康判定和越界判定之前调用，否则检查用的是上一拍的数据。
   */
  void Sample();

  bool Ready() const;               ///< @brief 参数和控制器配好了没有
  bool OriginValid() const;         ///< @brief 零点采了没有（采了才能把角度换成米）
  bool Healthy() const;             ///< @brief 反馈和控制器有没有故障
  bool Enabled() const;             ///< @brief 驱动现在是不是正在输出电流
  bool LastTransmitOk() const;      ///< @brief 上次发 CAN 指令，邮箱收下了没有
  uint32_t FaultCode() const;       ///< @brief 驱动自己锁存的故障位
  float TravelMetres() const;       ///< @brief 行程多少米
  float TravelMarginMetres() const; ///< @brief 越界判定在两端各放宽多少米
  /** @} */

  /** @name 输出 */
  /** @{ */
  /**
   * @brief 把两个继电器开关写到引脚上；顺带看总开关，为 0 就撤力。
   * @note 继电器写在最前面，不受任何使能条件影响，方便单独做硬件通断测试。
   *       返回 false 表示"这一拍别再去尝试使能了"。
   */
  bool ApplyManualOverrides();

  /**
   * @brief 以当前位置为保持目标，然后开始输出。
   * @note 不会沿用上一次的目标，避免突然冲出去。
   */
  bool Enable();

  void Disable();                     ///< @brief 停止输出。重复调用没关系
  bool SetHeightTarget(float metres); ///< @brief 把"多少米"换成转子目标角下发；没标零或超行程会拒绝
  bool HoldHere();                    ///< @brief 把保持目标改成当前位置，等于原地停住
  void ResetFault();                  ///< @brief 清掉驱动锁存的故障位
  void ResetFeedbackReference();      ///< @brief 重置反馈参考点，用来解掉跳变锁存
  void Neutral();                     ///< @brief 退出闭环，不再控制
  void Uneutral();                    ///< @brief 重新进入位置闭环
  /** @} */

  /** @name 标志位查询 */
  /** @{ */
  bool MotorEnableRequested() const; ///< @brief Keil 那个总开关是不是被打开了
  bool PumpRequested() const;        ///< @brief 气泵是不是被要求打开（是意图，不是引脚电平）
  bool ValveRequested() const;       ///< @brief 夹爪是不是被要求闭合
  bool PumpPinLevel() const;         ///< @brief PE12 现在实际是高还是低，只看不判断
  bool ValvePinLevel() const;        ///< @brief PE6 现在实际是高还是低
  /** @} */

  /** @name 采样结果（Sample() 刷新，外部只读） */
  /** @{ */
  float height_m = 0;          ///< @brief 相对原点的高度，m
  float velocity_m_s = 0;      ///< @brief 高度方向的速度，m/s
  float rotor_rad = 0;         ///< @brief 编码器累计的转子角度，rad
  bool feedback_valid = false; ///< @brief 反馈建立了且没故障
  /** @} */

  /** @name 控制器诊断（Sample() 刷新，外部只读，调 ADRC 时看这些） */
  /** @{ */
  float target_position_rad = 0;       ///< @brief 位置环想要到达的角度，rad
  float measured_position_rad = 0;     ///< @brief 编码器实测的角度，rad
  float reference_position_rad = 0;    ///< @brief TD 平滑后的参考位置 v1，rad
  float reference_velocity_rad_s = 0;  ///< @brief TD 平滑后的参考速度 v2，rad/s
  float eso_position_rad = 0;          ///< @brief ESO 估的位置 z1，rad
  float eso_velocity_rad_s = 0;        ///< @brief ESO 估的速度 z2，rad/s
  float disturbance_rad_s2 = 0;        ///< @brief ESO 估的总扰动 z3，rad/s²
  float raw_current_A = 0;             ///< @brief ADRC 算出来的原始电流，A
  float limited_current_A = 0;         ///< @brief 限幅之后的电流，A
  float final_current_A = 0;           ///< @brief 真正发出去的电流，A
  float accepted_command_A = 0;        ///< @brief CAN 邮箱收下的电流，不等于电机真的出了这么多
  int16_t feedback_current_raw = 0;    ///< @brief 电调回报的电流原始码，没标定
  bool feedback_current_valid = false; ///< @brief false 时上面这个原始码不能当安培用
  uint8_t temperature_C = 0;           ///< @brief 电调温度，°C
  /** @} */

private:
  MotorDJI motor_;                      ///< @brief 电机控制层：驱动 + ADRC + 电流链路
  BSP::GPIO::Inst pump_{Pin{'E', 12}};  ///< @brief PE12 气泵继电器（吸盘，标签 XiPan）
  BSP::GPIO::Inst valve_{Pin{'E', 6}};  ///< @brief PE6 电磁阀继电器（夹爪，标签 DianCiFa）

  CatchBlockConfig config_;        ///< @brief 生效的硬件参数，只在 Configure() 里赋值
  float origin_rad_ = 0;           ///< @brief 零点对应的转子角度，rad
  float metres_per_rotor_rad_ = 0; ///< @brief 转子转 1 弧度走多少米，1:1 传动约 1.3019 mm/rad，全工程只算这一处
  bool configured_ = false;        ///< @brief 控制器已配置
  bool origin_valid_ = false;      ///< @brief 零点有效
};
