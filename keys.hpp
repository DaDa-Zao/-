#pragma once
#include <cstdint>
#include "bsp_gpio.hpp"

/**
 * @file keys.hpp
 * @brief 面板按键：9 个独立按键，替代 Keil Watch 里手改 command 的操作。
 *
 * 【为什么要有这一层】
 * 取放过程要两只手扶着机构或物块，腾不出手去点鼠标改 Watch。
 * 这一层把最常用的动作变成"按一下"：吸盘/夹爪翻转、重配、标零、撤力、
 * 升顶、回原点，外加使能/失能两个安全开关。
 *
 * 【硬件约定】
 * 每个按键一端接 GND、一端接 GPIO，引脚配成**输入上拉**：
 * 松开读到 1，按下读到 0。不需要外部电阻，也不需要额外电源。
 *
 * 【9 个键怎么装下 11 个动作】
 * `7/8`（吸盘开/关）合成一个 toggle 键，`9/10`（夹爪闭合/松开）同理。
 * toggle **直接翻转 `catch_pump_on` / `catch_valve_closed` 这两个权威变量，
 * 不另存一份状态** —— 否则 Keil 手改、自动流程改动、按键漏检三个来源一对不上，
 * 之后每次按都反着来，极难查。代价是当前状态看不出来，
 * 看 `catch_debug.pump_on` / `valve_closed`（那是引脚实际电平，比请求更可信）。
 *
 * 【长按分级：危险动作要确认，安全动作要立即】
 *   `command = 5`（撤力，带载会掉块）      长按 2 s
 *   `command = 1 / 2`（重配、标零）        长按 1 s
 *   使能（机构会动）                       长按 0.5 s
 *   失能（安全方向）                       **按下立即**
 *   吸盘 / 夹爪 / 升顶 / 回原点            按下立即
 *
 * ⚠ **自动流程 13~16 依赖两路继电器状态**：`state = 4` 推进到 5 要求
 * `PumpRequested() && ValveRequested()`，`command = 14` 也要求两路都为真。
 * 所以自动流程跑到一半按 toggle，会让流程卡在 state 4 或被拒 —— 手动阶段随便按，
 * 自动流程中别碰。
 *
 * 运行上下文：只在 App 任务（5 ms 一拍，200 Hz）里调用，内部不加临界区。
 */
class PanelKeys
{
public:
  /// @brief 全局唯一实例。避免出现两份按键状态各自消抖。
  static PanelKeys &GetInstance();

  /**
   * @brief 把 9 个引脚配成输入上拉，并绑定实例。重复调用无副作用。
   * @note 必须早于第一次 `Update()`。挂在 `CatchBlock::Start()` 里调一次。
   */
  void Init();

  /**
   * @brief 扫一遍全部按键：消抖 → 判边沿 → 判长按 → 执行动作。
   * @note 每拍调一次。纯 GPIO 读 + 计数器，不阻塞、不分配内存。
   */
  void Update();

  /**
   * @brief 9 个键**消抖后**的电平位图，bit i = 1 表示第 i 个键是松开的。
   * @note 给 `Publish()` 搬进 `catch_debug.keys_raw`。按下时对应位变 0。
   *       它是"消抖后"而不是原始电平 —— 所以看到翻转就说明抖动窗口也过了。
   */
  uint16_t RawBitmap() const;

  /// @brief 最近一次真正触发了动作的键索引 0~8，0xFF = 从没触发过。
  uint8_t LastKey() const;

private:
  PanelKeys() = default;
  PanelKeys(const PanelKeys &) = delete;
  PanelKeys &operator=(const PanelKeys &) = delete;

  /// @brief 按键索引。顺序必须与 cpp 里的 `kBindings` 一一对应。
  enum : uint8_t
  {
    KeyPump = 0,     ///< 吸盘 开/关（toggle）
    KeyValve,        ///< 夹爪 闭合/松开（toggle）
    KeyReconfig,     ///< `command = 1`，长按
    KeyArmOrigin,    ///< `command = 2`，长按
    KeyReset,        ///< `command = 5`，长按
    KeyLiftTop,      ///< `command = 11`，按下立即
    KeyDescend,      ///< `command = 12`，按下立即
    KeyEnable,       ///< `catch_motor_enable = 1`，长按
    KeyDisable,      ///< `catch_motor_enable = 0`，**按下立即**
    KeyCount
  };

  /// @brief 一个键的物理位置与触发方式
  struct Binding
  {
    char port;           ///< @brief 端口字母 'A' / 'C'
    uint8_t number;      ///< @brief 引脚号 0~15
    uint16_t hold_ticks; ///< @brief 按住多少拍才触发；0 = 按下沿立即触发
  };

  /// @brief 一个键的运行时状态
  struct KeyState
  {
    BSP::GPIO::Inst inst; ///< @brief 引脚实例（Init 里绑定）
    uint8_t raw_last = 1; ///< @brief 上一拍的原始电平，1 = 松开
    uint8_t stable = 1;   ///< @brief 消抖之后的电平，1 = 松开
    uint8_t same = 0;     ///< @brief 连续读到同一电平的拍数，用于消抖
    uint16_t held = 0;    ///< @brief 本次已按住多少拍，松开清零
    bool fired = false;   ///< @brief 本次按下是否已触发过，防长按重复触发
  };

  static const Binding kBindings[KeyCount];

  KeyState keys_[KeyCount];
  bool initialized_ = false;

  uint16_t raw_bitmap_ = 0xFFFF; ///< @brief 消抖后的电平位图，每拍刷新
  uint8_t last_key_ = 0xFF;      ///< @brief 最近一次触发的键索引，只由 Act() 写

  /// @brief 按下沿的处理：`hold_ticks == 0` 的键在这里就执行，其余等长按。
  void OnPressed(uint8_t idx);

  /// @brief 真正的动作分派。九个键各自做什么只有这一处。
  void Act(uint8_t idx);
};
