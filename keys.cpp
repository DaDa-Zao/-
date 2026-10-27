#include "keys.hpp"
#include "bsp_halport.hpp"
#include "catch_block.hpp"
#include "catch_block_driver.hpp"

namespace
{
/// @brief 按下后要连续读到多少次相同电平才认账。
/// @note App 拍是 5 ms，4 拍 = 20 ms。机械按键抖动通常 5~20 ms，这个窗口够覆盖；
///       再长会让操作手感变迟钝，再短会重复触发。
constexpr uint8_t kDebounceTicks = 4;

/**
 * @brief 端口字母 → HAL 端口指针。
 * @note  只列了本板实际用到的几个。换到别的端口要一起加时钟使能。
 */
GPIO_TypeDef *PortOf(char port)
{
  switch (port)
  {
    case 'A': return GPIOA;
    case 'C': return GPIOC;
    default:  return nullptr;
  }
}
}

const PanelKeys::Binding PanelKeys::kBindings[KeyCount] =
{
    // 端口, 引脚, 长按拍数（0 = 按下立即触发；5 ms 一拍，故 100 拍 = 0.5 s、400 拍 = 2 s）
    //
    // ⚠ PA0 在 STM32F427 上是 WKUP 引脚，不少开发板会在它上面挂一个"下拉 + 接 3.3V"的板载按键。
    //    如果这里配成上拉之后发现 PA0 **恒读到 0**（按键完全没反应，且 Watch 里 pump_on 自己来回翻），
    //    就是撞上了板载下拉。把这一行的引脚和别的键换一下即可，只需改这张表，Init() 会自动跟着变。
    {'A', 0, 0},   // KeyPump       吸盘 开/关
    {'A', 1, 0},   // KeyValve      夹爪 闭合/松开
    {'A', 2, 200}, // KeyReconfig   command = 1   长按 1.0 s
    {'A', 3, 200}, // KeyArmOrigin  command = 2   长按 1.0 s
    {'A', 4, 400}, // KeyReset      command = 5   长按 2.0 s（撤力，带载会掉块）
    {'A', 5, 0},   // KeyLiftTop    command = 11
    {'C', 0, 0},   // KeyDescend    command = 12
    {'C', 1, 100}, // KeyEnable     使能          长按 0.5 s
    {'C', 2, 0},   // KeyDisable    失能          按下立即（安全方向不做确认）
};

PanelKeys &PanelKeys::GetInstance()
{
  static PanelKeys instance;
  return instance;
}

/**
 * @brief 配 9 个引脚并绑定实例。
 * @note  逐个引脚配，而不是按端口批量配 —— 这样改上面那张表就够了，这里不用跟着改。
 *        只在上电跑一次，九次 HAL_GPIO_Init 的开销可以忽略。
 *
 *        **这里和 CubeMX 的 `MX_GPIO_Init()` 有一份重复配置，是故意的**：
 *        CubeMX 已经把这 9 个引脚配成上拉输入了，而本函数在它之后执行
 *        （`main()` 早于框架调度 `Start()`），所以真正生效的是这里这份。
 *        留着它自己配，是因为本函数**从 `kBindings` 表推导引脚** ——
 *        改表就能换引脚，配置自动跟着变；CubeMX 那份是手点的，改了表它不会同步。
 *        两边都填上拉时完全一致，不存在冲突。
 */
void PanelKeys::Init()
{
  if (initialized_)
  {
    return;
  }

  // 时钟在 MX_GPIO_Init() 里已经开过，这里重复开是幂等的，只是为了不依赖调用顺序。
  // 换到别的端口要在这里补对应时钟。
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  for (uint8_t i = 0; i < KeyCount; ++i)
  {
    GPIO_TypeDef *port = PortOf(kBindings[i].port);
    if (port == nullptr)
    {
      continue; // 表里写了不支持的端口，跳过而不是崩掉
    }

    GPIO_InitTypeDef gi{};
    gi.Pin = uint16_t(1u << kBindings[i].number);
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gi);

    KeyState &k = keys_[i];
    k.inst = BSP::GPIO::Inst(Pin{kBindings[i].port, kBindings[i].number});

    // 用**实际电平**初始化消抖状态，而不是假定"松开"。
    // 否则上电时如果手正压在面板上，会被当成一次"松开→按下"的边沿，
    // 一上电就执行一次动作 —— 其中就有撤力键。
    const uint8_t level = k.inst.Read() ? 1 : 0;
    k.raw_last = level;
    k.stable = level;
    k.same = kDebounceTicks;
    k.held = 0;
    k.fired = false;

    if (level != 0)
    {
      raw_bitmap_ |= uint16_t(1u << i);
    }
    else
    {
      raw_bitmap_ &= uint16_t(~(1u << i));
    }
  }
  initialized_ = true;
}

/**
 * @brief 扫一遍九个键。
 * @note  每拍全扫，没有中断、没有状态机。九次 GPIO 读 + 一点计数，代价可以忽略。
 */
void PanelKeys::Update()
{
  if (!initialized_)
  {
    return;
  }

  uint16_t bitmap = 0;
  for (uint8_t i = 0; i < KeyCount; ++i)
  {
    KeyState &k = keys_[i];
    const uint8_t level = k.inst.Read() ? 1 : 0;

    // ── 消抖：连续 kDebounceTicks 拍读到同一电平才认账 ──────────────────
    if (level == k.raw_last)
    {
      if (k.same < 255)
      {
        ++k.same;
      }
    }
    else
    {
      k.raw_last = level;
      k.same = 0; // 电平一变就重新数，抖动期间永远攒不满
    }

    if (k.same >= kDebounceTicks && k.stable != level)
    {
      k.stable = level;
      k.held = 0;
      k.fired = false;
      if (level == 0) // 松开→按下
      {
        OnPressed(i);
      }
    }

    // ── 长按：只对 hold_ticks > 0 的键有效，且每次按下只触发一次 ────────
    if (k.stable == 0 && !k.fired)
    {
      if (k.held < 0xFFFF)
      {
        ++k.held;
      }
      if (kBindings[i].hold_ticks > 0 && k.held >= kBindings[i].hold_ticks)
      {
        Act(i);
        k.fired = true; // 按住不放不会连续重复
      }
    }

    // 消抖后的电平拼成位图。用局部变量累加、循环结束一次性赋值，
    // 免得 Publish() 读到"只更新了一半"的组合。
    if (k.stable != 0)
    {
      bitmap |= uint16_t(1u << i);
    }
  }
  raw_bitmap_ = bitmap;
}

uint16_t PanelKeys::RawBitmap() const
{
  return raw_bitmap_;
}

uint8_t PanelKeys::LastKey() const
{
  return last_key_;
}

/**
 * @brief 按下沿处理。
 * @note  需要长按的键在这里什么都不做 —— `Update()` 数够了拍数才执行。
 *        这样"长按"和"短按"不会各触发一次。
 */
void PanelKeys::OnPressed(uint8_t idx)
{
  if (kBindings[idx].hold_ticks == 0)
  {
    Act(idx);
    keys_[idx].fired = true;
  }
}

/**
 * @brief 九个键各自做什么，全工程只有这一处。
 * @note  toggle 直接翻转权威变量，不另存状态：`catch_pump_on` / `catch_valve_closed`
 *        本身就是驱动层每拍写引脚的依据，另存一份必然会和 Keil 手改、自动流程不同步。
 */
void PanelKeys::Act(uint8_t idx)
{
  last_key_ = idx; // 先记"是哪个键触发的"，再看动作有没有被状态机接受

  switch (idx)
  {
    case KeyPump:
      catch_pump_on = catch_pump_on ? 0 : 1;
      break;
    case KeyValve:
      catch_valve_closed = catch_valve_closed ? 0 : 1;
      break;
    case KeyReconfig:
      catch_debug.command = 1; // 重配：要求已撤力，零点作废
      break;
    case KeyArmOrigin:
      catch_debug.command = 2; // 标零：要求机构静止在原点
      break;
    case KeyReset:
      catch_debug.command = 5; // 撤力 + 清锁存 + 退回未配置（须有支承）
      break;
    case KeyLiftTop:
      catch_debug.command = 11;
      break;
    case KeyDescend:
      catch_debug.command = 12;
      break;
    case KeyEnable:
      catch_motor_enable = 1; // 还要过 blockers 那一关，按了不等于真的使能
      break;
    case KeyDisable:
      catch_motor_enable = 0; // 下一拍 Update() 开头立即撤力
      break;
    default:
      break;
  }
}
