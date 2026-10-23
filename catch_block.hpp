#pragma once
#include "System.hpp"
#include "bsp_gpio.hpp"

/**
 * @file catch_block.hpp
 * @brief 取块机构（夹取 → 提升 → 顶端保持 → 主动投放 → 受控回程）的业务层。
 *
 * 坐标与单位：内部位置/速度一律用"转子累计角度 rad"；对外高度用米，
 *             换算系数约 2.6037 mm/rad（0.025 m 带轮 × 2 ÷ 19.2032 减速比）。
 * 运行上下文：Update() 在 App 任务（名义 5 ms）跑状态机与继电器；
 *             位置环在 Control 任务（1 kHz）由 MotorDJI::Control() 执行。
 * 安全边界：没有失电保持、没有夹取/位置传感器。故障动作是"撤驱动"而非
 *             "安全停止"，必须依赖外部支承与防坠。
 */

// Keil Watch 可直接改写：写非 0 为开，写 0 为关。写 0 是"撤力/松开"，不是刹车。
extern "C" volatile uint8_t catch_pump_on;      // PE6 气泵：非 0 = 吸附
extern "C" volatile uint8_t catch_valve_closed; // PE12 电磁阀：非 0 = 夹爪闭合
extern "C" volatile uint8_t catch_motor_enable; // 非 0 = 允许输出，仍受配置/零点/健康检查门控

// Keil 调试通道：每拍最多消费一条，走与 Submit() 相同的一条状态机路径。
// 用法：先填 catch_debug_config / catch_debug_height_m，最后写 catch_debug_command。
// 命令：1=配置 2=当前位置标零 3=移动到高度 4=保持当前位置。
// 结果：0=未执行 1=已接受 2=被拒绝 3=执行后处于故障。
extern "C" volatile uint8_t catch_debug_command;
extern "C" volatile uint8_t catch_debug_result;
extern "C" volatile float catch_debug_height_m;        // 命令 3 的目标高度 (m)
extern "C" volatile float catch_debug_actual_height_m; // 实际高度回读 (m)
extern "C" volatile uint32_t catch_debug_faults;       // 应用故障 | 驱动故障
extern "C" volatile uint8_t catch_debug_state;         // State 枚举值
extern "C" volatile uint8_t catch_debug_feedback_valid; // 1 = 反馈有效
extern "C" volatile uint8_t catch_debug_motor_enabled; // 1 = 驱动正在输出

// 仅用于有外部防坠保护的调试；任何电控指令都不能保证失电防坠。
class CatchBlock : public Application
{
    // 单例，由框架注册后按应用周期调用。
    SINGLETON(CatchBlock) : Application("CatchBlock")
    {
    }
    APPLICATION_OVERRIDE
public:
    // 流程状态。除 Unconfigured / Idle / FaultHold / Fault 外，其余状态都允许电机输出。
    enum class State : uint8_t
    {
        Unconfigured, // 未配置：只有它能接受 Configure
        Idle,         // 已配置、未标零：等待 ArmAtSupportedOrigin
        OriginHold,   // 原点保持：可关夹爪、可点动
        GripClose,    // 夹爪闭合动作中，等 relay_dwell_ms
        SuctionOn,    // 气泵建立中，等 dwell 且两路标志为高
        GripVerify,   // 等待操作员确认夹取成功
        Lifting,      // 位置环上升中
        TargetHold,   // 顶端保持：等操作员许可投放
        ReleaseReady, // 许可已给：再校验一拍后进入 Release
        Release,      // 已允许投放：等确认物块离开
        Descending,   // 受控下降回原点
        ManualMoving, // 调试位移中（命令 3）
        ManualHold,   // 调试保持中（命令 4）
        FaultHold,    // 故障但仍在保持位置，仅原先已使能时有效
        Fault         // 故障且已断驱动，必须外部支承
    };
    // 人工命令。除 None 外每条只做一件事，经单槽邮箱进入应用任务。
    enum class Command : uint8_t
    {
        None,                        // 空命令，Submit() 直接拒绝
        Configure,                   // 下发参数并初始化驱动（须禁用）
        ArmAtSupportedOrigin,        // 在有支承的原点采零点并设软限位
        CloseGrip,                   // 原点稳定后进入夹取流程
        ConfirmGripAndLift,          // 人工确认夹取后上升到 travel_m
        PermitDrop,                  // 人工许可投放（须顶端稳定）
        ConfirmObjectClearAndReturn, // 人工确认物块已离开后回原点
        Hold,                        // 软急停：冻结动作，保持当前位置
        ResetWhileSupported,         // 有支承时清故障与反馈参考，回到未配置
        MoveToHeight,                // 调试：移动到 target_height_m
        HoldCurrent                  // 调试：以当前位置为保持目标
    };
    // 故障位，可叠加；锁存后不自动恢复。
    enum Fault : uint32_t
    {
        NoFault = 0,
        Configuration = 1,  // 参数非法或驱动初始化失败
        Feedback = 2,       // 反馈无效、超时或跳变
        Controller = 4,     // 控制器判故障（数值/状态不一致）
        Timing = 8,         // 应用周期超出 2~15 ms
        Timeout = 16,       // 阶段等待超时
        Travel = 32,        // 高度或目标越界
        CanTransmit = 64,   // 上次 CAN 发送结果非 Ok
        OperatorHold = 128  // 操作员请求的 Hold
    };
    // 全部可调参数。数值为 0 的必填项会被 Validate() 拒绝。
    struct Config
    {
        uint8_t esc_id = 1;              // C620 编号 1..8，反馈 ID = 0x200+id（CAN1 应为 0x201）
        int8_t lift_encoder_sign = 0;    // ±1：高度上升时编码器累计值的方向
        int8_t current_encoder_sign = 0; // ±1：正 CAN 电流使转子加速的方向
        bool direction_verified = false;      // 两个方向都已实测
        bool model_verified = false;          // J / Kt 已确认或辨识
        bool current_limits_verified = false; // 电流上限与斜率已按温升、机械确认
        bool motion_limits_verified = false;  // 速度/加速度/超速门限已实测
        float inertia_kg_m2 = 0;         // kg·m²，折算到转子轴的整机惯量
        float torque_constant_Nm_A = 0;  // N·m/A，转子域（手册 0.3 是输出轴值）
        float viscous_Nm_s_rad = 0;      // N·m·s/rad，0 表示交给 ESO 吸收
        float max_current_A = 0;         // A，必须大于静态承载电流
        float current_slew_A_s = 0;      // A/s，太小托不住，太大冲击
        float observer_Hz = 0;           // ESO 带宽，须大于 controller_Hz
        float controller_Hz = 0;         // 位置环带宽
        float travel_m = 0.395f;         // m，原点至顶端，仍须核对两端余量
        float pulley_radius_m = 0.025f;  // m，同步带轮有效节圆半径
        float external_speed_ratio = 2.0f; // 大/小齿轮转速比（带轮是输出的 2 倍）
        float gearbox_ratio = 3591.0f / 187.0f; // 减速箱比，来自 M3508 手册
        float speed_m_s = 0.020f;        // m/s，TD 规划限速（候选值）
        float acceleration_m_s2 = 0.040f;// m/s²，TD 规划限加速度（候选值）
        float overspeed_m_s = 0.030f;    // m/s，超速门限，应留 1.2~1.5 倍余量
        float position_tolerance_m = 0.001f;   // m，到位判定
        float settled_speed_m_s = 0.001f;      // m/s，到位判定的速度门限
        float travel_fault_margin_m = 0.003f;  // m，软限位与越界判定的余量
        uint32_t feedback_timeout_us = 10000;  // µs，反馈超时
        uint32_t settle_ms = 500;        // ms，到位需持续稳定的时间
        uint32_t relay_dwell_ms = 300;   // ms，动作间隔，不证明动作成功
        uint32_t phase_timeout_ms = 45000; // ms，单阶段超时
    };
    // 一次操作请求：命令 + 该命令需要的人工确认 + 参数。
    struct Request
    {
        Command command = Command::None;
        // 每次操作都要明确确认，不能仅凭等待时间推定安全条件成立。
        bool external_protection_confirmed = false; // 外部防坠/支承已就位
        bool supported_origin_confirmed = false;    // 机构当前确实被支承
        bool grip_confirmed = false;                // 人工确认已夹紧/吸牢
        bool drop_zone_clear_confirmed = false;     // 投放区已隔离
        bool object_clear_confirmed = false;        // 物块已离开
        float target_height_m = 0;                  // Command::MoveToHeight 的目标 (m)
        Config config;                              // 仅 Configure 使用
    };
    // 供上位机与 Keil 读取的快照，由 Publish() 整体替换。
    struct Status
    {
        State state = State::Unconfigured;
        uint32_t faults = 0, motor_faults = 0, rejected_requests = 0; // 应用故障 / 驱动故障 / 拒绝数
        float height_m = 0, velocity_m_s = 0; // 相对原点的高度与速度
        float target_position_rad = 0, measured_position_rad = 0; // 目标/实测转子角
        float reference_position_rad = 0, reference_velocity_rad_s = 0; // TD 参考轨迹
        float eso_position_rad = 0, eso_velocity_rad_s = 0, disturbance_rad_s2 = 0; // ESO z1/z2/z3
        float raw_current_A = 0, limited_current_A = 0, final_current_A = 0; // 原始/限幅/下发
        float accepted_command_A = 0;   // CAN 邮箱已接受，不是实测电流
        int16_t feedback_current_raw = 0; // 反馈原始电流码，未标定
        bool feedback_current_A_valid = false; // false：raw 不能当安培用
        bool feedback_valid = false;    // 反馈已建立且无故障
        uint8_t temperature_C = 0;      // 电调温度
        bool pump_on = false, valve_closed = false; // 两路引脚实际电平
        bool motor_enabled = false;     // 驱动是否在输出
    };
    // 单槽邮箱：false = 已有待处理命令或命令为 None，不代表动作失败。
    bool Submit(const Request& request);
    // 调试入口：提交一条 Configure，自动带上"有支承/有外部保护"确认。
    bool ConfigureForTest(const Config& config);
    // 调试入口：把当前静止位置设为 0 m，其余检查仍走 ArmAtOrigin()。
    bool SetOriginForTest();
    // 调试入口：移动到相对原点的高度 (m)，范围 0..travel_m。
    bool MoveToHeight(float height_m);
    // 调试入口：以当前反馈位置为保持目标，退出调试位移。
    bool HoldPosition();
    // 关中断拷贝一份快照，任意任务可调用。
    Status GetSnapshot() const;
    // 未实现也无调用点：位置环在 ControlAllMotors() 中运行，应用层不直接驱动。
    void ControlTick();
    void AfterControlTick();
private:
    MotorDJI motor_;  // 电机控制层：驱动 + ADRC + 电流链路
    // PE6 气泵继电器 / PE12 电磁阀继电器
    BSP::GPIO::Inst pump_{Pin{'E',6}}, valve_{Pin{'E',12}};

    Config config_;               // 当前生效的参数
    Request pending_;             // 邮箱中的待处理请求
    bool pending_valid_ = false, started_ = false, configured_ = false; // 邮箱非空 / 已启动 / 已配置
    bool origin_valid_ = false, protection_ack_ = false; // 零点有效 / 本次已确认外部保护
    State state_ = State::Unconfigured;

    uint32_t faults_ = 0, rejected_ = 0, phase_start_ms_ = 0; // 应用故障 / 拒绝数 / 阶段起点

    uint64_t previous_tick_us_ = 0, settled_since_us_ = 0; // 上一拍时刻 / 到位起始时刻

    float origin_rad_ = 0, metres_per_rotor_rad_ = 0; // 零点转子角 / 高度换算系数(约2.6037mm/rad)
    float height_m_ = 0, velocity_m_s_ = 0;          // 当前高度 / 高度方向速度
    float manual_target_m_ = 0;      // 手动位移的目标高度

    Status status_; // 对外发布的快照
    bool Validate(const Config& config) const;             // 参数总检查
    bool Configure(const Request& request, uint32_t now_ms);   // 初始化驱动与控制器
    bool ArmAtOrigin(const Request& request, uint32_t now_ms); // 采零点、设软限位
    void Process(const Request& request, uint32_t now_ms);     // 命令分发与状态转换
    void Transition(State state, uint32_t now_ms);             // 切状态并复位本阶段计时
    void Fail(uint32_t fault, bool can_hold, uint32_t now_ms); // 锁存故障并选择保持或断驱动
    bool SetHeight(float metres);                          // 高度(m) → 转子目标(rad)
    bool Settled(float target_m, uint64_t now_us);         // 到位且稳定判定
    void CheckMotion(uint64_t now_us, uint32_t now_ms, uint64_t elapsed_us); // 运行中检查
    void ReadFeedback();                                   // 反馈 → height/velocity
    void ConsumeRequest(uint32_t now_ms);                  // 取出邮箱命令
    void ConsumeDebugCommand(uint32_t now_ms);             // 取出 Keil 命令
    void Advance(uint64_t now_us, uint32_t now_ms);        // 自动推进与超时
    void ApplyDebugOutputs();                              // 写继电器、切换使能
    void Publish();                                        // 汇总快照与调试镜像
};

// Keil 里逐项填写的参数副本；命令 1 时整体拷进 Request。
extern CatchBlock::Config catch_debug_config;
