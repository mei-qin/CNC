#ifndef AXIS_CFG_H
#define AXIS_CFG_H

#include <stdint.h>
#include <string.h>
#include <stdatomic.h>
typedef int32_t int32;
typedef uint16_t uint16;
/************************ 核心宏定义 ************************/
#define AXIS_NUM         5      // 五轴核心宏！改此值可灵活增减轴数
#define EC_TIMEOUTMON    500     // SOEM超时时间
#define NSEC_PER_SEC     1000000000
#define CYCLE_TIME_NS    1000000  // 1ms实时周期，五轴共用

/************************ CiA402 控制字（台达B3-E标准） ************************/
#define CW_FAULT_RESET   0x0080
#define CW_SHUTDOWN      0x0006
#define CW_SWITCH_ON     0x0007
#define CW_ENABLE_OP     0x000F
#define CW_PP_TRIGGER    0x001F   // bit4上升沿触发
#define CW_CSP_ENABLE    0x004F 

/************************ CiA402 状态字（掩码+标准状态） ************************/
#define SW_MASK          0x006F   // CiA402状态字有效位掩码
#define SW_FAULT_CLEAR   0x0000   // 故障位(bit3)清零
#define SW_SHUTDOWN_RDY  0x0021   // 关机就绪
#define SW_SWITCHED_ON   0x0023   // 伺服上电完成
#define SW_OP_ENABLED    0x0027   // 操作使能就绪
#define SW_TARGET_REACH  0x0400   // 目标位置到达

/************************ PDO 字节偏移（台达B3-E标准，所有轴统一） ************************/
#define PDO_CW_BYTE0     0
#define PDO_CW_BYTE1     1
#define PDO_POS_BYTE0    2
#define PDO_POS_BYTE1    3
#define PDO_POS_BYTE2    4
#define PDO_POS_BYTE3    5
#define PDO_SW_BYTE0     0
#define PDO_SW_BYTE1     1
#define PDO_FOLLOW_BYTE0 6   // 0x60F4 跟随误差（32位，小端）
#define PDO_FOLLOW_BYTE1 7
#define PDO_FOLLOW_BYTE2 8
#define PDO_FOLLOW_BYTE3 9

// 补全所有未定义的对象字典标识符
#define OD6060_MODE_OF_OPERATION 0x6060  // 操作模式
#define OD607A_TARGET_POSITION   0x607A  // 目标位置（32位有符号）
#define OD6081_MAX_SPEED         0x6081  // 最大速度（32位无符号）
#define OD6083_ACCELERATION      0x6083  // 加速时间（32位无符号）
#define OD6084_DECELERATION      0x6084  // 减速时间（32位无符号）
#define OD6040_CONTROL_WORD      0x6040  // 控制字（备用）
#define OD6041_STATUS_WORD       0x6041  // 状态字（备用）
#define OD60FF_TARGET_SPEED      0x60FF  // 目标速度（32位有符号）


// 状态字位掩码
#define SW_READY_FUNC_START  0x0001  // bit0：准备功能启动
#define SW_SERVO_READY       0x0002  // bit1：伺服准备完成
#define SW_SERVO_ENABLE      0x0004  // bit2：伺服使能
#define SW_ERROR             0x0008  // bit3：异常信号
#define SW_MAIN_POWER_ON     0x0010  // bit4：入力侧供电
#define SW_EMERGENCY_STOP    0x0020  // bit5：紧急停止
#define SW_READY_FUNC_OFF    0x0040  // bit6：准备功能关闭
#define SW_WARNING           0x0080  // bit7：警告信号
#define SW_REMOTE_CTRL       0x0200  // bit9：远程控制
#define SW_TARGET_REACH      0x0400  // bit10：目标到达

// 核心状态组合宏（便于上层判断）
#define SW_IS_NORMAL         ((SW_ERROR == 0) && (SW_EMERGENCY_STOP == 0)) // 无异常无急停
#define SW_IS_ENABLED        (SW_SERVO_ENABLE == 1)                        // 伺服使能
#define SW_IS_TARGET_REACH   (SW_TARGET_REACH == 1)                        // 目标到达

// ========== CSP模式相关宏 ==========
// CiA402模式值
#define CSP_MODE           0x08    // CSP模式（循环同步位置）
#define PP_MODE            0x01    // PP模式（点位，原有）

// CSP轨迹生成参数
#define CSP_AMPLITUDE      10000   // 轨迹幅值（脉冲）
#define CSP_FREQUENCY      1.0f    // 轨迹频率（Hz）
#define CSP_OFFSET         0       // 轨迹偏移（脉冲）

//#define LOGICAL_AXIS_NUM 5
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define AXIS_A 3
#define AXIS_B 4
#define AXIS_ALL -1

#define MAX_SLAVES_PER_AXIS 2
#define MIN_FEED_SPEED  0.5   // 合成速度下限（mm/s），低于此值钳制
#define MIN_FEED_ACC   10.0   // 合成加速度下限（mm/s^2），防止龟速蠕动
#define MIN_FEED_DEC   10.0   // 合成减速度下限（mm/s^2）
#define QUEUE_SIZE 1024 // 命令队列大小

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG_TO_RAD (M_PI / 180.0)

/************************ 命令类型宏 ************************/
#define CMD_TYPE_MOTION  0   // 运动段（G00/G01/G02/G03）
#define CMD_TYPE_MCODE   1   // M代码段（辅助功能）

#define MCODE_WAIT_TIMEOUT_MS  5000  // M代码等待绝对超时（ms），防止队列死锁

/************************ 跟随误差监控参数 ************************/
#define FOLLOW_ERR_MAX_PULSE     5000   // 跟随误差硬限（脉冲），超过立即停机
#define FOLLOW_ERR_WARN_PULSE    3000   // 跟随误差警告阈值（脉冲）
#define FOLLOW_ERR_WARN_TIME_MS  200    // 警告阈值持续时长（ms），超过则停机

/************************ 实时线程环形日志缓冲 ************************/
#define RT_LOG_BUF_SIZE  64
#define RT_LOG_MSG_LEN   128

typedef struct {
    char buffer[RT_LOG_BUF_SIZE][RT_LOG_MSG_LEN];
    volatile int head;
    volatile int tail;
} RtLog_t;


/************************ 轴参数结构体（单轴所有参数独立封装） ************************/
// 每个轴的独立配置/状态，五轴通过数组管理
typedef struct {
    // ① 静态配置（初始化赋值，运行中不变）
    int slave_id; 
    int slave_ids[MAX_SLAVES_PER_AXIS];  // 关联的从站ID数组（支持多从站）
    int slave_count;      // 有效的从站数量（<= MAX_SLAVES_PER_AXIS）
    char axis_name[16];    // 轴名（例如 "X","Y","Z","A","B"）
    int32 pp_target_pos;   // PP模式目标位置（脉冲/PUU，32位有符号）
    int32 csp_base_pos;    // CSP模式的基准位置（用于相对轨迹计算）
    float csp_speed;       // CSP模式速度，单位为 脉冲/ms（浮点）
   


    // ② 动态状态（运行时实时更新，每个轴独立）
    int cia_step;          // CiA402状态机步骤（0-6）
    int cia_step_delay;    // 步骤延时计数器
    int pp_trigger_sent;   // PP触发标志（0=未触发，1=已触发）
    int is_op_ready;       // 操作使能就绪（0=否，1=是）
    int is_target_reach;   // 目标到达（0=否，1=是）
    int is_error;          // 故障标志（0=无错，1=故障）
    int state;
    int is_enabled;
    int32_t target_pos;    // 目标位置（脉冲），由上层命令或轨迹生成器设置
    int32_t actual_pos;    // 实际位置（脉冲），从驱动器或编码器反馈

    int32_t home_offset[MAX_SLAVES_PER_AXIS]; // 归零/原点偏移（每个从站）
    double current_cmd_pos; // 当前命令位置（以工程单位表示，用于UI/控制）
    double pulse_per_unit; // 脉冲/单位（如 脉冲/mm 或 脉冲/度），用于位置/速度换算

    // ④ 轴动力学参数（由 SMC_ConfigAxisDynamics 配置）
    int axis_type;        // 0: 线性轴 (Linear), 1: 旋转轴 (Rotary)
    double max_speed;     // 单轴最大允许速度 (mm/s 或 deg/s)
    double max_acc;       // 单轴最大允许加速度 (mm/s^2 或 deg/s^2)
    double max_dec;       // 单轴最大允许减速度 (mm/s^2 或 deg/s^2)
    double equivalent_radius; // 旋转轴的物理半径，单位: mm (用于弧长换算: mm = deg * (PI/180) * radius)

    // ③  软件限位参数（可选，视驱动器支持情况而定）
    int enable_soft_limit;   // 软件限位使能（0=否，1=是）
    double soft_limit_pos;  // 软件限位位置- 正向限位
    double soft_limit_neg;  // 软件限位位置- 负向限位

    //
    int enable_sync_alarm;    // 同步报警使能（0=否，1=是）
    int32_t sync_tolerance_pulse; // 同步容差（脉冲），用于判断同步异常
    int32_t sync_max_err_pulse;   // 同步最大误差（脉冲），超过则判定为同步故障
    int sync_err_time_ms;       // 同步误差持续时间（ms），超过则判定为同步故障
    int _current_sync_timer;    // 同步误差计时器（ms），用于跟踪同步异常持续时间

    int32_t _follow_err_timer;  // 跟随误差警告持续时间计时器（ms）
    
} AxisCtrl_t;

/************************ 规划器全局参数 ************************/
typedef struct {
    double corner_tolerance;      // G64 拐角容差 (mm)
    double max_centripetal_acc;   // G64 最大向心加速度 (mm/s^2)
} PlannerConfig_t;

extern PlannerConfig_t g_planner_config;


typedef enum{
    HOLD_NORMAL=0,
    HOLD_BRAKING,
    HOLD_PAUSED,
    HOLD_RESUMING //

}FeedHoldState_t;

typedef struct{
    int is_moving;

    double start_pos[AXIS_NUM];
    double target_pos[AXIS_NUM];
    double dir_vec[AXIS_NUM];

    double current_pos[AXIS_NUM];
    double v_max,v_start,v_end;

    int32_t total_time_ms;
    int32_t current_time_ms;
    double virtual_time_ms;
    double time_scale;
    FeedHoldState_t hold_state;
    int pause_request;
    _Atomic int alarm_reset_request;

    double total_distance;
    int32_t t_acc;
    int32_t t_dec;
    int32_t t_cru;
    //int32_t t_total;

    double v_target;
    double acc;
    double dec;

    int is_waiting_mcode;       // M代码等待屏障标志
    int32_t mcode_wait_timer;   // M代码非阻塞延时计数器（ms）
    int current_mcode;          // 当前等待中的M代码编号

}Interpolator_t;

/*
 * Interpolator_t 字段说明（补充，便于维护）
 * - is_moving: 是否处于运动中
 * - start_pos/target_pos/current_pos: 各轴的起始/目标/当前位置（工程单位）
 * - dir_vec: 运动方向单位向量
 * - v_max/v_start/v_end: 速度上限与起始/结束速度
 * - total_time_ms/current_time_ms: 轨迹总时长与已运行时间（ms）
 * - t_acc/t_dec/t_cru: 加速/减速/巡航时间（ms）
 * - v_target/acc/dec: 目标速度与加/减速
 */

typedef struct{
    double target_pos[AXIS_NUM];
    _Atomic int is_ready;
    int32_t speed;

    int cmd_type;       // CMD_TYPE_MOTION 或 CMD_TYPE_MCODE
    int m_code;         // M代码编号（如 3=M03, 5=M05）
    double s_value;     // S值（如主轴转速）

    double total_distance;
    double dir_vec[AXIS_NUM];
    //double dir_x,dir_y,dir_z; // 运动方向单位向量
    
    double v_max;
    double v_start;
    double v_end;

    int32_t t_acc;
    int32_t t_dec;
    int32_t t_cru;
    int32_t t_total;

    double v_target;
    double acc;
    double dec;
}TrajectorySegment_t;


/* CommandQueue_t 说明:
 * - buffer: 轨迹段环形缓冲
 * - head: 下一个可读位置索引
 * - tail: 下一个可写位置索引
 */

typedef struct{
    TrajectorySegment_t buffer[QUEUE_SIZE];
    volatile int head;
    volatile int tail;
    
}CommandQueue_t;

typedef enum{

    COORD_G53=0, 
    COORD_G54=1,
    COORD_G55=2,
    COORD_G56=3,
    COORD_G57=4,
    COORD_G58=5,
    COORD_G59=6,

}CoordSystem_t;

typedef struct{

    CoordSystem_t current_coord;   // 当前坐标系
    double work_offsets[6][AXIS_NUM]; // 各坐标系的工件坐标偏移（mm或度），例如 word_offsets[1] 就是 G54 的偏移
    double current_g53_pos[AXIS_NUM]; // 当前G53坐标位置（机床坐标），实时更新用于UI显示和坐标转换
    double current_logical_pos[AXIS_NUM];// 当前逻辑坐标位置（相对于当前坐标系），实时更新用于UI显示和控制

}CoordManager_t;

#endif // AXIS_CFG_H
