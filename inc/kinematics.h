#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

// ====================================================================
// 4x4 齐次变换矩阵库 (Homogeneous Transformation Matrix)
// @Context: Non-RealTime Background Thread
// @Safe: 纯数学运算，无副作用，无动态分配，无系统调用
// ====================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef double Mat4x4[4][4];

// m = I (单位矩阵)
void Mat4x4_Identity(Mat4x4 m);

// res = a * b (支持 res 与 a/b 别名，内部 temp 缓冲)
void Mat4x4_Multiply(Mat4x4 res, const Mat4x4 a, const Mat4x4 b);

// m = Translate(tx, ty, tz)
void Mat4x4_Translate(Mat4x4 m, double tx, double ty, double tz);

// m = RotateAxis(axis_id, angle_rad), axis_id: 0=X, 1=Y, 2=Z
void Mat4x4_RotateAxis(Mat4x4 m, int axis_id, double angle_rad);

// res = in^(-1) (利用正交旋转的转置特性: R^(-1)=R^T)
void Mat4x4_Inverse(Mat4x4 res, const Mat4x4 in);

// p_out = m * [p_in, 1]^T (齐次坐标 w=1, 支持 p_in==p_out 别名)
void Mat4x4_ApplyPoint(const Mat4x4 m, const double p_in[3], double p_out[3]);

// ====================================================================
// 通用五轴运动学引擎 — 支持 Head-Head / Table-Table / Mixed 三大构型
//
// 统一数学模型 (Matrix Concatenation):
//
//   T_head: 刀具侧运动链 (machine base → tool tip)
//   T_table: 工件侧运动链 (machine base → workpiece point)
//
//   统一逆解公式 (微分形式，保证零位 joint = tip):
//     delta_head = T_head * Origin  - T_head_0 * Origin
//     delta_table = T_table * tip   - T_table_0 * tip
//     joint = tip + delta_table - delta_head
//
//   HEAD_HEAD (双摆头: rotaries on tool side):
//     T_head = R1 * T_pivot * R2 * T_tool
//     T_table = I
//     joint = tip - delta_head
//
//   TABLE_TABLE (双转台: rotaries on workpiece side):
//     T_head = I
//     T_table = R1 * T_pivot * R2
//     joint = tip + delta_table
//
//   MIXED (一摆一转: rot_1 on head, rot_2 on table):
//     T_head = R1 * T_tool
//     T_table = R2 * T_pivot
//     joint = tip + delta_table - delta_head
// ====================================================================

typedef enum {
    KIN_NONE = 0,       // 无运动学变换 (joint = tip)
    KIN_HEAD_HEAD,      // 双摆头 (BC / AC)
    KIN_TABLE_TABLE,    // 双转台 (AC / BC)
    KIN_MIXED           // 混合型 (一摆一转)
} KinType_t;

typedef struct {
    KinType_t type;
    int rot_1_idx;          // 第 1 旋转轴底层索引 (轴数组下标)
    int rot_2_idx;          // 第 2 旋转轴底层索引
    int rot_1_axis;         // 第 1 旋转轴旋转维度: 0=X, 1=Y, 2=Z
    int rot_2_axis;         // 第 2 旋转轴旋转维度: 0=X, 1=Y, 2=Z
    double tool_offset[3];  // 主轴面到刀尖的偏置向量 (mm)
    double pivot_offset[3]; // 两旋转中心之间的物理偏置 (mm)
    // P1 (2026-09-04): 双摆头链序 — 0=R1 外层 (已交付默认, B外C内刀具同轴, C 对刀尖中性)
    //                   1=R2 外层 (典型 C外B内 腕)。同时参数化 Kinematics_Inverse 装配与
    //                   Kinematics_SolveOrientation 姿态解算。默认 0 保证已交付行为不变。
    int chain_order;
} KinConfig_t;

// 全局运动学配置 (由 SMC_ConfigKinematics 初始化)
extern KinConfig_t g_kin_config;

// 通用逆运动学: 刀尖坐标 → 物理机床坐标
void Kinematics_Inverse(double tip_xyz[3], double rot_a, double rot_b, double rot_c, double joint_xyz[3]);

// ================== P1 (2026-09-04): G43.5 刀轴矢量姿态解算 ==================
typedef enum {
    KIN_ORIENT_OK          = 0,   // 解算成功 (角度个数看 n_solved)
    KIN_ORIENT_ERR_UNREACH = -1,  // 矢量在本构型下不可达 (chain0: u 不在可达平面内)
    KIN_ORIENT_ERR_CONFIG  = -2   // 构型不支持矢量解算 (非 HEAD_HEAD / 轴向组合 / 刀具不同轴)
} KinOrientStatus_t;

// 刀轴单位矢量 u (机床系) → 旋转角 (度)。
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 (sqrt/atan2/acos 可重入), 全程局部变量, 无静态状态。
// u_in: 任意非零缩放 (内部归一化); cur_rot*: 当前两旋转角 (度, 双解近者选择基准)。
// n_solved: 1=只解出 rot_1 (rot_2 不定保持现值 — chain0 全部姿态 + chain1 竖直奇异);
//           2=双轴全解 (chain1 常规)。out_rot2 在 n_solved==1 时不写。
int Kinematics_SolveOrientation(const double u_in[3], double cur_rot1_deg, double cur_rot2_deg,
                                double *out_rot1_deg, double *out_rot2_deg, int *n_solved);

// ================== P1 (2026-09-07): G68.2/G68.3 倾斜面构帧 ==================
// 行主序 3x3 旋转: R_out[3*row+col]。列向量 = [u v n] (倾斜面 X/Y/Z 轴在机床系的像)。
// 正变换: machine_xyz = R·p_xyz; 反变换: p = Rᵀ·(machine − offset)。

// G68.2 构帧: n = normalize(IJK) 为倾斜面法向 (=刀轴目标方向), Q = 绕法向旋转 (度)。
// a = 与 n 最不平行的参考轴, u0 = normalize(a×n), u = Rodrigues(u0, n, Q), v = n×u。
// @Context: Non-RealTime Background Thread (parser 调用) @Safe: 纯数学可重入。
// @return 0=成功; -1=零法向
int Kinematics_BuildTiltFrame(const double n_in[3], double q_deg, double R_out[9]);

// G68.3 固定角 (extrinsic X→Y→Z): R = Rz(az)·Ry(ay)·Rx(ax)，对齐 Fanuc 30i G68.3 语义。
// @Context: Non-RealTime Background Thread (parser 调用) @Safe: 纯数学可重入
void Kinematics_BuildEulerXYZ(double ax_deg, double ay_deg, double az_deg, double R_out[9]);

#endif // KINEMATICS_H
