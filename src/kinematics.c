// ====================================================================
// 通用五轴运动学引擎 — 4x4 齐次变换矩阵库 + 三构型动态逆解
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学运算 (sin/cos 可重入)，无副作用，无动态分配
// ====================================================================

#include "kinematics.h"
#include <string.h>

#define DEG2RAD (M_PI / 180.0)

// 全局运动学配置 (默认 KIN_NONE: joint = tip, 无变换)
KinConfig_t g_kin_config = {
    KIN_NONE,
    -1, -1,             // rot_1_idx, rot_2_idx (未配置)
    0, 0,               // rot_1_axis, rot_2_axis
    {0.0, 0.0, 0.0},    // tool_offset
    {0.0, 0.0, 0.0}     // pivot_offset
};

// ====================================================================
// 4x4 齐次变换矩阵基础运算
// ====================================================================

void Mat4x4_Identity(Mat4x4 m)
{
    memset(m, 0, sizeof(Mat4x4));
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
    m[3][3] = 1.0;
}

void Mat4x4_Multiply(Mat4x4 res, const Mat4x4 a, const Mat4x4 b)
{
    double tmp[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double s = 0.0;
            for (int k = 0; k < 4; k++)
                s += a[i][k] * b[k][j];
            tmp[i][j] = s;
        }
    }
    memcpy(res, tmp, sizeof(Mat4x4));
}

void Mat4x4_Translate(Mat4x4 m, double tx, double ty, double tz)
{
    Mat4x4_Identity(m);
    m[0][3] = tx;
    m[1][3] = ty;
    m[2][3] = tz;
}

void Mat4x4_RotateAxis(Mat4x4 m, int axis_id, double angle_rad)
{
    double c = cos(angle_rad), s = sin(angle_rad);
    Mat4x4_Identity(m);
    switch (axis_id) {
        case 0: // X
            m[1][1] =  c; m[1][2] = -s;
            m[2][1] =  s; m[2][2] =  c;
            break;
        case 1: // Y
            m[0][0] =  c; m[0][2] =  s;
            m[2][0] = -s; m[2][2] =  c;
            break;
        case 2: // Z
            m[0][0] =  c; m[0][1] = -s;
            m[1][0] =  s; m[1][1] =  c;
            break;
    }
}

void Mat4x4_Inverse(Mat4x4 res, const Mat4x4 in)
{
    // 齐次变换 M = [R|t; 0|1], M^(-1) = [R^T | -R^T*t; 0 | 1]
    double tmp[4][4];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            tmp[i][j] = in[j][i]; // R^T

    for (int i = 0; i < 3; i++) {
        tmp[i][3] = 0.0;
        for (int j = 0; j < 3; j++)
            tmp[i][3] -= tmp[i][j] * in[j][3]; // -R^T * t
    }
    tmp[3][0] = 0.0; tmp[3][1] = 0.0; tmp[3][2] = 0.0; tmp[3][3] = 1.0;
    memcpy(res, tmp, sizeof(Mat4x4));
}

void Mat4x4_ApplyPoint(const Mat4x4 m, const double p_in[3], double p_out[3])
{
    // 强制转换为齐次坐标 [x, y, z, 1.0] 参与运算
    double x = p_in[0], y = p_in[1], z = p_in[2], w = 1.0;
    double rx = m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3]*w;
    double ry = m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3]*w;
    double rz = m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3]*w;
    // 纯刚体运动无需透视除法
    p_out[0] = rx;
    p_out[1] = ry;
    p_out[2] = rz;
}

// ====================================================================
// 内部工具：根据轴索引查表获取旋转角度
// g_axis_map['A'-'A']==idx → rot_a, 'B'→rot_b, 'C'→rot_c
// ====================================================================

extern int g_axis_map[26];

static double get_angle_for_idx(int axis_idx, double rot_a, double rot_b, double rot_c)
{
    if (axis_idx < 0) return 0.0;
    if (g_axis_map['A' - 'A'] == axis_idx) return rot_a;
    if (g_axis_map['B' - 'A'] == axis_idx) return rot_b;
    if (g_axis_map['C' - 'A'] == axis_idx) return rot_c;
    return 0.0;
}

// ====================================================================
// 通用逆运动学 — 基于构型的动态矩阵装配
//
// 统一公式 (微分形式):
//   delta_head  = T_head(θ) * Origin  - T_head(0) * Origin
//   delta_table = T_table(θ) * tip    - T_table(0) * tip
//   joint = tip + delta_table - delta_head
//
// 确保 θ=0 时 delta=0 → joint=tip (零位一致)
// ====================================================================

void Kinematics_Inverse(double tip_xyz[3], double rot_a, double rot_b, double rot_c, double joint_xyz[3])
{
    KinConfig_t *cfg = &g_kin_config;

    // KIN_NONE: 直通 (joint = tip)
    if (cfg->type == KIN_NONE || cfg->rot_1_idx < 0 || cfg->rot_2_idx < 0) {
        joint_xyz[0] = tip_xyz[0];
        joint_xyz[1] = tip_xyz[1];
        joint_xyz[2] = tip_xyz[2];
        return;
    }

    // 获取两旋转轴的角度 (度 → 弧度)
    double angle1 = get_angle_for_idx(cfg->rot_1_idx, rot_a, rot_b, rot_c) * DEG2RAD;
    double angle2 = get_angle_for_idx(cfg->rot_2_idx, rot_a, rot_b, rot_c) * DEG2RAD;

    // 构建旋转矩阵 R1, R2
    Mat4x4 R1, R2;
    Mat4x4_RotateAxis(R1, cfg->rot_1_axis, angle1);
    Mat4x4_RotateAxis(R2, cfg->rot_2_axis, angle2);

    // 构建偏置矩阵
    Mat4x4 T_tool;
    Mat4x4_Translate(T_tool, -cfg->tool_offset[0], -cfg->tool_offset[1], -cfg->tool_offset[2]);

    Mat4x4 T_pivot;
    Mat4x4_Translate(T_pivot, cfg->pivot_offset[0], cfg->pivot_offset[1], cfg->pivot_offset[2]);

    // ---- 根据构型动态装配 T_head 和 T_table ----
    Mat4x4 T_head, T_table;
    Mat4x4_Identity(T_head);
    Mat4x4_Identity(T_table);

    // 临时矩阵用于连乘
    Mat4x4 tmp_a, tmp_b;

    switch (cfg->type) {

    case KIN_HEAD_HEAD:
        // T_head = R1 * T_pivot * R2 * T_tool
        Mat4x4_Multiply(tmp_a, R2, T_tool);        // R2 * T_tool
        Mat4x4_Multiply(tmp_b, T_pivot, tmp_a);     // T_pivot * R2 * T_tool
        Mat4x4_Multiply(T_head, R1, tmp_b);         // R1 * T_pivot * R2 * T_tool
        break;

    case KIN_TABLE_TABLE:
        // T_table = R1 * T_pivot * R2
        Mat4x4_Multiply(tmp_a, T_pivot, R2);        // T_pivot * R2
        Mat4x4_Multiply(T_table, R1, tmp_a);        // R1 * T_pivot * R2
        break;

    case KIN_MIXED:
        // rot_1 on head, rot_2 on table
        // T_head = R1 * T_tool
        Mat4x4_Multiply(T_head, R1, T_tool);
        // T_table = R2 * T_pivot
        Mat4x4_Multiply(T_table, R2, T_pivot);
        break;

    default:
        break;
    }

    // ---- 计算零位矩阵 (angle = 0) ----
    Mat4x4 R1_zero, R2_zero;
    Mat4x4_RotateAxis(R1_zero, cfg->rot_1_axis, 0.0);
    Mat4x4_RotateAxis(R2_zero, cfg->rot_2_axis, 0.0);

    Mat4x4 T_head_0, T_table_0;
    Mat4x4_Identity(T_head_0);
    Mat4x4_Identity(T_table_0);

    switch (cfg->type) {
    case KIN_HEAD_HEAD:
        // T_head_0 = I * T_pivot * I * T_tool = T_pivot * T_tool
        Mat4x4_Multiply(T_head_0, T_pivot, T_tool);
        break;
    case KIN_TABLE_TABLE:
        // T_table_0 = I * T_pivot * I = T_pivot
        memcpy(T_table_0, T_pivot, sizeof(Mat4x4));
        break;
    case KIN_MIXED:
        // T_head_0 = I * T_tool = T_tool
        memcpy(T_head_0, T_tool, sizeof(Mat4x4));
        // T_table_0 = I * T_pivot = T_pivot
        memcpy(T_table_0, T_pivot, sizeof(Mat4x4));
        break;
    default:
        break;
    }

    // ---- 微分补偿计算 ----
    static const double origin[3] = {0.0, 0.0, 0.0};
    double vec_head[3], vec_head_0[3];
    double vec_table[3], vec_table_0[3];

    Mat4x4_ApplyPoint(T_head, origin, vec_head);
    Mat4x4_ApplyPoint(T_head_0, origin, vec_head_0);

    Mat4x4_ApplyPoint(T_table, tip_xyz, vec_table);
    Mat4x4_ApplyPoint(T_table_0, tip_xyz, vec_table_0);

    // joint = tip + delta_table - delta_head
    for (int i = 0; i < 3; i++) {
        joint_xyz[i] = tip_xyz[i]
                     + (vec_table[i] - vec_table_0[i])
                     - (vec_head[i]  - vec_head_0[i]);
    }
}
