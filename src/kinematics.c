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
    {0.0, 0.0, 0.0},    // pivot_offset
    0                   // chain_order (P1: 0=R1 外层, 已交付默认)
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

// Cross-Semantics Pose 实验 (2026-09-30): 方向矢量变换 (w=0, 平移列不贡献)
void Mat4x4_ApplyDir(const Mat4x4 m, const double v_in[3], double v_out[3])
{
    double x = v_in[0], y = v_in[1], z = v_in[2];
    v_out[0] = m[0][0]*x + m[0][1]*y + m[0][2]*z;
    v_out[1] = m[1][0]*x + m[1][1]*y + m[1][2]*z;
    v_out[2] = m[2][0]*x + m[2][1]*y + m[2][2]*z;
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
        if (cfg->chain_order == 0) {
            // 现状链序 (已交付默认, 字节级保持): T_head = R1 * T_pivot * R2 * T_tool
            Mat4x4_Multiply(tmp_a, R2, T_tool);        // R2 * T_tool
            Mat4x4_Multiply(tmp_b, T_pivot, tmp_a);     // T_pivot * R2 * T_tool
            Mat4x4_Multiply(T_head, R1, tmp_b);         // R1 * T_pivot * R2 * T_tool
        } else {
            // P1 (2026-09-04) chain 1: T_head = R2 * T_pivot * R1 * T_tool (C外B内腕)
            Mat4x4_Multiply(tmp_a, R1, T_tool);        // R1 * T_tool
            Mat4x4_Multiply(tmp_b, T_pivot, tmp_a);     // T_pivot * R1 * T_tool
            Mat4x4_Multiply(T_head, R2, tmp_b);         // R2 * T_pivot * R1 * T_tool
        }
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
        // (P1 注: θ=0 时 R1=R2=I, 两链序零位数值相同 — chain_order 不影响此块)
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

// ====================================================================
// Cross-Semantics Pose 实验 (2026-09-30): 前向运动学 — joint + 旋转角 → 刀尖 + 刀轴
// @Context: Non-RealTime Background Thread (sim_engine 落盘线程调用)
// @Safe: 纯数学 (sin/cos 可重入), 全程局部变量, 无静态状态。
//
// T_head 装配与 Kinematics_Inverse 逐链序复刻 — 故意不抽公共代码:
// Inverse 属已验证路径, 保持零字节改动; 本函数独立演化。
// ====================================================================

int Kinematics_Forward(const double joint_xyz[3], double rot_a, double rot_b, double rot_c,
                       double tip_xyz[3], double u_tool[3])
{
    KinConfig_t *cfg = &g_kin_config;

    // KIN_NONE / 未配置: 直通 (joint = tip, 刀轴竖直)
    if (cfg->type == KIN_NONE || cfg->rot_1_idx < 0 || cfg->rot_2_idx < 0) {
        memcpy(tip_xyz, joint_xyz, 3 * sizeof(double));
        u_tool[0] = 0.0; u_tool[1] = 0.0; u_tool[2] = -1.0;
        return 0;
    }

    // TABLE_TABLE / MIXED: 前向数学不同 (tip 随动台旋转), 未实现 — 直通返回 -1
    if (cfg->type != KIN_HEAD_HEAD) {
        memcpy(tip_xyz, joint_xyz, 3 * sizeof(double));
        u_tool[0] = 0.0; u_tool[1] = 0.0; u_tool[2] = -1.0;
        return -1;
    }

    double angle1 = get_angle_for_idx(cfg->rot_1_idx, rot_a, rot_b, rot_c) * DEG2RAD;
    double angle2 = get_angle_for_idx(cfg->rot_2_idx, rot_a, rot_b, rot_c) * DEG2RAD;

    Mat4x4 R1, R2;
    Mat4x4_RotateAxis(R1, cfg->rot_1_axis, angle1);
    Mat4x4_RotateAxis(R2, cfg->rot_2_axis, angle2);

    Mat4x4 T_tool;
    Mat4x4_Translate(T_tool, -cfg->tool_offset[0], -cfg->tool_offset[1], -cfg->tool_offset[2]);

    Mat4x4 T_pivot;
    Mat4x4_Translate(T_pivot, cfg->pivot_offset[0], cfg->pivot_offset[1], cfg->pivot_offset[2]);

    // ---- T_head 装配 (与 Inverse 同链序) ----
    Mat4x4 T_head, tmp_a, tmp_b;
    if (cfg->chain_order == 0) {
        Mat4x4_Multiply(tmp_a, R2, T_tool);        // R2 * T_tool
        Mat4x4_Multiply(tmp_b, T_pivot, tmp_a);     // T_pivot * R2 * T_tool
        Mat4x4_Multiply(T_head, R1, tmp_b);         // R1 * T_pivot * R2 * T_tool
    } else {
        Mat4x4_Multiply(tmp_a, R1, T_tool);        // R1 * T_tool
        Mat4x4_Multiply(tmp_b, T_pivot, tmp_a);     // T_pivot * R1 * T_tool
        Mat4x4_Multiply(T_head, R2, tmp_b);         // R2 * T_pivot * R1 * T_tool
    }

    // 零位矩阵 (θ=0 时 R1=R2=I, 两链序数值相同): T_head_0 = T_pivot * T_tool
    Mat4x4 T_head_0;
    Mat4x4_Multiply(T_head_0, T_pivot, T_tool);

    // tip = joint + (T_head(θ)·O − T_head(0)·O)
    static const double origin[3] = {0.0, 0.0, 0.0};
    double vec_head[3], vec_head_0[3];
    Mat4x4_ApplyPoint(T_head, origin, vec_head);
    Mat4x4_ApplyPoint(T_head_0, origin, vec_head_0);
    for (int i = 0; i < 3; i++)
        tip_xyz[i] = joint_xyz[i] + (vec_head[i] - vec_head_0[i]);

    // u_tool = 旋转链 × (0,0,-1) (内层先作用; 平移矩阵对方向不贡献)
    double t0[3] = {0.0, 0.0, -1.0}, t1[3];
    if (cfg->chain_order == 0) {
        Mat4x4_ApplyDir(R2, t0, t1);
        Mat4x4_ApplyDir(R1, t1, u_tool);
    } else {
        Mat4x4_ApplyDir(R1, t0, t1);
        Mat4x4_ApplyDir(R2, t1, u_tool);
    }
    return 0;
}

// ====================================================================
// P1 (2026-09-04): G43.5 刀轴矢量 → 旋转角闭式解
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 (sqrt/atan2/acos/fmod 可重入), 全程局部变量, 无静态状态。
//
// 数学 (t = 刀轴单位矢 = (0,0,-1) 内层系, u = 机床系期望刀轴, 归一化后):
//   chain 0 (R1 外层, 刀具与 rot_2 轴线同轴): u = R_1(b)·t — 可达集为一条
//     圆弧, rot_2 对方向中性 → n_solved=1 (rot_2 保持现值)。
//     rot_1 绕 Y: u = (-sin b, 0, -cos b) ⇒ b = atan2(-u.x, -u.z), 需 |u.y|<EPS
//     rot_1 绕 X: u = (0, sin b, -cos b)  ⇒ b = atan2( u.y, -u.z), 需 |u.x|<EPS
//   chain 1 (R2 外层绕 Z): u = R_z(c)·R_1(b)·t
//     u_z = -cos b ⇒ b = acos(clamp(-u.z)) ∈ [0,π];
//     内层绕 Y: u = (-sin b·cos c, -sin b·sin c, -cos b) ⇒ c = atan2(-u.y, -u.x)
//     内层绕 X: u = (-sin b·sin c,  sin b·cos c, -cos b) ⇒ c = atan2(-u.x, u.y)
//     双解 (b,c) / (-b, c+π) 取离当前近者 (C 距差折 ±180, 复用 wrap 思想);
//     腕奇异 |sin b|<EPS (u 竖直): c 不定 → 保持现值, n_solved=1。
// 输出单位: 度 (直配 parser val_axis)。
// ====================================================================

#define KIN_ORIENT_EPS 1e-6
#define KIN_RAD2DEG    (180.0 / M_PI)

int Kinematics_SolveOrientation(const double u_in[3], double cur_rot1_deg, double cur_rot2_deg,
                                double *out_rot1_deg, double *out_rot2_deg, int *n_solved)
{
    *n_solved = 0;
    KinConfig_t *cfg = &g_kin_config;

    if (cfg->type != KIN_HEAD_HEAD || cfg->rot_1_idx < 0 || cfg->rot_2_idx < 0)
        return KIN_ORIENT_ERR_CONFIG;

    double len = sqrt(u_in[0]*u_in[0] + u_in[1]*u_in[1] + u_in[2]*u_in[2]);
    if (len < 1e-9) return KIN_ORIENT_ERR_UNREACH;   // 防御 (parser 已前置拦零矢量)
    double ux = u_in[0] / len, uy = u_in[1] / len, uz = u_in[2] / len;

    if (cfg->chain_order == 0) {
        // 前提: 刀具与内层 rot_2 轴线同轴 (否则闭式解不成立)
        if (cfg->rot_2_axis != 2 ||
            fabs(cfg->tool_offset[0]) > 1e-9 || fabs(cfg->tool_offset[1]) > 1e-9)
            return KIN_ORIENT_ERR_CONFIG;
        double b;
        if (cfg->rot_1_axis == 1) {              // 外层绕 Y
            if (fabs(uy) > KIN_ORIENT_EPS) return KIN_ORIENT_ERR_UNREACH;
            b = atan2(-ux, -uz);
        } else if (cfg->rot_1_axis == 0) {       // 外层绕 X
            if (fabs(ux) > KIN_ORIENT_EPS) return KIN_ORIENT_ERR_UNREACH;
            b = atan2(uy, -uz);
        } else {
            return KIN_ORIENT_ERR_CONFIG;
        }
        *out_rot1_deg = b * KIN_RAD2DEG;
        *n_solved = 1;
        return KIN_ORIENT_OK;
    }

    // ---- chain 1: R2 外层 (必须绕 Z) ----
    if (cfg->rot_2_axis != 2) return KIN_ORIENT_ERR_CONFIG;

    double cz = -uz;
    if (cz >  1.0) cz =  1.0;
    if (cz < -1.0) cz = -1.0;
    double b = acos(cz);                          // ∈ [0, π]

    if (fabs(sin(b)) < KIN_ORIENT_EPS) {
        // 腕奇异 (u 竖直): c 不定保持现值; b=180° 超软限位由 0x0024 管线兜底
        *out_rot1_deg = (uz < 0.0) ? 0.0 : 180.0;
        *n_solved = 1;
        return KIN_ORIENT_OK;
    }

    double c;
    if (cfg->rot_1_axis == 1)      c = atan2(-uy, -ux);   // 内层绕 Y
    else if (cfg->rot_1_axis == 0) c = atan2(-ux, uy);    // 内层绕 X
    else return KIN_ORIENT_ERR_CONFIG;

    double b1 =  b * KIN_RAD2DEG;
    double c1 =  c * KIN_RAD2DEG;
    double b2 = -b1;
    double c2 =  c1 + 180.0;

    // C 距差折 (-180, 180] — 与 wrap_rotary_shortest 同约定
    double d1c = fmod(c1 - cur_rot2_deg, 360.0);
    if (d1c > 180.0) d1c -= 360.0; else if (d1c <= -180.0) d1c += 360.0;
    double d2c = fmod(c2 - cur_rot2_deg, 360.0);
    if (d2c > 180.0) d2c -= 360.0; else if (d2c <= -180.0) d2c += 360.0;
    double d1 = (b1 - cur_rot1_deg) * (b1 - cur_rot1_deg) + d1c * d1c;
    double d2 = (b2 - cur_rot1_deg) * (b2 - cur_rot1_deg) + d2c * d2c;

    if (d1 <= d2) { *out_rot1_deg = b1; *out_rot2_deg = c1; }   // 平局归 b>0 解
    else          { *out_rot1_deg = b2; *out_rot2_deg = c2; }
    *n_solved = 2;
    return KIN_ORIENT_OK;
}

// ====================================================================
// P1 (2026-09-07): G68.2/G68.3 倾斜面构帧
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 (sqrt/sin/cos 可重入), 全程局部变量。
// R_out 行主序, 列向量 = [u v n]:
//   正变换 machine = R·p:  m_x = R[0]px+R[1]py+R[2]pz (逐行点乘)
//   反变换 p = Rᵀ·d:       p_x = R[0]dx+R[3]dy+R[6]dz (取列即行)
// ====================================================================
int Kinematics_BuildTiltFrame(const double n_in[3], double q_deg, double R_out[9])
{
    double len = sqrt(n_in[0]*n_in[0] + n_in[1]*n_in[1] + n_in[2]*n_in[2]);
    if (len < 1e-9) return -1;
    double nx = n_in[0]/len, ny = n_in[1]/len, nz = n_in[2]/len;

    // 参考轴 a: 与 n 最不平行 (n 接近 X 时用 Y, 否则用 X)
    double ax = (fabs(nx) < 0.9) ? 1.0 : 0.0;
    double ay = (fabs(nx) < 0.9) ? 0.0 : 1.0;
    double az = 0.0;

    // u0 = normalize(a × n)
    double u0x = ay*nz - az*ny;
    double u0y = az*nx - ax*nz;
    double u0z = ax*ny - ay*nx;
    double l0 = sqrt(u0x*u0x + u0y*u0y + u0z*u0z);
    if (l0 < 1e-12) return -1;               // 防御: a∥n (理论上不会发生)
    u0x /= l0; u0y /= l0; u0z /= l0;

    // w = n × u0 (单位, ⊥u0)
    double wx = ny*u0z - nz*u0y;
    double wy = nz*u0x - nx*u0z;
    double wz = nx*u0y - ny*u0x;

    // Rodrigues (n⊥u0 退化式): u = u0·cosQ + w·sinQ
    double q  = q_deg * DEG2RAD;
    double cq = cos(q), sq = sin(q);
    double ux = u0x*cq + wx*sq;
    double uy = u0y*cq + wy*sq;
    double uz = u0z*cq + wz*sq;

    // v = n × u
    double vx = ny*uz - nz*uy;
    double vy = nz*ux - nx*uz;
    double vz = nx*uy - ny*ux;

    R_out[0] = ux; R_out[1] = vx; R_out[2] = nx;
    R_out[3] = uy; R_out[4] = vy; R_out[5] = ny;
    R_out[6] = uz; R_out[7] = vz; R_out[8] = nz;
    return 0;
}

void Kinematics_BuildEulerXYZ(double ax_deg, double ay_deg, double az_deg, double R_out[9])
{
    double axr = ax_deg * DEG2RAD, ayr = ay_deg * DEG2RAD, azr = az_deg * DEG2RAD;
    double cx = cos(axr), sx = sin(axr);
    double cy = cos(ayr), sy = sin(ayr);
    double cz = cos(azr), sz = sin(azr);

    // R = Rz(az)·Ry(ay)·Rx(ax) 展开式
    R_out[0] = cz*cy;
    R_out[1] = cz*sy*sx - sz*cx;
    R_out[2] = cz*sy*cx + sz*sx;
    R_out[3] = sz*cy;
    R_out[4] = sz*sy*sx + cz*cx;
    R_out[5] = sz*sy*cx - cz*sx;
    R_out[6] = -sy;
    R_out[7] = cy*sx;
    R_out[8] = cy*cx;
}
