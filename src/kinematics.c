// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: cos/sin 为可重入的纯数学函数，允许使用。

#include "kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (M_PI / 180.0)

void Kinematics_Inverse(double tip_xyz[3], double rot_a, double rot_b, double rot_c, double joint_xyz[3])
{
    double A = rot_a * DEG2RAD;
    double B = rot_b * DEG2RAD;
    double C = rot_c * DEG2RAD;

    double cA = cos(A), sA = sin(A);
    double cB = cos(B), sB = sin(B);
    double cC = cos(C), sC = sin(C);

    double x = tip_xyz[0];
    double y = tip_xyz[1];
    double z = tip_xyz[2];

    // P_machine = R_Y(-B) * R_X(-A) * R_Z(-C) * P_workpiece
    // 逆转补偿：cos(-θ)=cos(θ), sin(-θ)=-sin(θ)
    // 顺序：先绕 Z 逆转(-C) → 绕 X 逆转(-A) → 绕 Y 逆转(-B)

    // Step 1: R_Z(-C) * P_tip
    double x1 =  cC * x + sC * y;
    double y1 = -sC * x + cC * y;
    double z1 =  z;

    // Step 2: R_X(-A) * P1
    double x2 =  x1;
    double y2 =  cA * y1 + sA * z1;
    double z2 = -sA * y1 + cA * z1;

    // Step 3: R_Y(-B) * P2
    joint_xyz[0] =  cB * x2 - sB * z2;
    joint_xyz[1] =  y2;
    joint_xyz[2] =  sB * x2 + cB * z2;
}
