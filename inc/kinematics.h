#ifndef KINEMATICS_H
#define KINEMATICS_H

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学运算，无副作用。

// 通用转台逆运动学（支持 AC / BC / ABC 构型）
// 将刀尖点在工件坐标系中的位置，根据当前旋转轴角度，
// 转换为物理机床线性轴坐标。
//
// 运动链: 机床基座 -> B轴(绕Y) -> A轴(绕X) -> C轴(绕Z) -> 工件
// 正解: P_machine = R_Y(B) * R_X(A) * R_Z(C) * P_workpiece
// 旋转中心偏置暂设为 0（纯旋转矩阵补偿）。
//
// tip_xyz[3]:  刀尖在工件坐标系中的 X/Y/Z (mm)
// rot_a, rot_b, rot_c: 旋转轴角度 (度), 未配置的轴传 0.0
// joint_xyz[3]: 输出补偿后的物理机床 X/Y/Z (mm)
void Kinematics_Inverse(double tip_xyz[3], double rot_a, double rot_b, double rot_c, double joint_xyz[3]);

#endif // KINEMATICS_H
