#ifndef MACRO_EVAL_H
#define MACRO_EVAL_H

// 宏变量数组大小：覆盖 #1-#33 (局部)、#100-#199 (公共)、#500-#999 (永久/RAM)
// #N 直接映射到 g_macro_vars[N]，索引 0 保留不用
#define MACRO_VAR_ARRAY_SIZE  1000
#define MACRO_VAR_MAX_INDEX   999

// ---- Phase 2A.1 系统变量索引常量 ----
// 详细映射见 macro_eval.c Macro_GetValue/Macro_SetValue 的分发逻辑
#define SYSVAR_MACHINE_POS_BASE  5021   // #5021+#5022+...+#5025 读机床坐标 (G53)
#define SYSVAR_LOGICAL_POS_BASE  5031   // #5031+...+#5035       读逻辑坐标 (当前 WCS)
#define SYSVAR_WCS_OFFSET_BASE   5221   // #5221-#5325           WCS 偏置 (G54-G59) 读/写
#define SYSVAR_USER_ALARM        3000   // 写触发软停机 (g_sys_alarm_state = 1)
#define SYSVAR_OPERATOR_MSG      3006   // 写打印操作员消息 (奇数值=暂停, 偶数值=仅显示)

// 模块初始化（main 启动调用一次，把 #500-#999 区域清零，避免上电随机值）
void Macro_Init(void);

// 安全访问接口（带越界保护，越界返回 0.0 / 拒绝写入并打印警告）
double Macro_GetValue(int index);
void   Macro_SetValue(int index, double val);

// ---- Phase 2B M5: 局部变量 #1-#33 批量访问 (M98/M99 调用栈 save/restore 用) ----
// out/in 数组大小 34 (索引 1..33 有效, [0] 不用)
// 比逐次调用 Macro_GetValue/SetValue 高效, 且单次 bounds check
void Macro_GetLocals(double out[34]);         // out[i] = g_macro_vars[i], i=1..33
void Macro_SetLocals(const double in[34]);    // g_macro_vars[i] = in[i], i=1..33
void Macro_ClearLocals(void);                 // g_macro_vars[i] = 0.0, i=1..33

// 表达式求值：从 *p 解析一个完整表达式，推进 *p 到表达式末尾
// 支持：
//   数值常量 12.34、变量 #100、嵌套 [...]
//   一元 +/-、四则 + - * /（带除零保护）
//   SIN COS TAN（角度制）SQRT ABS ROUND FIX FUP
// 递归深度上限 32，防爆栈
double Evaluate_Expression(const char **p);

// 行首赋值拦截：解析 "#N = <expr>"
// 返回 1 = 已消费此行（不应再当运动指令处理）
// 返回 0 = 不是赋值，交回主解析器
int Macro_TryParseAssignment(const char *line);

#endif // MACRO_EVAL_H
