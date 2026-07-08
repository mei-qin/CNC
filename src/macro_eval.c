#include "macro_eval.h"
#include "global_def.h"   // g_coord_mgr, g_axis_map, g_sys_alarm_state, AXIS_NUM
#include "axis_ctrl.h"    // api_flush_planner / is_trajectory_finished
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <unistd.h>       // usleep

#define PI 3.14159265358979323846

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 全局静态数组 + math.h，无 malloc，无系统调用阻塞。
// 注意：#500-#999 在本实现中仅为 RAM 数组，掉电丢失。
//       永久化（FRAM/Flash 回写）属于独立任务，不在本提交范围。

static double g_macro_vars[MACRO_VAR_ARRAY_SIZE];

#define MACRO_EXPR_MAX_DEPTH 32  // 表达式递归深度上限，防爆栈

void Macro_Init(void)
{
    for(int i = 0; i < MACRO_VAR_ARRAY_SIZE; i++){
        g_macro_vars[i] = 0.0;
    }
}

// 系统变量轴序号 → 轴字母
// offset 0..4 → X/Y/Z/C/B，与 main.c SMC_ConfigAxisTopology 顺序一致
// (Fanuc 标准: #5021=X, #5022=Y, #5023=Z, #5024=第一旋转, #5025=第二旋转)
static char sysvar_letter(int offset)
{
    static const char letters[] = {'X', 'Y', 'Z', 'C', 'B'};
    if(offset < 0 || offset >= (int)(sizeof(letters) / sizeof(letters[0]))){
        return '?';
    }
    return letters[offset];
}

// ---- Parser/RT 时序同步屏障 ----
// 问题: parser 线程跑得远比 RT 线程快 (队列深度 1024)。读到 #5021-#5025 (机床坐标)
// 或 #5031-#5035 (逻辑坐标) 等反映"物理当前状态"的系统变量时, 直接读
// g_coord_mgr.current_g53_pos[] 拿到的是"现在"的位置, 而不是"该 G 代码行
// 在 RT 执行到那一刻"的位置。后果示例:
//   G01 X100 F1000
//   #1 = #5021     (期望 100, 实际可能接近 0)
//   G01 X[#1 + 50] (期望走到 150, 实际走到 ~50, 撞机/走错)
//
// 修复: 读物理状态宏变量前, 把 parser 已下发的运动全部 flush 到 RT,
// 阻塞等插补器静止, 再读位置 — 把"未来状态"读法变成"现在状态"读法。
// 代价: 队列断流一次, 后续 parser 重新填充。仅 #5021-#5025/#5031-#5035 触发,
// 不影响普通 G 代码的流水线性能。
//
// @Context: Non-RealTime Background Thread (parser 求值路径调用)
// @Side-Effect: 触发 planner flush + 阻塞等待, 不能在 RT 线程调用
static void sync_to_physical_state(void)
{
    api_flush_planner();
    while (!is_trajectory_finished()) {
        usleep(1000);   // 1ms 轮询, 不阻塞 RT
    }
}

double Macro_GetValue(int index)
{
    // ---- 局部/公共变量: 直接索引 ----
    if(index >= 0 && index <= MACRO_VAR_MAX_INDEX){
        return g_macro_vars[index];
    }

    // ---- 系统变量: 机床坐标 (G53) ----
    if(index >= SYSVAR_MACHINE_POS_BASE && index < SYSVAR_MACHINE_POS_BASE + AXIS_NUM){
        sync_to_physical_state();   // 等队列空, 再读物理位置
        int phys = g_axis_map[sysvar_letter(index - SYSVAR_MACHINE_POS_BASE) - 'A'];
        if(phys < 0) return 0.0;
        return g_coord_mgr.current_g53_pos[phys];
    }

    // ---- 系统变量: 逻辑坐标 (当前 WCS) ----
    if(index >= SYSVAR_LOGICAL_POS_BASE && index < SYSVAR_LOGICAL_POS_BASE + AXIS_NUM){
        sync_to_physical_state();   // 等队列空, 再读逻辑位置
        int phys = g_axis_map[sysvar_letter(index - SYSVAR_LOGICAL_POS_BASE) - 'A'];
        if(phys < 0) return 0.0;
        return g_coord_mgr.current_logical_pos[phys];
    }

    // ---- 系统变量: WCS 偏置 (G54-G59) ----
    // 公式: wcs_idx = (N - 5221) / 20, axis_off = (N - 5221) % 20
    // wcs_idx 0=G54..5=G59; axis_off 0..4 对应 X/Y/Z/C/B
    if(index >= SYSVAR_WCS_OFFSET_BASE && index <= 5325){
        int wcs_idx  = (index - SYSVAR_WCS_OFFSET_BASE) / 20;
        int axis_off = (index - SYSVAR_WCS_OFFSET_BASE) % 20;
        if(wcs_idx > 5 || axis_off >= AXIS_NUM) return 0.0;
        int phys = g_axis_map[sysvar_letter(axis_off) - 'A'];
        if(phys < 0) return 0.0;
        return g_coord_mgr.work_offsets[wcs_idx][phys];
    }

    // ---- P5': 系统变量: 扩展 WCS 偏置 (G54.1 P1-P48) ----
    // 公式: stride=20, p_idx = (N - 7001) / 20, axis_off = (N - 7001) % 20
    // p_idx 0=P1..47=P48; axis_off 0..4 对应 X/Y/Z/C/B
    if(index >= SYSVAR_EXT_WCS_OFFSET_BASE && index <= 7948){
        int p_idx    = (index - SYSVAR_EXT_WCS_OFFSET_BASE) / 20;
        int axis_off = (index - SYSVAR_EXT_WCS_OFFSET_BASE) % 20;
        if(p_idx > 47 || axis_off >= AXIS_NUM) return 0.0;
        int phys = g_axis_map[sysvar_letter(axis_off) - 'A'];
        if(phys < 0) return 0.0;
        return g_coord_mgr.work_offsets_ext[p_idx][phys];
    }

    // ---- 系统变量: 报警/消息 (写专用，读返回 0) ----
    if(index == SYSVAR_USER_ALARM || index == SYSVAR_OPERATOR_MSG){
        return 0.0;
    }

    printf("[Macro] 未知系统变量 #%d，返回 0.0\n", index);
    return 0.0;
}

void Macro_SetValue(int index, double val)
{
    // ---- 局部/公共变量: 直接索引 ----
    if(index >= 0 && index <= MACRO_VAR_MAX_INDEX){
        g_macro_vars[index] = val;
        return;
    }

    // ---- 系统变量: WCS 偏置写入 (G54-G59) ----
    if(index >= SYSVAR_WCS_OFFSET_BASE && index <= 5325){
        int wcs_idx  = (index - SYSVAR_WCS_OFFSET_BASE) / 20;
        int axis_off = (index - SYSVAR_WCS_OFFSET_BASE) % 20;
        if(wcs_idx > 5 || axis_off >= AXIS_NUM){
            printf("[Macro] 无效 WCS 偏置变量 #%d\n", index);
            return;
        }
        int phys = g_axis_map[sysvar_letter(axis_off) - 'A'];
        if(phys < 0){
            printf("[Macro] WCS 偏置 #%d 对应轴未映射\n", index);
            return;
        }
        g_coord_mgr.work_offsets[wcs_idx][phys] = val;
        return;
    }

    // ---- P5': 系统变量: 扩展 WCS 偏置写入 (G54.1 P1-P48) ----
    if(index >= SYSVAR_EXT_WCS_OFFSET_BASE && index <= 7948){
        int p_idx    = (index - SYSVAR_EXT_WCS_OFFSET_BASE) / 20;
        int axis_off = (index - SYSVAR_EXT_WCS_OFFSET_BASE) % 20;
        if(p_idx > 47 || axis_off >= AXIS_NUM){
            printf("[Macro] 无效扩展 WCS 偏置变量 #%d\n", index);
            return;
        }
        int phys = g_axis_map[sysvar_letter(axis_off) - 'A'];
        if(phys < 0){
            printf("[Macro] 扩展 WCS 偏置 #%d 对应轴未映射\n", index);
            return;
        }
        g_coord_mgr.work_offsets_ext[p_idx][phys] = val;
        return;
    }

    // ---- 系统变量: #3000 用户报警 (触发软停机) ----
    if(index == SYSVAR_USER_ALARM){
        int code = (int)val;
        printf("[Macro] #3000 用户报警: code=%d (触发软停机)\n", code);
        g_sys_alarm_state = 1;
        return;
    }

    // ---- 系统变量: #3006 操作员消息 (打印 + 可选暂停) ----
    if(index == SYSVAR_OPERATOR_MSG){
        int code = (int)val;
        printf("[Macro] #3006 操作员消息: code=%d %s\n",
               code, (code & 1) ? "(暂停)" : "(仅显示)");
        // 注: 奇数值的"暂停"语义在 Phase 2A.1 简化版中仅打印不真停
        // (避免与 g_parser_ctrl.is_paused 的 UI 清除流程耦合, 待 UI 介入后补)
        return;
    }

    // ---- 系统变量: 机床坐标/逻辑坐标只读 ----
    if((index >= SYSVAR_MACHINE_POS_BASE && index < SYSVAR_MACHINE_POS_BASE + AXIS_NUM) ||
       (index >= SYSVAR_LOGICAL_POS_BASE && index < SYSVAR_LOGICAL_POS_BASE + AXIS_NUM)){
        printf("[Macro] 拒绝写入只读系统变量 #%d\n", index);
        return;
    }

    printf("[Macro] 未知系统变量 #%d = %g，拒绝写入\n", index, val);
}

// ---- Phase 2B M5: 局部变量 #1-#33 批量访问 ----
// M98 调用时一次性保存调用者 #1-#33, M99 返回时一次性恢复
// 单次循环, 无 bounds check 开销 (索引 1..33 是常量安全范围)
void Macro_GetLocals(double out[34])
{
    for(int i = 1; i <= 33; i++){
        out[i] = g_macro_vars[i];
    }
}

void Macro_SetLocals(const double in[34])
{
    for(int i = 1; i <= 33; i++){
        g_macro_vars[i] = in[i];
    }
}

void Macro_ClearLocals(void)
{
    for(int i = 1; i <= 33; i++){
        g_macro_vars[i] = 0.0;
    }
}

// ---- 递归下降求值器 ----
// 文法 (Phase 2B M2 扩展: 加比较 + 逻辑运算)：
//   logical    := [(NOT)] comparison { (AND|OR|XOR) comparison }
//   comparison := expression [ (EQ|NE|GT|LT|GE|LE) expression ]
//   expression := term   { ('+'|'-') term }
//   term       := factor { ('*'|'/') factor }
//   factor     := ('-'|'+') factor
//               | '[' logical ']'
//               | '#' integer
//               | func '[' logical ']'
//               | number
// 布尔语义: 0.0 = false, 非 0 = true; 比较/逻辑运算返回 1.0 或 0.0

static double parse_expression(const char **p, int depth);
static double parse_comparison(const char **p, int depth);
static double parse_logical(const char **p, int depth);

static void skip_ws(const char **p)
{
    while(**p == ' ' || **p == '\t') (*p)++;
}

static double parse_factor(const char **p, int depth)
{
    if(depth > MACRO_EXPR_MAX_DEPTH){
        printf("[Macro] 表达式嵌套深度超限 (%d)，中止\n", depth);
        return 0.0;
    }
    skip_ws(p);

    // 一元正负
    if(**p == '-'){ (*p)++; return -parse_factor(p, depth + 1); }
    if(**p == '+'){ (*p)++; return  parse_factor(p, depth + 1); }

    // 嵌套括号 [...]（CNC 宏程序标准用中括号，小括号已被注释占用）
    // M2: 内层走 parse_logical (支持 [#1 LT 5] 条件表达式, 纯算术向后兼容下推到 expression)
    if(**p == '['){
        (*p)++;
        double v = parse_logical(p, depth + 1);
        skip_ws(p);
        if(**p == ']') (*p)++;
        else printf("[Macro] 警告：缺少 ']'\n");
        return v;
    }

    // 变量 #N
    if(**p == '#'){
        (*p)++;
        char *end;
        long idx = strtol(*p, &end, 10);
        if(end == *p){
            printf("[Macro] '#' 后缺变量号\n");
            return 0.0;
        }
        *p = end;
        return Macro_GetValue((int)idx);
    }

    // 函数调用 func[ ... ]
    if(isalpha((unsigned char)**p)){
        char name[8] = {0};
        int n = 0;
        while(**p && isalpha((unsigned char)**p) && n < (int)sizeof(name) - 1){
            name[n++] = (char)toupper((unsigned char)**p);
            (*p)++;
        }
        skip_ws(p);
        if(**p != '['){
            printf("[Macro] 函数 %s 后缺 '['，按 0 处理\n", name);
            return 0.0;
        }
        (*p)++;
        double arg = parse_logical(p, depth + 1);  // M2: 走 logical (支持 SIN[#1 GT 0] 等)
        skip_ws(p);
        if(**p == ']') (*p)++;
        else printf("[Macro] 函数 %s 缺少 ']'\n", name);

        // 角度函数入参为度（CNC 宏程序标准）
        if(!strcmp(name, "SIN"))  return sin(arg * PI / 180.0);
        if(!strcmp(name, "COS"))  return cos(arg * PI / 180.0);
        if(!strcmp(name, "TAN"))  return tan(arg * PI / 180.0);
        if(!strcmp(name, "SQRT")) return sqrt(arg);
        if(!strcmp(name, "ABS"))  return fabs(arg);
        if(!strcmp(name, "FIX"))  return floor(arg);     // 向下取整
        if(!strcmp(name, "FUP"))  return ceil(arg);      // 向上取整
        if(!strcmp(name, "ROUND")){
            // Fanuc ROUND：四舍五入到最近整数，负数对称（-2.5 -> -3）
            return (arg >= 0.0) ? floor(arg + 0.5) : ceil(arg - 0.5);
        }
        printf("[Macro] 未知函数 %s\n", name);
        return 0.0;
    }

    // 数值常量
    char *end;
    double v = strtod(*p, &end);
    if(end == *p){
        printf("[Macro] 期望数值，遇到 '%c'\n", **p ? **p : '?');
        return 0.0;
    }
    *p = end;
    return v;
}

static double parse_term(const char **p, int depth)
{
    double v = parse_factor(p, depth);
    for(;;){
        skip_ws(p);
        char op = **p;
        if(op != '*' && op != '/') break;
        (*p)++;
        double rhs = parse_factor(p, depth + 1);
        if(op == '*') v *= rhs;
        else {
            // 除零保护（用户输入边界校验）
            if(fabs(rhs) < 1e-12){
                printf("[Macro] 除零保护，本项置 0\n");
                v = 0.0;
            } else v /= rhs;
        }
    }
    return v;
}

static double parse_expression(const char **p, int depth)
{
    double v = parse_term(p, depth);
    for(;;){
        skip_ws(p);
        char op = **p;
        if(op != '+' && op != '-') break;
        (*p)++;
        double rhs = parse_term(p, depth + 1);
        if(op == '+') v += rhs; else v -= rhs;
    }
    return v;
}

// ---- Phase 2B M2: 比较与逻辑运算符 ----
typedef enum {
    OP_NONE,
    OP_EQ, OP_NE, OP_GT, OP_LT, OP_GE, OP_LE,   // 比较 (range 1-6, 顺序重要用于范围判断)
    OP_AND, OP_OR, OP_XOR,                        // 二元逻辑 (7-9)
    OP_NOT,                                       // 一元前缀 (10)
} MacroOp_t;

// peek (不消费 *p): 检查当前位置是否是合法运算符关键字
// 大小写不敏感, 必须完整词 (后续非字母), 否则不匹配 (防 "ANDX" 误判)
// 匹配成功返回 1, *op_out 写枚举值, *end_out 写运算符后位置
static int peek_logical_op(const char *p, MacroOp_t *op_out, const char **end_out)
{
    while(*p == ' ' || *p == '\t') p++;
    if(!isalpha((unsigned char)*p)) return 0;

    char name[8] = {0};
    int n = 0;
    while(*p && isalpha((unsigned char)*p) && n < (int)sizeof(name) - 1){
        name[n++] = (char)toupper((unsigned char)*p);
        p++;
    }
    if(*p && isalpha((unsigned char)*p)) return 0;  // 太长, 不是关键字

    MacroOp_t op = OP_NONE;
    if(!strcmp(name, "EQ"))      op = OP_EQ;
    else if(!strcmp(name, "NE")) op = OP_NE;
    else if(!strcmp(name, "GT")) op = OP_GT;
    else if(!strcmp(name, "LT")) op = OP_LT;
    else if(!strcmp(name, "GE")) op = OP_GE;
    else if(!strcmp(name, "LE")) op = OP_LE;
    else if(!strcmp(name, "AND")) op = OP_AND;
    else if(!strcmp(name, "OR"))  op = OP_OR;
    else if(!strcmp(name, "XOR")) op = OP_XOR;
    else if(!strcmp(name, "NOT")) op = OP_NOT;

    if(op == OP_NONE) return 0;
    *op_out = op;
    if(end_out) *end_out = p;
    return 1;
}

// Phase 2B M2: comparison 层 (EQ/NE/GT/LT/GE/LE)
// 单次比较, 不可链式 ([1 LT 2 LT 3] 不合法, 应当用括号)
static double parse_comparison(const char **p, int depth)
{
    double lhs = parse_expression(p, depth);
    MacroOp_t op;
    const char *after;
    if(!peek_logical_op(*p, &op, &after)) return lhs;
    if(op < OP_EQ || op > OP_LE) return lhs;  // 非比较运算符, 留给 parse_logical 处理

    *p = after;
    double rhs = parse_expression(p, depth + 1);

    switch(op){
        case OP_EQ: return (fabs(lhs - rhs) < 1e-9) ? 1.0 : 0.0;
        case OP_NE: return (fabs(lhs - rhs) >= 1e-9) ? 1.0 : 0.0;
        case OP_GT: return (lhs >  rhs) ? 1.0 : 0.0;
        case OP_LT: return (lhs <  rhs) ? 1.0 : 0.0;
        case OP_GE: return (lhs >= rhs) ? 1.0 : 0.0;
        case OP_LE: return (lhs <= rhs) ? 1.0 : 0.0;
        default:    return 0.0;
    }
}

// Phase 2B M2: logical 层 (NOT 一元前缀 + AND/OR/XOR 二元左结合)
// AND/OR/XOR 同级 (M2 简化), 复杂条件建议用括号明确分组
static double parse_logical(const char **p, int depth)
{
    if(depth > MACRO_EXPR_MAX_DEPTH){
        printf("[Macro] 表达式嵌套深度超限 (%d)\n", depth);
        return 0.0;
    }
    skip_ws(p);

    // 一元 NOT 前缀
    MacroOp_t prefix_op;
    const char *prefix_after;
    int is_not = 0;
    if(peek_logical_op(*p, &prefix_op, &prefix_after) && prefix_op == OP_NOT){
        *p = prefix_after;
        is_not = 1;
    }

    double lhs = parse_comparison(p, depth);
    if(is_not) lhs = (lhs == 0.0) ? 1.0 : 0.0;

    // 二元 AND/OR/XOR (左结合, 同级)
    for(;;){
        MacroOp_t op;
        const char *after;
        if(!peek_logical_op(*p, &op, &after)) break;
        if(op != OP_AND && op != OP_OR && op != OP_XOR) break;
        *p = after;

        double rhs = parse_logical(p, depth + 1);

        double l = (lhs != 0.0) ? 1.0 : 0.0;
        double r = (rhs != 0.0) ? 1.0 : 0.0;
        if(op == OP_AND)      lhs = (l && r) ? 1.0 : 0.0;
        else if(op == OP_OR)  lhs = (l || r) ? 1.0 : 0.0;
        else                  lhs = (l != r) ? 1.0 : 0.0;  // XOR
    }
    return lhs;
}

double Evaluate_Expression(const char **p)
{
    return parse_logical(p, 0);  // M2: 顶层改 logical (纯算术下推到 expression, 向后兼容)
}

// 行首赋值拦截：#N = <expr>
// 标准 CNC 中赋值必须独占一行（不会出现 G01 X10 #100 = 5 这种语法）
// 返回 1 = 已消费此行（不应再当运动指令）
// 返回 0 = 不是赋值，交回主解析器
int Macro_TryParseAssignment(const char *line)
{
    const char *p = line;
    skip_ws(&p);
    if(*p != '#') return 0;

    char *end;
    long idx = strtol(p + 1, &end, 10);
    if(end == p + 1) return 0;  // # 后非数字，不是赋值

    const char *eq = end;
    while(*eq == ' ' || *eq == '\t') eq++;
    if(*eq != '=') return 0;    // 后面不是 '='，不是赋值（可能为 X#100 形式，交主解析器）

    const char *rhs = eq + 1;
    double val = Evaluate_Expression(&rhs);
    Macro_SetValue((int)idx, val);
    printf("[Macro] #%d = %g\n", (int)idx, val);
    return 1;
}
