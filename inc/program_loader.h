#ifndef PROGRAM_LOADER_H
#define PROGRAM_LOADER_H

#include <stdio.h>

// ---- Phase 2B M1: 程序加载器与 N 标签管理 ----
// 把 G 代码文件全文加载到内存, 建立 N 标签 → 行索引映射,
// 供 parser_thread_func 用 PC 游标遍历 + GOTO 跳转

#define MAX_PROGRAM_LINES    20000    // 单文件最多 2 万行 (典型 CNC < 1000 行)
#define MAX_PROGRAM_LINE_LEN 256      // 与 parser buffer 对齐
#define MAX_N_LABELS         1000     // N 标签数上限
#define MAX_PC_STEPS         1000000  // PC 步进计数器上限, 防无条件 GOTO 死循环
#define MAX_WHILE_NESTING    30       // Phase 2B M3: WHILE/DO/END 嵌套深度上限
#define MAX_O_LABELS         100      // Phase 2B M5: O 子程序标签数上限
#define MAX_M98_REPEAT       9999     // Phase 2B M5: M98 L 重复次数上限 (Fanuc 标准)

typedef struct {
    char text[MAX_PROGRAM_LINE_LEN];  // 行文本 (含 '\0')
    int  line_no;                      // 1-based 源文件行号 (报错用, 含空行/注释)
} ProgramLine_t;

typedef struct {
    int label_num;   // N 后的数字 (如 N100 → 100)
    int line_idx;    // 在 lines[] 中的索引 (0-based)
} NLabel_t;

// ---- Phase 2B M5: O 子程序标签 ----
// O<num> 行扫描后存入 o_labels[]
// 主流程 fall-through 到 O 行时, parser 跳到 skip_to (子程序 M99 之后)
// M98 调用时, Program_FindOLabel 查 entry line_idx 跳入
typedef struct {
    int o_number;   // O 后的数字 (如 O0100 → 100)
    int line_idx;   // O 行在 lines[] 中的索引 (子程序入口)
    int skip_to;    // 主流程跳过子程序体的目标行 (M99 行的下一行), -1 表示未配对
} OLabel_t;

typedef struct {
    ProgramLine_t lines[MAX_PROGRAM_LINES];
    int           num_lines;
    NLabel_t      n_labels[MAX_N_LABELS];
    int           num_n_labels;
    // ---- Phase 2B M3: WHILE/DO/END 配对表 ----
    // do_to_end[i] = j 表示 lines[i] 是 WHILE/DO 行, 对应 END 在 lines[j]
    // end_to_do[i] = j 表示 lines[i] 是 END 行, 对应 DO 在 lines[j]
    // 非循环行: -1
    int do_to_end[MAX_PROGRAM_LINES];
    int end_to_do[MAX_PROGRAM_LINES];
    // ---- Phase 2B M5: O 子程序标签表 ----
    OLabel_t o_labels[MAX_O_LABELS];
    int      num_o_labels;
} GCodeProgram_t;

// 加载文件到内存 (全文 fgets 一次性读入 + N 标签预扫描)
// 失败返回 NULL 并打印错误; 成功返回静态单例指针
GCodeProgram_t* Program_Load(const char *filepath);

// 重置程序状态 (静态分配, 仅清零计数, 不真正 free)
void Program_Free(GCodeProgram_t *prog);

// 按 N 标签号查行索引, 找不到返回 -1
int Program_FindLabel(const GCodeProgram_t *prog, int label_num);

// Phase 2B M5: 按 O 子程序号查 o_labels[] 索引 (不是 line_idx), 找不到返回 -1
// 调用方拿到索引后用 prog->o_labels[idx].line_idx 取子程序入口行
int Program_FindOLabel(const GCodeProgram_t *prog, int o_num);

#endif // PROGRAM_LOADER_H
