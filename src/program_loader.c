#include "program_loader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// @Context: Non-RealTime Background Thread (parser_thread_func 启动时调用)
// @Safe: malloc-free (静态 5MB 全局缓冲), fgets 阻塞 I/O, 仅 parser 线程访问无需锁

static GCodeProgram_t s_program;  // 静态单例

// ---- Phase 2B M3: WHILE/DO/END 栈式配对预扫描 ----
// 算法: 扫描所有行, WHILE 行 push 栈, END 行 pop 配对, 栈非空报错
// 失败返回 -1, 调用方据此让 Program_Load 返回 NULL
static int scan_while_pairs(GCodeProgram_t *prog)
{
    // 初始化为 -1 (表示非循环行)
    for(int i = 0; i < prog->num_lines; i++){
        prog->do_to_end[i] = -1;
        prog->end_to_do[i] = -1;
    }

    int stack_line[MAX_WHILE_NESTING];  // DO 行索引
    int stack_n[MAX_WHILE_NESTING];     // DO 编号
    int top = 0;

    for(int i = 0; i < prog->num_lines; i++){
        const char *p = prog->lines[i].text;
        while(*p == ' ' || *p == '\t') p++;

        // 检测 WHILE 关键字 (行首)
        if(toupper((unsigned char)p[0])=='W' && toupper((unsigned char)p[1])=='H' &&
           toupper((unsigned char)p[2])=='I' && toupper((unsigned char)p[3])=='L' &&
           toupper((unsigned char)p[4])=='E' && !isalpha((unsigned char)p[5])){
            // 走过 WHILE 后, 跳过 [...] 条件块, 找 "DO <n>"
            const char *q = p + 5;
            while(*q == ' ' || *q == '\t') q++;
            if(*q != '['){
                printf("[Loader] 错误: WHILE 后缺 '[' (行 %d)\n",
                       prog->lines[i].line_no);
                return -1;
            }
            // 找匹配 ]
            int depth = 1; q++;
            while(*q && depth > 0){
                if(*q == '[') depth++;
                else if(*q == ']') depth--;
                q++;
            }
            if(depth != 0){
                printf("[Loader] 错误: WHILE 条件缺 ']' (行 %d)\n",
                       prog->lines[i].line_no);
                return -1;
            }
            while(*q == ' ' || *q == '\t') q++;
            if(toupper((unsigned char)q[0]) != 'D' ||
               toupper((unsigned char)q[1]) != 'O'){
                printf("[Loader] 错误: WHILE 行 %d 缺 DO\n",
                       prog->lines[i].line_no);
                return -1;
            }
            q += 2;
            char *end;
            long n = strtol(q, &end, 10);
            if(end == q){
                printf("[Loader] 错误: DO 后缺编号 (行 %d)\n",
                       prog->lines[i].line_no);
                return -1;
            }
            if(top >= MAX_WHILE_NESTING){
                printf("[Loader] 错误: WHILE 嵌套深度超 %d (行 %d)\n",
                       MAX_WHILE_NESTING, prog->lines[i].line_no);
                return -1;
            }
            stack_line[top] = i;
            stack_n[top]    = (int)n;
            top++;
            continue;
        }

        // 检测 END 关键字 (行首, 后必须非字母防 ENDX 误匹配)
        if(toupper((unsigned char)p[0])=='E' && toupper((unsigned char)p[1])=='N' &&
           toupper((unsigned char)p[2])=='D' && !isalpha((unsigned char)p[3])){
            char *end;
            long n = strtol(p + 3, &end, 10);
            if(end == p + 3){
                printf("[Loader] 错误: END 后缺编号 (行 %d)\n",
                       prog->lines[i].line_no);
                return -1;
            }
            if(top == 0){
                printf("[Loader] 错误: END %ld 无匹配 DO (行 %d)\n",
                       n, prog->lines[i].line_no);
                return -1;
            }
            top--;
            if(stack_n[top] != (int)n){
                printf("[Loader] 错误: DO %d 与 END %ld 编号不匹配 (DO 行 %d, END 行 %d)\n",
                       stack_n[top], n,
                       prog->lines[stack_line[top]].line_no,
                       prog->lines[i].line_no);
                return -1;
            }
            prog->do_to_end[stack_line[top]] = i;
            prog->end_to_do[i] = stack_line[top];
        }
    }

    if(top != 0){
        printf("[Loader] 错误: %d 个 DO 未闭合 (首个 DO 在行 %d)\n",
               top, prog->lines[stack_line[0]].line_no);
        return -1;
    }

    int pair_count = 0;
    for(int i = 0; i < prog->num_lines; i++){
        if(prog->do_to_end[i] >= 0) pair_count++;
    }
    printf("[Loader] WHILE/DO/END 配对完成: %d 对\n", pair_count);
    return 0;
}

// ---- Phase 2B M5: O/M99 子程序块栈式配对预扫描 ----
// 第 4 遍扫描 (在 scan_while_pairs 之后)
// 算法: O<num> 行 push 栈 + 追加 o_labels[], M99 行 pop 配对填 skip_to
// 重复 O 编号报错; 栈非空 EOF 报错; M99 行首深度 0 视为主程序 M99 (沿用)
// 失败返回 -1, 调用方据此让 Program_Load 返回 NULL
static int scan_o_blocks(GCodeProgram_t *prog)
{
    int stack_o_idx[MAX_O_LABELS];   // 栈元素: 对应 o_labels[] 中的下标
    int top = 0;

    for(int i = 0; i < prog->num_lines; i++){
        const char *p = prog->lines[i].text;
        while(*p == ' ' || *p == '\t') p++;

        // 检测 O<num> (大小写不敏感, O 后必须紧跟数字)
        if(toupper((unsigned char)*p) == 'O' && isdigit((unsigned char)p[1])){
            char *end;
            long o_num = strtol(p + 1, &end, 10);
            if(end == p + 1){
                printf("[Loader] 错误: O 后缺编号 (行 %d)\n",
                       prog->lines[i].line_no);
                return -1;
            }
            if(o_num < 1 || o_num > 99999){
                printf("[Loader] 错误: O 编号 %ld 越界 (1..99999, 行 %d)\n",
                       o_num, prog->lines[i].line_no);
                return -1;
            }
            // 重复检查
            for(int j = 0; j < prog->num_o_labels; j++){
                if(prog->o_labels[j].o_number == (int)o_num){
                    printf("[Loader] 错误: O%ld 重复定义 (源行 %d 已有)\n",
                           o_num, prog->lines[prog->o_labels[j].line_idx].line_no);
                    return -1;
                }
            }
            if(prog->num_o_labels >= MAX_O_LABELS){
                printf("[Loader] 错误: O 标签数超 %d (行 %d)\n",
                       MAX_O_LABELS, prog->lines[i].line_no);
                return -1;
            }
            // 追加 + 入栈
            int new_idx = prog->num_o_labels;
            prog->o_labels[new_idx].o_number = (int)o_num;
            prog->o_labels[new_idx].line_idx = i;
            prog->o_labels[new_idx].skip_to  = -1;
            prog->num_o_labels++;
            if(top >= MAX_O_LABELS){
                printf("[Loader] 错误: O 块嵌套深度超 %d (行 %d)\n",
                       MAX_O_LABELS, prog->lines[i].line_no);
                return -1;
            }
            stack_o_idx[top++] = new_idx;
            continue;
        }

        // 检测 M99 (大小写不敏感, M99 后必须非字母)
        if(toupper((unsigned char)p[0])=='M' &&
           toupper((unsigned char)p[1])=='9' &&
           toupper((unsigned char)p[2])=='9' &&
           !isalpha((unsigned char)p[3])){
            if(top > 0){
                int paired_idx = stack_o_idx[--top];
                prog->o_labels[paired_idx].skip_to = i + 1;
            }
            // top == 0: 主程序 M99, 预扫描忽略 (parser 处理主程序重启)
        }
    }

    if(top != 0){
        int first_idx = stack_o_idx[0];
        printf("[Loader] 错误: O%d 未匹配 M99 (源行 %d)\n",
               prog->o_labels[first_idx].o_number,
               prog->lines[prog->o_labels[first_idx].line_idx].line_no);
        return -1;
    }

    printf("[Loader] O/M99 子程序配对完成: %d 个\n", prog->num_o_labels);
    return 0;
}

GCodeProgram_t* Program_Load(const char *filepath)
{
    FILE *fp = fopen(filepath, "r");
    if(!fp){
        printf("[Loader] 无法打开文件: %s\n", filepath);
        return NULL;
    }

    s_program.num_lines = 0;
    s_program.num_n_labels = 0;
    s_program.num_o_labels = 0;

    char raw[MAX_PROGRAM_LINE_LEN];
    int src_line_no = 0;
    while(fgets(raw, sizeof(raw), fp) != NULL){
        src_line_no++;
        if(s_program.num_lines >= MAX_PROGRAM_LINES){
            printf("[Loader] 文件超过 %d 行上限, 后续行被截断\n", MAX_PROGRAM_LINES);
            break;
        }
        strncpy(s_program.lines[s_program.num_lines].text, raw,
                MAX_PROGRAM_LINE_LEN - 1);
        s_program.lines[s_program.num_lines].text[MAX_PROGRAM_LINE_LEN - 1] = '\0';
        s_program.lines[s_program.num_lines].line_no = src_line_no;
        s_program.num_lines++;
    }
    fclose(fp);

    // ---- 预扫描 N 标签 (line_start 形如 "N100 ..." 才有效) ----
    for(int i = 0; i < s_program.num_lines; i++){
        const char *p = s_program.lines[i].text;
        while(*p == ' ' || *p == '\t') p++;
        if(*p != 'N' && *p != 'n') continue;
        p++;
        char *end;
        long label = strtol(p, &end, 10);
        if(end == p) continue;  // N 后非数字, 不是有效标签

        if(s_program.num_n_labels >= MAX_N_LABELS){
            printf("[Loader] N 标签数超过 %d 上限, 后续标签忽略\n", MAX_N_LABELS);
            break;
        }
        // 重复检查 (首个生效, 后续警告忽略)
        int dup = 0;
        for(int j = 0; j < s_program.num_n_labels; j++){
            if(s_program.n_labels[j].label_num == (int)label){
                printf("[Loader] 警告: N%d 重复定义 (源行 %d 已有, 源行 %d 忽略)\n",
                       (int)label,
                       s_program.lines[s_program.n_labels[j].line_idx].line_no,
                       s_program.lines[i].line_no);
                dup = 1;
                break;
            }
        }
        if(dup) continue;

        s_program.n_labels[s_program.num_n_labels].label_num = (int)label;
        s_program.n_labels[s_program.num_n_labels].line_idx  = i;
        s_program.num_n_labels++;
    }

    // ---- Phase 2B M3: 预扫描 WHILE/DO/END 配对 (失败拒绝运行) ----
    if(scan_while_pairs(&s_program) < 0){
        printf("[Loader] WHILE/END 配对失败, 拒绝运行 (源文件: %s)\n", filepath);
        s_program.num_lines = 0;
        s_program.num_n_labels = 0;
        s_program.num_o_labels = 0;
        return NULL;
    }

    // ---- Phase 2B M5: 预扫描 O/M99 子程序配对 (失败拒绝运行) ----
    if(scan_o_blocks(&s_program) < 0){
        printf("[Loader] O/M99 配对失败, 拒绝运行 (源文件: %s)\n", filepath);
        s_program.num_lines = 0;
        s_program.num_n_labels = 0;
        s_program.num_o_labels = 0;
        return NULL;
    }

    printf("[Loader] 加载完成: %d 行, %d 个 N 标签, %d 个 O 子程序 (源文件: %s)\n",
           s_program.num_lines, s_program.num_n_labels, s_program.num_o_labels, filepath);
    return &s_program;
}

void Program_Free(GCodeProgram_t *prog)
{
    if(prog){
        prog->num_lines = 0;
        prog->num_n_labels = 0;
        prog->num_o_labels = 0;
    }
}

// 线性查找: N 标签典型 < 100 个, 比二分查找代码更简单可靠
int Program_FindLabel(const GCodeProgram_t *prog, int label_num)
{
    if(!prog) return -1;
    for(int i = 0; i < prog->num_n_labels; i++){
        if(prog->n_labels[i].label_num == label_num){
            return prog->n_labels[i].line_idx;
        }
    }
    return -1;
}

// Phase 2B M5: 线性查找 O 子程序号, 返回 o_labels[] 索引 (非 line_idx)
int Program_FindOLabel(const GCodeProgram_t *prog, int o_num)
{
    if(!prog) return -1;
    for(int i = 0; i < prog->num_o_labels; i++){
        if(prog->o_labels[i].o_number == o_num){
            return i;
        }
    }
    return -1;
}
