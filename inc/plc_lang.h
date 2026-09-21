#ifndef PLC_LANG_H
#define PLC_LANG_H

/* =====================================================================
 *  plc_lang.h —— P1 软 PLC ST 子集语言层 (词法/语法/语义)
 *
 *  语言定义见 docs/P1_SOFTPLC_DESIGN_SPEC.md §3:
 *    单 POU = { VAR 块 } + 顶层语句序列
 *    类型: BOOL / INT / DINT / REAL + FB 实例 (TON/TOF/CTU/R_TRIG/F_TRIG)
 *    语句: 赋值 (含 bit 选位) / IF-ELSIF-ELSE / WHILE / FB 调用
 *    表达式: OR XOR AND NOT, = <> < <= > >=, + - * / MOD, 一元 -
 *    函数: MOVE ABS MIN MAX LIMIT
 *    注释: // 与 (* *) (不嵌套); 大小写不敏感
 *
 *  并发模型:
 *    编译发生在 RPC handler 线程 (Non-RT, malloc 允许);
 *    产物 PlcProgram_t 由 plc_core 独占持有, 换装仅在 stop 态原子替换。
 *    本模块不做任何锁 — 调用方 (plc_core) 保证单线程访问。
 *
 *  语义红线 (编译期强制):
 *    G_* 系统变量只读 (出现在赋值左侧 = 编译错误)
 *    F_* 系统变量只写 (出现在表达式右侧 = 编译错误)
 *    BOOL 不参与算术; 数值隐式转换允许 (运行期钳制)
 * ===================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 容量上限 ---- */
#define PLC_NAME_MAX        32    /* 标识符长度 (含 \0) */
#define PLC_USER_VAR_MAX    128   /* 用户变量上限 (含 FB 实例) */
#define PLC_SYS_VAR_MAX     24    /* 系统变量表容量 (实际 ~16, 留余量) */
#define PLC_DIAG_MSG_LEN    64    /* 诊断消息长度 (含 \0) */
#define PLC_DIAG_MAX        32    /* 单次编译诊断收集上限 */

/* ---- 基本类型 ---- */
typedef enum {
    PLC_T_BOOL = 0, PLC_T_INT, PLC_T_DINT, PLC_T_REAL
} PlcType;

typedef enum {
    PLC_FB_NONE = 0, PLC_FB_TON, PLC_FB_TOF, PLC_FB_CTU,
    PLC_FB_R_TRIG, PLC_FB_F_TRIG
} PlcFbKind;

/* ---- 变量类 (决定可读/可写) ---- */
typedef enum {
    PLC_CLS_USER = 0,   /* 用户声明, 可读写 */
    PLC_CLS_SYS_G,      /* G 区只读 (CNC→PLC) */
    PLC_CLS_SYS_F       /* F 区只写 (PLC→CNC) */
} PlcVarClass;

/* ---- 编译期/运行期共用值 (literal + 变量初值) ---- */
typedef struct {
    uint8_t type;              /* PlcType */
    union { int32_t i; double d; } v;   /* BOOL/INT/DINT 用 i, REAL 用 d */
} PlcVal;

/* ---- FB 输出字段 (ident.field 访问) ---- */
typedef enum { PLC_FF_NONE = 0, PLC_FF_Q, PLC_FF_CV, PLC_FF_ET } PlcFbField;

/* ---- FB 调用参数名 ---- */
typedef enum {
    PLC_FP_NONE = 0, PLC_FP_IN, PLC_FP_PT,
    PLC_FP_CU, PLC_FP_PV, PLC_FP_R, PLC_FP_CLK
} PlcFbParam;

/* =====================================================================
 *  AST
 * ===================================================================== */
typedef struct PlcExpr PlcExpr_t;

typedef enum {
    XE_LIT = 0,   /* 字面量 */
    XE_VAR,       /* 变量引用 */
    XE_BIT,       /* 变量位选 (INT/DINT 变量 . 0-15) */
    XE_FIELD,     /* FB 输出字段 (.Q/.CV/.ET) */
    XE_BIN,       /* 双目 */
    XE_UN,        /* 单目 (NOT/NEG) */
    XE_FUNC       /* 内置函数 */
} PlcExprKind;

typedef enum {
    XO_OR = 0, XO_XOR, XO_AND,
    XO_EQ, XO_NE, XO_LT, XO_LE, XO_GT, XO_GE,
    XO_ADD, XO_SUB, XO_MUL, XO_DIV, XO_MOD
} PlcBinOp;

typedef enum { XU_NOT = 0, XU_NEG } PlcUnOp;

typedef enum { XF_MOVE = 0, XF_ABS, XF_MIN, XF_MAX, XF_LIMIT } PlcFuncKind;

struct PlcExpr {
    uint8_t  kind;             /* PlcExprKind */
    uint8_t  type;             /* PlcType 结果类型 (语义分析填充) */
    uint16_t line;             /* 源码行 (诊断/运行期错误定位) */
    union {
        PlcVal  lit;
        struct { int16_t var_idx; } vr;
        struct { int16_t var_idx; uint8_t bit; } bit;
        struct { int16_t var_idx; uint8_t field; } fld;   /* PlcFbField */
        struct { uint8_t op; PlcExpr_t *l, *r; } bin;     /* PlcBinOp */
        struct { uint8_t op; PlcExpr_t *e; } un;          /* PlcUnOp */
        struct { uint8_t fn; PlcExpr_t *a[3]; uint8_t n; } func;
    } u;
};

/* ---- 语句 (单链表) ---- */
typedef struct PlcStmt PlcStmt_t;

typedef struct { PlcExpr_t *cond; PlcStmt_t *body; } PlcIfBranch_t;

typedef struct { uint8_t param; PlcExpr_t *val; } PlcFbArg_t;   /* param = PlcFbParam */

typedef enum {
    PS_ASSIGN = 0, PS_IF, PS_WHILE, PS_FBCALL
} PlcStmtKind;

struct PlcStmt {
    uint8_t  kind;             /* PlcStmtKind */
    uint16_t line;
    union {
        /* 赋值: bit < 0 = 整变量; >= 0 = var.bit 位选 (var 须 INT/DINT) */
        struct { int16_t var_idx; int8_t bit; PlcExpr_t *rhs; } as;
        struct {
            PlcIfBranch_t  b_then;
            PlcIfBranch_t *elsif;    /* 数组, n_elsif 个 */
            int16_t        n_elsif;
            PlcStmt_t     *body_else;
        } ifs;
        struct { PlcExpr_t *cond; PlcStmt_t *body; } wh;
        struct { int16_t var_idx; PlcFbArg_t args[7]; int n_args; } fb;
    } u;
    PlcStmt_t *next;
};

/* ---- 变量表槽 ---- */
typedef struct {
    char        name[PLC_NAME_MAX];   /* 大写规范化 */
    uint8_t     type;                 /* PlcType (FB 实例时无意义) */
    uint8_t     fb;                   /* PlcFbKind (非 NONE = FB 实例) */
    uint8_t     cls;                  /* PlcVarClass */
    PlcVal      init;                 /* 用户变量初值 (FB 实例无) */
} PlcVarSlot_t;

/* ---- 编译产物 ---- */
typedef struct {
    int          n_vars;                       /* 系统 (前缀) + 用户 */
    int          n_user_vars;
    PlcVarSlot_t vars[PLC_SYS_VAR_MAX + PLC_USER_VAR_MAX];
    PlcStmt_t   *stmts;                        /* 顶层语句链 */
    int          n_stmts;
    /* 分配登记 (防编译失败泄漏): 所有 AST 节点经 arena 登记,
     * PlcProgramFree 按登记表整体释放, 无需遍历树 */
    void       **allocs;
    int          n_allocs;
} PlcProgram_t;

/* ---- 诊断 ---- */
typedef struct {
    int32_t line;
    char    msg[PLC_DIAG_MSG_LEN];
} PlcDiag_t;

/* =====================================================================
 *  API
 * ===================================================================== */

/* @Context: Non-RealTime (RPC handler 线程 / plc_core)
 * @Safe: malloc + 解析。不修改任何全局状态 (除 malloc 堆)。
 * @param src/src_len  程序文本 (不要求 NUL 终结)
 * @param out_prog     编译成功时输出堆上产物 (调用方用 PlcProgramFree 释放)
 * @param diags        诊断输出缓冲 (可为 NULL, 则不收集)
 * @param diag_max     diags 容量
 * @param n_diag       实际诊断数 (含被截断计数, 总数不超 PLC_DIAG_MAX)
 * @return 0=编译成功 (out_prog 有效); -1=有错误 (out_prog 不产生) */
int  PlcCompile(const char *src, int src_len,
                PlcProgram_t **out_prog,
                PlcDiag_t *diags, int diag_max, int *n_diag);

/* @Context: Non-RealTime
 * @Safe: 递归 free AST + 产物本体。NULL 安全。 */
void PlcProgramFree(PlcProgram_t *prog);

/* 系统变量表条目数 (plc_core 建 G/F 绑定时用) */
int  PlcSysVarCount(void);
/* 按变量序号取系统变量名 (idx < PlcSysVarCount), 越界返回 NULL */
const char *PlcSysVarName(int idx);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PLC_LANG_H */
