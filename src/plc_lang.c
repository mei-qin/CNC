/* =====================================================================
 *  plc_lang.c —— P1 软 PLC ST 子集语言层实现 (词法/语法/语义)
 *
 *  设计见 inc/plc_lang.h 头注释 + docs/P1_SOFTPLC_DESIGN_SPEC.md §3。
 *
 *  实现要点:
 *    - 手写词法器 (不依赖 NUL 终结, 按 src_len 界定) + 递归下降语法
 *    - 语义检查内联在语法构建时 (VAR 块先于语句 → 符号表构造完成后再查)
 *    - 所有 AST 节点经 arena 登记; 编译失败整体回收, 成功后移交 PlcProgram_t
 *    - 诊断中文, 定位到行; 上限 PLC_DIAG_MAX 条后熔断
 *    - 语句级错误恢复: 跳到 ';' 或下一语句关键字, 继续收集后续错误
 * ===================================================================== */

#include "plc_lang.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

/* =====================================================================
 *  系统变量表 (顺序 = 变量表前缀序号, plc_core 建 G/F 绑定依赖此序)
 * ===================================================================== */
typedef struct {
    const char *name;
    uint8_t     type;   /* PlcType */
    uint8_t     cls;    /* PlcVarClass */
} PlcSysVarDef_t;

static const PlcSysVarDef_t g_sys_vars[] = {
    {"G_RUNNING",         PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_HOLD",            PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_ALARM",           PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_PAUSED",          PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_WAIT_MCODE",      PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_LASER_KILL",      PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_MCODE",           PLC_T_INT,  PLC_CLS_SYS_G},
    {"G_MCODE_STROBE",    PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_SPINDLE_MODE",    PLC_T_INT,  PLC_CLS_SYS_G},
    {"G_COOLANT_STATE",   PLC_T_INT,  PLC_CLS_SYS_G},
    {"G_LASER_GATE",      PLC_T_BOOL, PLC_CLS_SYS_G},
    {"G_LASER_INTERLOCK", PLC_T_INT,  PLC_CLS_SYS_G},
    {"G_UPTIME_MS",       PLC_T_DINT, PLC_CLS_SYS_G},
    /* P1.5-1 (2026-09-14): AI 模拟输入工程量 (sim 注入/硬件采集同源) */
    {"G_AI0",             PLC_T_REAL, PLC_CLS_SYS_G},
    {"G_AI1",             PLC_T_REAL, PLC_CLS_SYS_G},
    {"G_AI2",             PLC_T_REAL, PLC_CLS_SYS_G},
    {"G_AI3",             PLC_T_REAL, PLC_CLS_SYS_G},
    {"F_PERIPHERAL_OK",   PLC_T_INT,  PLC_CLS_SYS_F},
    {"F_MCODE_FIN",       PLC_T_INT,  PLC_CLS_SYS_F},
    {"F_STATUS",          PLC_T_INT,  PLC_CLS_SYS_F},
};

int PlcSysVarCount(void)
{
    return (int)(sizeof(g_sys_vars) / sizeof(g_sys_vars[0]));
}

const char *PlcSysVarName(int idx)
{
    if (idx < 0 || idx >= PlcSysVarCount()) return NULL;
    return g_sys_vars[idx].name;
}

/* ---- 保留字 (不可作变量名) ---- */
static const char *g_reserved[] = {
    "VAR", "END_VAR", "IF", "THEN", "ELSIF", "ELSE", "END_IF",
    "WHILE", "DO", "END_WHILE",
    "BOOL", "INT", "DINT", "REAL",
    "TON", "TOF", "CTU", "R_TRIG", "F_TRIG",
    "TRUE", "FALSE", "AND", "OR", "XOR", "NOT", "MOD",
    "MOVE", "ABS", "MIN", "MAX", "LIMIT",
    NULL
};

/* =====================================================================
 *  诊断 + arena
 * ===================================================================== */
typedef struct {
    void **ptrs;
    int    n, cap;
} Arena;

static void *arena_alloc(Arena *a, size_t sz)
{
    if (a->n >= a->cap) {
        int ncap = (a->cap == 0) ? 128 : a->cap * 2;
        void **np = (void **)realloc(a->ptrs, (size_t)ncap * sizeof(void *));
        if (np == NULL) return NULL;
        a->ptrs = np;
        a->cap = ncap;
    }
    void *p = calloc(1, sz);
    if (p == NULL) return NULL;
    a->ptrs[a->n++] = p;
    return p;
}

/* =====================================================================
 *  词法
 * ===================================================================== */
enum {
    TK_EOF = 0, TK_IDENT, TK_INT, TK_REAL, TK_SYMB
};

typedef struct {
    /* 源 */
    const char *src; int len; int pos; int line;
    /* 当前 token */
    int    tk;
    char   txt[PLC_NAME_MAX + 8];   /* 原文 (ident/符号) */
    char   up[PLC_NAME_MAX + 8];    /* 大写 (关键字/查表) */
    PlcVal val;                     /* TK_INT/TK_REAL 字面值 */
    /* 产物/诊断 */
    PlcProgram_t *prog;
    PlcDiag_t    *diags;            /* 内部全量缓冲 (PLC_DIAG_MAX) */
    int           n_diag;
    int           fatal;
    Arena         arena;
} P;

static void pl_diag(P *p, int line, const char *fmt, ...)
{
    if (p->n_diag >= PLC_DIAG_MAX) {
        p->fatal = 1;
        return;
    }
    va_list ap;
    PlcDiag_t *d = &p->diags[p->n_diag++];
    d->line = line;
    va_start(ap, fmt);
    vsnprintf(d->msg, PLC_DIAG_MSG_LEN, fmt, ap);
    va_end(ap);
}

/* 前瞻字符 */
static int pl_peek(P *p)
{
    return (p->pos < p->len) ? (unsigned char)p->src[p->pos] : -1;
}

/* 取下一个 token 到 p->tk/txt/up/val; 注释与空白在此跳过 */
static void pl_next(P *p)
{
    /* 跳空白 + 注释 */
    for (;;) {
        int c = pl_peek(p);
        if (c == ' ' || c == '\t' || c == '\r') { p->pos++; continue; }
        if (c == '\n') { p->pos++; p->line++; continue; }
        if (c == '/' && p->pos + 1 < p->len && p->src[p->pos + 1] == '/') {
            while (p->pos < p->len && p->src[p->pos] != '\n') p->pos++;
            continue;
        }
        if (c == '(' && p->pos + 1 < p->len && p->src[p->pos + 1] == '*') {
            int start_line = p->line;
            p->pos += 2;
            int closed = 0;
            while (p->pos < p->len) {
                if (p->src[p->pos] == '*' && p->pos + 1 < p->len
                    && p->src[p->pos + 1] == ')') { p->pos += 2; closed = 1; break; }
                if (p->src[p->pos] == '\n') p->line++;
                p->pos++;
            }
            if (!closed) {
                p->tk = TK_EOF;
                pl_diag(p, start_line, "注释 (* *) 未闭合");
                p->fatal = 1;
                return;
            }
            continue;
        }
        break;
    }

    int c = pl_peek(p);
    if (c < 0) { p->tk = TK_EOF; return; }

    /* 标识符 / 关键字 */
    if (isalpha(c) || c == '_') {
        int n = 0;
        while (p->pos < p->len && (isalnum((unsigned char)p->src[p->pos])
               || p->src[p->pos] == '_')) {
            if (n < PLC_NAME_MAX + 7) p->txt[n++] = p->src[p->pos];
            p->pos++;
        }
        p->txt[n] = '\0';
        for (int i = 0; i <= n; i++)
            p->up[i] = (char)toupper((unsigned char)p->txt[i]);
        p->tk = TK_IDENT;
        return;
    }

    /* 数字 (无符号; 负号走一元) */
    if (isdigit(c)) {
        int is_real = 0, n = 0;
        while (p->pos < p->len && isdigit((unsigned char)p->src[p->pos])) {
            if (n < PLC_NAME_MAX + 7) p->txt[n++] = p->src[p->pos];
            p->pos++;
        }
        if (pl_peek(p) == '.') {
            is_real = 1;
            if (n < PLC_NAME_MAX + 7) p->txt[n++] = '.';
            p->pos++;
            while (p->pos < p->len && isdigit((unsigned char)p->src[p->pos])) {
                if (n < PLC_NAME_MAX + 7) p->txt[n++] = p->src[p->pos];
                p->pos++;
            }
        }
        if (pl_peek(p) == 'e' || pl_peek(p) == 'E') {
            is_real = 1;
            if (n < PLC_NAME_MAX + 7) p->txt[n++] = 'e';
            p->pos++;
            if (pl_peek(p) == '+' || pl_peek(p) == '-') {
                if (n < PLC_NAME_MAX + 7) p->txt[n++] = (char)pl_peek(p);
                p->pos++;
            }
            while (p->pos < p->len && isdigit((unsigned char)p->src[p->pos])) {
                if (n < PLC_NAME_MAX + 7) p->txt[n++] = p->src[p->pos];
                p->pos++;
            }
        }
        p->txt[n] = '\0';
        if (is_real) {
            p->tk = TK_REAL;
            p->val.type = PLC_T_REAL;
            p->val.v.d = strtod(p->txt, NULL);
        } else {
            long v = strtol(p->txt, NULL, 10);
            p->tk = TK_INT;
            p->val.type = (v > 32767 || v < -32768) ? PLC_T_DINT : PLC_T_INT;
            p->val.v.i = (int32_t)v;
        }
        return;
    }

    /* 符号 (先两字符) */
    if (c == ':' && p->pos + 1 < p->len && p->src[p->pos + 1] == '=') {
        p->pos += 2; strcpy(p->txt, ":="); strcpy(p->up, ":=");
        p->tk = TK_SYMB; return;
    }
    if (c == '<' && p->pos + 1 < p->len
        && (p->src[p->pos + 1] == '=' || p->src[p->pos + 1] == '>')) {
        p->txt[0] = (char)c; p->txt[1] = p->src[p->pos + 1]; p->txt[2] = '\0';
        strcpy(p->up, p->txt);
        p->pos += 2; p->tk = TK_SYMB; return;
    }
    if (c == '>' && p->pos + 1 < p->len && p->src[p->pos + 1] == '=') {
        p->txt[0] = '>'; p->txt[1] = '='; p->txt[2] = '\0';
        strcpy(p->up, p->txt);
        p->pos += 2; p->tk = TK_SYMB; return;
    }
    if (strchr(":;,().+-*/=<>", c)) {
        p->txt[0] = (char)c; p->txt[1] = '\0';
        strcpy(p->up, p->txt);
        p->pos++; p->tk = TK_SYMB; return;
    }

    p->tk = TK_EOF;
    pl_diag(p, p->line, "非法字符 '%c' (0x%02X)", isprint(c) ? c : '?', c);
    p->fatal = 1;
}

/* 关键字匹配 (当前 token 是 IDENT 且大写等于 kw) */
static int kw(P *p, const char *k)
{
    return p->tk == TK_IDENT && strcmp(p->up, k) == 0;
}

/* 符号匹配 */
static int sym(P *p, const char *s)
{
    return p->tk == TK_SYMB && strcmp(p->txt, s) == 0;
}

static int accept_sym(P *p, const char *s)
{
    if (sym(p, s)) { pl_next(p); return 1; }
    return 0;
}

static int expect_sym(P *p, const char *s)
{
    if (accept_sym(p, s)) return 1;
    pl_diag(p, p->line, "期望 '%s', 实际 %s%s%s",
            s, (p->tk == TK_EOF) ? "文件结束" : "'",
            (p->tk == TK_EOF) ? "" : p->txt,
            (p->tk == TK_EOF) ? "" : "'");
    return 0;
}

/* =====================================================================
 *  符号表
 * ===================================================================== */
static int var_find(P *p, const char *upper_name)
{
    for (int i = 0; i < p->prog->n_vars; i++)
        if (strcmp(p->prog->vars[i].name, upper_name) == 0) return i;
    return -1;
}

/* =====================================================================
 *  表达式 (优先级爬升; 语义内联; 失败返回 NULL)
 * ===================================================================== */
static PlcExpr_t *parse_expr(P *p);

static int t_numeric(uint8_t t) { return t != PLC_T_BOOL && t != 0xFF; }

static uint8_t t_arith(uint8_t a, uint8_t b)
{
    if (!t_numeric(a) || !t_numeric(b)) return 0xFF;
    if (a == PLC_T_REAL || b == PLC_T_REAL) return PLC_T_REAL;
    if (a == PLC_T_DINT || b == PLC_T_DINT) return PLC_T_DINT;
    return PLC_T_INT;
}

static PlcExpr_t *mk_expr(P *p, uint8_t kind)
{
    PlcExpr_t *e = (PlcExpr_t *)arena_alloc(&p->arena, sizeof(PlcExpr_t));
    if (e == NULL) { p->fatal = 1; return NULL; }
    e->kind = kind;
    e->line = (uint16_t)p->line;
    return e;
}

static PlcExpr_t *mk_lit(P *p, PlcVal v)
{
    PlcExpr_t *e = mk_expr(p, XE_LIT);
    if (e == NULL) return NULL;
    e->type = v.type;
    e->u.lit = v;
    return e;
}

static PlcExpr_t *mk_bin(P *p, uint8_t op, PlcExpr_t *l, PlcExpr_t *r)
{
    if (l == NULL || r == NULL) return NULL;
    PlcExpr_t *e = mk_expr(p, XE_BIN);
    if (e == NULL) return NULL;
    e->u.bin.op = op;
    e->u.bin.l = l;
    e->u.bin.r = r;

    switch (op) {
    case XO_OR: case XO_XOR: case XO_AND:
        if (l->type != PLC_T_BOOL || r->type != PLC_T_BOOL) {
            pl_diag(p, e->line, "逻辑运算符 AND/OR/XOR 要求 BOOL 操作数");
            return NULL;
        }
        e->type = PLC_T_BOOL;
        break;
    case XO_EQ: case XO_NE:
        if ((l->type == PLC_T_BOOL) != (r->type == PLC_T_BOOL)) {
            pl_diag(p, e->line, "= / <> 两侧须同为 BOOL 或同为数值");
            return NULL;
        }
        e->type = PLC_T_BOOL;
        break;
    case XO_LT: case XO_LE: case XO_GT: case XO_GE:
        if (!t_numeric(l->type) || !t_numeric(r->type)) {
            pl_diag(p, e->line, "比较运算符要求数值操作数");
            return NULL;
        }
        e->type = PLC_T_BOOL;
        break;
    default: {    /* 算术 */
        uint8_t t = t_arith(l->type, r->type);
        if (t == 0xFF) {
            pl_diag(p, e->line, "算术运算符要求数值操作数 (BOOL 不参与)");
            return NULL;
        }
        if ((op == XO_DIV || op == XO_MOD)
            && r->kind == XE_LIT && t_numeric(r->type)
            && (r->type == PLC_T_REAL ? r->u.lit.v.d == 0.0
                                      : r->u.lit.v.i == 0)) {
            pl_diag(p, e->line, "除数/MOD 为零字面量");
            return NULL;
        }
        e->type = t;
        break;
    }
    }
    return e;
}

static PlcExpr_t *mk_un(P *p, uint8_t op, PlcExpr_t *x)
{
    if (x == NULL) return NULL;
    PlcExpr_t *e = mk_expr(p, XE_UN);
    if (e == NULL) return NULL;
    e->u.un.op = op;
    e->u.un.e = x;
    if (op == XU_NOT) {
        if (x->type != PLC_T_BOOL) {
            pl_diag(p, e->line, "NOT 要求 BOOL 操作数");
            return NULL;
        }
        e->type = PLC_T_BOOL;
    } else {
        if (!t_numeric(x->type)) {
            pl_diag(p, e->line, "一元负号要求数值操作数");
            return NULL;
        }
        e->type = x->type;
    }
    return e;
}

/* primary: 字面量 / 括号 / 函数 / 变量 [.bit|.field] */
static PlcExpr_t *parse_primary(P *p)
{
    if (p->fatal) return NULL;

    if (p->tk == TK_INT || p->tk == TK_REAL) {
        PlcVal v = p->val;
        pl_next(p);
        return mk_lit(p, v);
    }

    if (kw(p, "TRUE") || kw(p, "FALSE")) {
        PlcVal v = { PLC_T_BOOL, {0} };
        v.v.i = kw(p, "TRUE") ? 1 : 0;
        pl_next(p);
        return mk_lit(p, v);
    }

    if (accept_sym(p, "(")) {
        PlcExpr_t *e = parse_expr(p);
        if (e == NULL) return NULL;
        if (!expect_sym(p, ")")) return NULL;
        return e;
    }

    /* 函数 MOVE/ABS/MIN/MAX/LIMIT */
    static const struct { const char *name; uint8_t fn; uint8_t nargs; } funcs[] = {
        {"MOVE", XF_MOVE, 1}, {"ABS", XF_ABS, 1},
        {"MIN", XF_MIN, 2}, {"MAX", XF_MAX, 2}, {"LIMIT", XF_LIMIT, 3},
    };
    for (size_t i = 0; i < sizeof(funcs) / sizeof(funcs[0]); i++) {
        if (!kw(p, funcs[i].name)) continue;
        int line = p->line;
        pl_next(p);
        if (!expect_sym(p, "(")) return NULL;
        PlcExpr_t *e = mk_expr(p, XE_FUNC);
        if (e == NULL) return NULL;
        e->line = (uint16_t)line;
        e->u.func.fn = funcs[i].fn;
        e->u.func.n = funcs[i].nargs;
        for (uint8_t a = 0; a < funcs[i].nargs; a++) {
            PlcExpr_t *x = parse_expr(p);
            if (x == NULL) return NULL;
            e->u.func.a[a] = x;
            if (a + 1 < funcs[i].nargs && !expect_sym(p, ",")) return NULL;
        }
        if (!expect_sym(p, ")")) return NULL;
        /* 类型: 全数值 (LIMIT 结果取最小参数类型) */
        uint8_t t = e->u.func.a[0]->type;
        for (uint8_t a = 0; a < funcs[i].nargs; a++) {
            if (!t_numeric(e->u.func.a[a]->type)) {
                pl_diag(p, line, "%s 参数须为数值", funcs[i].name);
                return NULL;
            }
            t = t_arith(t, e->u.func.a[a]->type);
        }
        e->type = t;
        return e;
    }

    /* 变量引用 */
    if (p->tk == TK_IDENT) {
        char up[PLC_NAME_MAX + 8];
        strncpy(up, p->up, sizeof(up) - 1);
        up[sizeof(up) - 1] = '\0';
        int line = p->line;
        pl_next(p);

        int vi = var_find(p, up);
        if (vi < 0) {
            pl_diag(p, line, "未声明变量 '%s'", up);
            return NULL;
        }
        PlcVarSlot_t *vs = &p->prog->vars[vi];

        if (vs->cls == PLC_CLS_SYS_F) {
            pl_diag(p, line, "F_* 只写变量不可用于表达式 ('%s')", up);
            return NULL;
        }

        if (vs->fb != PLC_FB_NONE) {
            /* FB 实例: 须 .field */
            if (!accept_sym(p, ".")) {
                pl_diag(p, line, "FB 实例 '%s' 须以 .Q/.CV/.ET 访问", up);
                return NULL;
            }
            if (p->tk != TK_IDENT) {
                pl_diag(p, p->line, "FB 字段名非法 (期望 Q/CV/ET)");
                return NULL;
            }
            /* 字段名必须先拷贝 — pl_next 会覆写 lexer 缓冲区 */
            char fup[PLC_NAME_MAX + 8];
            strncpy(fup, p->up, sizeof(fup) - 1);
            fup[sizeof(fup) - 1] = '\0';
            pl_next(p);
            uint8_t field = PLC_FF_NONE;
            if (strcmp(fup, "Q") == 0) field = PLC_FF_Q;
            else if (strcmp(fup, "CV") == 0) field = PLC_FF_CV;
            else if (strcmp(fup, "ET") == 0) field = PLC_FF_ET;
            int ok = 0;
            uint8_t ft = PLC_T_BOOL;
            switch (field) {
            case PLC_FF_Q:
                ok = 1; ft = PLC_T_BOOL; break;                     /* 全部 FB 有 Q */
            case PLC_FF_CV:
                ok = (vs->fb == PLC_FB_CTU); ft = PLC_T_DINT; break;
            case PLC_FF_ET:
                ok = (vs->fb == PLC_FB_TON || vs->fb == PLC_FB_TOF); ft = PLC_T_DINT; break;
            default: ok = 0; break;
            }
            if (!ok) {
                pl_diag(p, line, "FB '%s' 无输出字段 .%s", up, fup);
                return NULL;
            }
            PlcExpr_t *e = mk_expr(p, XE_FIELD);
            if (e == NULL) return NULL;
            e->line = (uint16_t)line;
            e->u.fld.var_idx = (int16_t)vi;
            e->u.fld.field = field;
            e->type = ft;
            return e;
        }

        /* 标量: 可选 .bit */
        if (accept_sym(p, ".")) {
            if (p->tk != TK_INT || p->val.v.i < 0 || p->val.v.i > 15) {
                pl_diag(p, p->line, "位选语法: 变量.0~15 (字面量)");
                return NULL;
            }
            uint8_t bit = (uint8_t)p->val.v.i;
            pl_next(p);
            if (vs->type != PLC_T_INT && vs->type != PLC_T_DINT) {
                pl_diag(p, line, "'%s' 非 INT/DINT, 不支持位选", up);
                return NULL;
            }
            PlcExpr_t *e = mk_expr(p, XE_BIT);
            if (e == NULL) return NULL;
            e->line = (uint16_t)line;
            e->u.bit.var_idx = (int16_t)vi;
            e->u.bit.bit = bit;
            e->type = PLC_T_BOOL;
            return e;
        }

        PlcExpr_t *e = mk_expr(p, XE_VAR);
        if (e == NULL) return NULL;
        e->line = (uint16_t)line;
        e->u.vr.var_idx = (int16_t)vi;
        e->type = vs->type;
        return e;
    }

    pl_diag(p, p->line, "表达式语法错误 (意外的 token)");
    return NULL;
}

static PlcExpr_t *parse_unary(P *p)
{
    if (kw(p, "NOT")) { pl_next(p); return mk_un(p, XU_NOT, parse_unary(p)); }
    if (sym(p, "-"))  { pl_next(p); return mk_un(p, XU_NEG, parse_unary(p)); }
    return parse_primary(p);
}

static PlcExpr_t *parse_mul(P *p)
{
    PlcExpr_t *e = parse_unary(p);
    while (!p->fatal && e != NULL
           && (sym(p, "*") || sym(p, "/") || kw(p, "MOD"))) {
        uint8_t op = sym(p, "*") ? XO_MUL : (sym(p, "/") ? XO_DIV : XO_MOD);
        pl_next(p);
        e = mk_bin(p, op, e, parse_unary(p));
    }
    return e;
}

static PlcExpr_t *parse_add(P *p)
{
    PlcExpr_t *e = parse_mul(p);
    while (!p->fatal && e != NULL && (sym(p, "+") || sym(p, "-"))) {
        uint8_t op = sym(p, "+") ? XO_ADD : XO_SUB;
        pl_next(p);
        e = mk_bin(p, op, e, parse_mul(p));
    }
    return e;
}

static PlcExpr_t *parse_cmp(P *p)
{
    PlcExpr_t *e = parse_add(p);
    while (!p->fatal && e != NULL && (sym(p, "=") || sym(p, "<>")
           || sym(p, "<") || sym(p, "<=") || sym(p, ">") || sym(p, ">="))) {
        uint8_t op;
        if (sym(p, "="))       op = XO_EQ;
        else if (sym(p, "<>")) op = XO_NE;
        else if (sym(p, "<"))  op = XO_LT;
        else if (sym(p, "<=")) op = XO_LE;
        else if (sym(p, ">"))  op = XO_GT;
        else                   op = XO_GE;
        pl_next(p);
        e = mk_bin(p, op, e, parse_add(p));
    }
    return e;
}

static PlcExpr_t *parse_and(P *p)
{
    PlcExpr_t *e = parse_cmp(p);
    while (!p->fatal && e != NULL && kw(p, "AND")) {
        pl_next(p);
        e = mk_bin(p, XO_AND, e, parse_cmp(p));
    }
    return e;
}

static PlcExpr_t *parse_xor(P *p)
{
    PlcExpr_t *e = parse_and(p);
    while (!p->fatal && e != NULL && kw(p, "XOR")) {
        pl_next(p);
        e = mk_bin(p, XO_XOR, e, parse_and(p));
    }
    return e;
}

static PlcExpr_t *parse_expr(P *p)
{
    PlcExpr_t *e = parse_xor(p);
    while (!p->fatal && e != NULL && kw(p, "OR")) {
        pl_next(p);
        e = mk_bin(p, XO_OR, e, parse_xor(p));
    }
    return e;
}

/* =====================================================================
 *  语句
 * ===================================================================== */
static PlcStmt_t *parse_stmts(P *p, const char **stop_kws, int n_stop);

static PlcStmt_t *mk_stmt(P *p, uint8_t kind)
{
    PlcStmt_t *s = (PlcStmt_t *)arena_alloc(&p->arena, sizeof(PlcStmt_t));
    if (s == NULL) { p->fatal = 1; return NULL; }
    s->kind = kind;
    s->line = (uint16_t)(p->line & 0xFFFF);
    return s;
}

static int stop_hit(P *p, const char **stop_kws, int n_stop)
{
    if (p->tk != TK_IDENT) return 0;
    for (int i = 0; i < n_stop; i++)
        if (strcmp(p->up, stop_kws[i]) == 0) return 1;
    return 0;
}

/* 语句级错误恢复: 跳到 ';' 消费或下一语句起始 */
static void recover_stmt(P *p)
{
    while (p->tk != TK_EOF) {
        if (p->tk == TK_SYMB && strcmp(p->txt, ";") == 0) { pl_next(p); return; }
        if (p->tk == TK_IDENT
            && (strcmp(p->up, "IF") == 0 || strcmp(p->up, "WHILE") == 0
                || strcmp(p->up, "END_IF") == 0 || strcmp(p->up, "END_WHILE") == 0
                || strcmp(p->up, "ELSE") == 0 || strcmp(p->up, "ELSIF") == 0
                || strcmp(p->up, "VAR") == 0 || strcmp(p->up, "END_VAR") == 0))
            return;
        pl_next(p);
    }
}

/* 赋值 / FB 调用 (IDENT 已消费, up/line 传入) */
static PlcStmt_t *parse_assign_or_call(P *p, const char *up, int line)
{
    int vi = var_find(p, up);
    if (vi < 0) {
        pl_diag(p, line, "未声明变量 '%s'", up);
        recover_stmt(p);
        return NULL;
    }
    PlcVarSlot_t *vs = &p->prog->vars[vi];

    /* FB 调用 */
    if (sym(p, "(")) {
        if (vs->fb == PLC_FB_NONE) {
            pl_diag(p, line, "'%s' 不是 FB 实例, 不可调用", up);
            recover_stmt(p);
            return NULL;
        }
        pl_next(p);
        PlcStmt_t *s = mk_stmt(p, PS_FBCALL);
        if (s == NULL) return NULL;
        s->line = (uint16_t)(line & 0xFFFF);
        s->u.fb.var_idx = (int16_t)vi;

        if (!sym(p, ")")) {
            for (;;) {
                if (p->tk != TK_IDENT) {
                    pl_diag(p, p->line, "FB 参数名非法 (按名传参: IN := 表达式)");
                    recover_stmt(p);
                    return NULL;
                }
                /* 参数名必须先拷贝 — pl_next 会覆写 lexer 缓冲区 */
                char pname[PLC_NAME_MAX + 8];
                strncpy(pname, p->up, sizeof(pname) - 1);
                pname[sizeof(pname) - 1] = '\0';
                int pline = p->line;
                pl_next(p);
                if (!expect_sym(p, ":=")) { recover_stmt(p); return NULL; }

                uint8_t param = PLC_FP_NONE;
                if (strcmp(pname, "IN") == 0)  param = PLC_FP_IN;
                else if (strcmp(pname, "PT") == 0)  param = PLC_FP_PT;
                else if (strcmp(pname, "CU") == 0)  param = PLC_FP_CU;
                else if (strcmp(pname, "PV") == 0)  param = PLC_FP_PV;
                else if (strcmp(pname, "R") == 0)   param = PLC_FP_R;
                else if (strcmp(pname, "CLK") == 0) param = PLC_FP_CLK;

                int valid = 0;
                uint8_t need_bool = 1;
                switch (param) {
                case PLC_FP_IN:  valid = (vs->fb == PLC_FB_TON || vs->fb == PLC_FB_TOF); break;
                case PLC_FP_PT:  valid = (vs->fb == PLC_FB_TON || vs->fb == PLC_FB_TOF); need_bool = 0; break;
                case PLC_FP_CU:  valid = (vs->fb == PLC_FB_CTU); break;
                case PLC_FP_PV:  valid = (vs->fb == PLC_FB_CTU); need_bool = 0; break;
                case PLC_FP_R:   valid = (vs->fb == PLC_FB_CTU); break;
                case PLC_FP_CLK: valid = (vs->fb == PLC_FB_R_TRIG || vs->fb == PLC_FB_F_TRIG); break;
                default: valid = 0; break;
                }
                if (!valid) {
                    pl_diag(p, pline, "FB '%s' 无参数 %s", up, pname);
                    recover_stmt(p);
                    return NULL;
                }
                for (int a = 0; a < s->u.fb.n_args; a++)
                    if (s->u.fb.args[a].param == param) {
                        pl_diag(p, pline, "FB 参数 %s 重复", pname);
                        recover_stmt(p);
                        return NULL;
                    }

                PlcExpr_t *x = parse_expr(p);
                if (x == NULL) { recover_stmt(p); return NULL; }
                if (need_bool && x->type != PLC_T_BOOL) {
                    pl_diag(p, pline, "FB 参数 %s 须为 BOOL", pname);
                    recover_stmt(p);
                    return NULL;
                }
                if (!need_bool && !t_numeric(x->type)) {
                    pl_diag(p, pline, "FB 参数 %s 须为数值", pname);
                    recover_stmt(p);
                    return NULL;
                }
                s->u.fb.args[s->u.fb.n_args].param = param;
                s->u.fb.args[s->u.fb.n_args].val = x;
                s->u.fb.n_args++;

                if (accept_sym(p, ",")) continue;
                break;
            }
        }
        if (!expect_sym(p, ")")) { recover_stmt(p); return NULL; }
        if (!expect_sym(p, ";")) { /* ';' 缺失容忍 (recover 兜底) */ }
        return s;
    }

    /* 赋值: 目标 [ .bit ] := expr ; */
    int8_t bit = -1;
    if (accept_sym(p, ".")) {
        if (vs->fb != PLC_FB_NONE) {
            pl_diag(p, line, "FB 输出只读, 不可赋值 ('%s')", up);
            recover_stmt(p);
            return NULL;
        }
        if (p->tk != TK_INT || p->val.v.i < 0 || p->val.v.i > 15) {
            pl_diag(p, p->line, "位选目标: 变量.0~15 (字面量)");
            recover_stmt(p);
            return NULL;
        }
        bit = (int8_t)p->val.v.i;
        pl_next(p);
    }
    if (vs->cls == PLC_CLS_SYS_G) {
        pl_diag(p, line, "G_* 只读变量不可赋值 ('%s')", up);
        recover_stmt(p);
        return NULL;
    }
    if (vs->fb != PLC_FB_NONE) {
        pl_diag(p, line, "FB 实例 '%s' 不可整体赋值", up);
        recover_stmt(p);
        return NULL;
    }
    if (bit >= 0 && vs->type != PLC_T_INT && vs->type != PLC_T_DINT) {
        pl_diag(p, line, "'%s' 非 INT/DINT, 不支持位选赋值", up);
        recover_stmt(p);
        return NULL;
    }

    if (!expect_sym(p, ":=")) { recover_stmt(p); return NULL; }
    PlcExpr_t *rhs = parse_expr(p);
    if (rhs == NULL) { recover_stmt(p); return NULL; }

    if (bit >= 0) {
        if (rhs->type != PLC_T_BOOL) {
            pl_diag(p, line, "位选赋值右侧须为 BOOL");
            recover_stmt(p);
            return NULL;
        }
    } else if (vs->type == PLC_T_BOOL) {
        if (rhs->type != PLC_T_BOOL) {
            pl_diag(p, line, "BOOL 变量 '%s' 赋值右侧须为 BOOL", up);
            recover_stmt(p);
            return NULL;
        }
    } else if (!t_numeric(rhs->type)) {
        pl_diag(p, line, "数值变量 '%s' 赋值右侧须为数值", up);
        recover_stmt(p);
        return NULL;
    }

    PlcStmt_t *s = mk_stmt(p, PS_ASSIGN);
    if (s == NULL) return NULL;
    s->line = (uint16_t)(line & 0xFFFF);
    s->u.as.var_idx = (int16_t)vi;
    s->u.as.bit = bit;
    s->u.as.rhs = rhs;
    if (!expect_sym(p, ";")) { /* 容忍: 行尾缺失提示已给 */ }
    return s;
}

static PlcStmt_t *parse_statement(P *p)
{
    if (p->fatal) return NULL;

    if (kw(p, "IF")) {
        int line = p->line;
        pl_next(p);
        PlcExpr_t *cond = parse_expr(p);
        if (cond == NULL) { recover_stmt(p); return NULL; }
        if (cond->type != PLC_T_BOOL) {
            pl_diag(p, line, "IF 条件须为 BOOL");
            recover_stmt(p);
            return NULL;
        }
        if (!kw(p, "THEN")) {
            pl_diag(p, p->line, "期望 THEN");
            recover_stmt(p);
            return NULL;
        }
        pl_next(p);
        static const char *stop_then[] = { "ELSIF", "ELSE", "END_IF" };
        PlcStmt_t *s = mk_stmt(p, PS_IF);
        if (s == NULL) return NULL;
        s->line = (uint16_t)(line & 0xFFFF);
        s->u.ifs.b_then.cond = cond;
        s->u.ifs.b_then.body = parse_stmts(p, stop_then, 3);
        s->u.ifs.elsif = NULL;
        s->u.ifs.n_elsif = 0;
        s->u.ifs.body_else = NULL;

        while (kw(p, "ELSIF")) {
            int el = p->line;
            pl_next(p);
            PlcExpr_t *ec = parse_expr(p);
            if (ec == NULL) { recover_stmt(p); return NULL; }
            if (ec->type != PLC_T_BOOL) {
                pl_diag(p, el, "ELSIF 条件须为 BOOL");
                recover_stmt(p);
                return NULL;
            }
            if (!kw(p, "THEN")) {
                pl_diag(p, p->line, "期望 THEN");
                recover_stmt(p);
                return NULL;
            }
            pl_next(p);
            /* ELSIF 分支数组按需增长 (arena 登记的临时数组) */
            PlcIfBranch_t *nb = (PlcIfBranch_t *)arena_alloc(
                &p->arena, (size_t)(s->u.ifs.n_elsif + 1) * sizeof(PlcIfBranch_t));
            if (nb == NULL) { p->fatal = 1; return NULL; }
            if (s->u.ifs.n_elsif > 0)
                memcpy(nb, s->u.ifs.elsif,
                       (size_t)s->u.ifs.n_elsif * sizeof(PlcIfBranch_t));
            s->u.ifs.elsif = nb;
            s->u.ifs.elsif[s->u.ifs.n_elsif].cond = ec;
            s->u.ifs.elsif[s->u.ifs.n_elsif].body = parse_stmts(p, stop_then, 3);
            s->u.ifs.n_elsif++;
        }
        if (kw(p, "ELSE")) {
            pl_next(p);
            static const char *stop_else[] = { "END_IF" };
            s->u.ifs.body_else = parse_stmts(p, stop_else, 1);
        }
        if (!kw(p, "END_IF")) {
            pl_diag(p, p->line, "期望 END_IF");
            recover_stmt(p);
            return NULL;
        }
        pl_next(p);
        if (!expect_sym(p, ";")) { /* 容忍 */ }
        return s;
    }

    if (kw(p, "WHILE")) {
        int line = p->line;
        pl_next(p);
        PlcExpr_t *cond = parse_expr(p);
        if (cond == NULL) { recover_stmt(p); return NULL; }
        if (cond->type != PLC_T_BOOL) {
            pl_diag(p, line, "WHILE 条件须为 BOOL");
            recover_stmt(p);
            return NULL;
        }
        if (!kw(p, "DO")) {
            pl_diag(p, p->line, "期望 DO");
            recover_stmt(p);
            return NULL;
        }
        pl_next(p);
        static const char *stop_wh[] = { "END_WHILE" };
        PlcStmt_t *s = mk_stmt(p, PS_WHILE);
        if (s == NULL) return NULL;
        s->line = (uint16_t)(line & 0xFFFF);
        s->u.wh.cond = cond;
        s->u.wh.body = parse_stmts(p, stop_wh, 1);
        if (!kw(p, "END_WHILE")) {
            pl_diag(p, p->line, "期望 END_WHILE");
            recover_stmt(p);
            return NULL;
        }
        pl_next(p);
        if (!expect_sym(p, ";")) { /* 容忍 */ }
        return s;
    }

    if (p->tk == TK_IDENT) {
        char up[PLC_NAME_MAX + 8];
        strncpy(up, p->up, sizeof(up) - 1);
        up[sizeof(up) - 1] = '\0';
        int line = p->line;
        pl_next(p);
        return parse_assign_or_call(p, up, line);
    }

    pl_diag(p, p->line, "语句语法错误 (期望 IF/WHILE/赋值/FB调用)");
    recover_stmt(p);
    return NULL;
}

/* 语句序列直到 stop 关键字 (停在关键字上, 不消费) */
static PlcStmt_t *parse_stmts(P *p, const char **stop_kws, int n_stop)
{
    PlcStmt_t *head = NULL, **tail = &head;
    while (!p->fatal && p->tk != TK_EOF && !stop_hit(p, stop_kws, n_stop)) {
        PlcStmt_t *s = parse_statement(p);
        if (s != NULL) {
            *tail = s;
            tail = &s->next;
        }
    }
    return head;
}

/* =====================================================================
 *  VAR 块
 * ===================================================================== */
static void parse_var_block(P *p)
{
    pl_next(p);   /* 消费 VAR */
    while (!p->fatal && p->tk != TK_EOF && !kw(p, "END_VAR")) {
        if (p->tk != TK_IDENT) {
            pl_diag(p, p->line, "VAR 块内期望变量名");
            p->fatal = 1;
            return;
        }
        char up[PLC_NAME_MAX + 8];
        strncpy(up, p->up, sizeof(up) - 1);
        up[sizeof(up) - 1] = '\0';
        int line = p->line;
        pl_next(p);

        for (int i = 0; g_reserved[i]; i++) {
            if (strcmp(up, g_reserved[i]) == 0) {
                pl_diag(p, line, "'%s' 是保留字, 不可作变量名", up);
                p->fatal = 1;
                return;
            }
        }
        if (var_find(p, up) >= 0) {
            pl_diag(p, line, "变量 '%s' 重复声明", up);
            p->fatal = 1;
            return;
        }
        if (p->prog->n_user_vars >= PLC_USER_VAR_MAX) {
            pl_diag(p, line, "用户变量超过上限 %d", PLC_USER_VAR_MAX);
            p->fatal = 1;
            return;
        }
        if (!expect_sym(p, ":")) { p->fatal = 1; return; }

        PlcVarSlot_t *vs = &p->prog->vars[p->prog->n_vars];
        memset(vs, 0, sizeof(*vs));
        strncpy(vs->name, up, PLC_NAME_MAX - 1);

        if (p->tk != TK_IDENT) {
            pl_diag(p, line, "期望类型名 (BOOL/INT/DINT/REAL/FB)");
            p->fatal = 1;
            return;
        }
        if (kw(p, "BOOL"))      vs->type = PLC_T_BOOL;
        else if (kw(p, "INT"))  vs->type = PLC_T_INT;
        else if (kw(p, "DINT")) vs->type = PLC_T_DINT;
        else if (kw(p, "REAL")) vs->type = PLC_T_REAL;
        else if (kw(p, "TON"))     vs->fb = PLC_FB_TON;
        else if (kw(p, "TOF"))     vs->fb = PLC_FB_TOF;
        else if (kw(p, "CTU"))     vs->fb = PLC_FB_CTU;
        else if (kw(p, "R_TRIG"))  vs->fb = PLC_FB_R_TRIG;
        else if (kw(p, "F_TRIG"))  vs->fb = PLC_FB_F_TRIG;
        else {
            pl_diag(p, line, "未知类型 '%s'", p->txt);
            p->fatal = 1;
            return;
        }
        pl_next(p);

        /* 可选初值 (仅标量) */
        if (accept_sym(p, ":=")) {
            if (vs->fb != PLC_FB_NONE) {
                pl_diag(p, line, "FB 实例不支持初值");
                p->fatal = 1;
                return;
            }
            if (p->tk != TK_INT && p->tk != TK_REAL
                && !kw(p, "TRUE") && !kw(p, "FALSE")) {
                pl_diag(p, p->line, "初值须为字面量");
                p->fatal = 1;
                return;
            }
            PlcVal v = { PLC_T_BOOL, {0} };
            if (kw(p, "TRUE") || kw(p, "FALSE")) {
                v.type = PLC_T_BOOL;
                v.v.i = kw(p, "TRUE") ? 1 : 0;
            } else {
                v = p->val;
            }
            pl_next(p);
            if (vs->type == PLC_T_BOOL) {
                if (v.type != PLC_T_BOOL) {
                    pl_diag(p, line, "BOOL 变量初值须为 TRUE/FALSE");
                    p->fatal = 1;
                    return;
                }
            } else if (v.type == PLC_T_BOOL || !t_numeric(v.type)) {
                pl_diag(p, line, "数值变量初值须为数值");
                p->fatal = 1;
                return;
            }
            vs->init = v;
        }
        if (!expect_sym(p, ";")) { p->fatal = 1; return; }

        p->prog->n_vars++;
        p->prog->n_user_vars++;
    }
    if (!kw(p, "END_VAR")) {
        pl_diag(p, p->line, "VAR 块未以 END_VAR 结束");
        p->fatal = 1;
        return;
    }
    pl_next(p);
}

/* =====================================================================
 *  编译入口
 * ===================================================================== */
int PlcCompile(const char *src, int src_len,
               PlcProgram_t **out_prog,
               PlcDiag_t *diags, int diag_max, int *n_diag)
{
    if (out_prog == NULL || src == NULL || src_len <= 0) {
        if (n_diag) *n_diag = 0;
        return -1;
    }
    *out_prog = NULL;

    PlcDiag_t internal[PLC_DIAG_MAX];
    P p;
    memset(&p, 0, sizeof(p));
    p.src = src;
    p.len = src_len;
    p.pos = 0;
    p.line = 1;
    p.diags = internal;

    PlcProgram_t *prog = (PlcProgram_t *)calloc(1, sizeof(PlcProgram_t));
    if (prog == NULL) {
        if (n_diag) *n_diag = 0;
        return -1;
    }
    p.prog = prog;

    /* 系统变量预声明 (固定前缀) */
    for (int i = 0; i < PlcSysVarCount(); i++) {
        PlcVarSlot_t *vs = &prog->vars[prog->n_vars++];
        strncpy(vs->name, g_sys_vars[i].name, PLC_NAME_MAX - 1);
        vs->type = g_sys_vars[i].type;
        vs->cls  = g_sys_vars[i].cls;
        vs->fb   = PLC_FB_NONE;
    }

    pl_next(&p);    /* 预读首 token */

    /* VAR 块 (仅允许出现在语句前) */
    int seen_stmt = 0;
    while (!p.fatal && kw(&p, "VAR")) {
        if (seen_stmt) {
            pl_diag(&p, p.line, "VAR 块必须位于语句之前");
            p.fatal = 1;
            break;
        }
        parse_var_block(&p);
    }

    /* 顶层语句 */
    prog->stmts = parse_stmts(&p, NULL, 0);

    if (p.tk != TK_EOF && !p.fatal) {
        pl_diag(&p, p.line, "程序末尾存在多余 token (VAR 块须在前)");
    }

    /* 统计语句数 */
    for (PlcStmt_t *s = prog->stmts; s; s = s->next) prog->n_stmts++;

    int ok = (!p.fatal && p.n_diag == 0 && p.tk == TK_EOF);

    if (n_diag) *n_diag = p.n_diag;
    if (diags != NULL && diag_max > 0) {
        int ncopy = (p.n_diag < diag_max) ? p.n_diag : diag_max;
        memcpy(diags, internal, (size_t)ncopy * sizeof(PlcDiag_t));
    }

    if (!ok) {
        /* 失败: arena 全量回收 + 产物回收 */
        for (int i = 0; i < p.arena.n; i++) free(p.arena.ptrs[i]);
        free(p.arena.ptrs);
        free(prog);
        return -1;
    }

    prog->allocs = p.arena.ptrs;
    prog->n_allocs = p.arena.n;
    *out_prog = prog;
    return 0;
}

void PlcProgramFree(PlcProgram_t *prog)
{
    if (prog == NULL) return;
    if (prog->allocs != NULL) {
        for (int i = 0; i < prog->n_allocs; i++) free(prog->allocs[i]);
        free(prog->allocs);
    }
    free(prog);
}
