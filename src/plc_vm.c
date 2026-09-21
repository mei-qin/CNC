/* =====================================================================
 *  plc_vm.c —— P1 软 PLC 树遍历解释器实现
 *
 *  设计见 inc/plc_vm.h 头注释。
 *
 *  数值语义:
 *    - INT/DINT 运算在 int64 中间量完成, 结果按目标类型钳制
 *      (INT ±32767, DINT ±2147483647)
 *    - 任一 REAL 操作数 → double 运算
 *    - 整数除法截断 (IEC); MOD 符号随被除数 (同 C %)
 *    - 隐式收窄钳制: REAL→INT/DINT 越界饱和
 * ===================================================================== */

#include "plc_vm.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

/* ---- G 区系统变量序号 (与 plc_lang.c g_sys_vars 一致, 静态断言位) ---- */
enum {
    SV_G_RUNNING = 0, SV_G_HOLD, SV_G_ALARM, SV_G_PAUSED, SV_G_WAIT_MCODE,
    SV_G_LASER_KILL, SV_G_MCODE, SV_G_MCODE_STROBE, SV_G_SPINDLE_MODE,
    SV_G_COOLANT_STATE, SV_G_LASER_GATE, SV_G_LASER_INTERLOCK, SV_G_UPTIME_MS,
    SV_G_AI0, SV_G_AI1, SV_G_AI2, SV_G_AI3,
    SV_F_PERIPHERAL_OK, SV_F_MCODE_FIN, SV_F_STATUS,
    SV_COUNT
};

/* =====================================================================
 *  辅助: 值转换
 * ===================================================================== */
static int32_t val_i32(const PlcVal *v)
{
    return (v->type == PLC_T_REAL) ? (int32_t)v->v.d : v->v.i;
}

static double val_d(const PlcVal *v)
{
    return (v->type == PLC_T_REAL) ? v->v.d : (double)v->v.i;
}

static int32_t clamp_i32(int64_t x)
{
    if (x > 2147483647LL)  return 2147483647;
    if (x < -2147483648LL) return -2147483647 - 1;
    return (int32_t)x;
}

/* 按 target 类型转换 + 钳制 */
static PlcVal conv_to(const PlcVal *v, uint8_t target)
{
    PlcVal r;
    r.type = target;
    if (target == PLC_T_REAL) {
        r.v.d = val_d(v);
    } else if (target == PLC_T_BOOL) {
        r.v.i = (v->type == PLC_T_REAL) ? (v->v.d != 0.0) : (v->v.i != 0);
    } else if (target == PLC_T_INT) {
        int64_t x = (v->type == PLC_T_REAL) ? (int64_t)v->v.d : v->v.i;
        if (x > 32767)  x = 32767;
        if (x < -32768) x = -32768;
        r.v.i = (int32_t)x;
    } else {    /* DINT */
        r.v.i = clamp_i32((v->type == PLC_T_REAL) ? (int64_t)v->v.d : v->v.i);
    }
    return r;
}

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* =====================================================================
 *  VM 状态
 * ===================================================================== */
typedef struct {
    PlcVm_t   *vm;
    int64_t    dt_ms;
    uint64_t   deadline_ns;
    int        err;
} VmRun;

static void vm_err(VmRun *r, int line, const char *msg)
{
    if (r->err) return;
    r->err = 1;
    r->vm->runtime_err = 1;
    r->vm->err_line = line;
    strncpy(r->vm->err_msg, msg, PLC_VM_ERR_MSG_LEN - 1);
    r->vm->err_msg[PLC_VM_ERR_MSG_LEN - 1] = '\0';
}

/* =====================================================================
 *  Init
 * ===================================================================== */
int PlcVm_Init(PlcVm_t *vm, PlcProgram_t *prog)
{
    if (vm == NULL || prog == NULL) return -1;
    memset(vm, 0, sizeof(*vm));
    vm->prog = prog;
    int n = prog->n_vars;
    if (n > PLC_SYS_VAR_MAX + PLC_USER_VAR_MAX) n = PLC_SYS_VAR_MAX + PLC_USER_VAR_MAX;

    for (int i = 0; i < n; i++) {
        vm->fb_of[i] = -1;
        if (i < PlcSysVarCount()) {
            /* 系统变量: 初值 0 (G 每周期刷新 / F 由程序写) */
            vm->cells[i].type = prog->vars[i].type;
            vm->cells[i].v.i = 0;
            vm->cells[i].v.d = 0.0;
        } else {
            /* 用户变量: 声明初值 (按声明类型转换) */
            vm->cells[i] = conv_to(&prog->vars[i].init, prog->vars[i].type);
            if (prog->vars[i].fb != PLC_FB_NONE) {
                vm->fb_of[i] = (int16_t)vm->n_fb;
                vm->fb[vm->n_fb].kind = prog->vars[i].fb;
                vm->n_fb++;
            }
        }
    }
    /* P1.5-2: 强制表清零 (memset 后 atomic 显式初始化, 强制不跨重启) */
    for (int i = 0; i < PLC_USER_VAR_MAX; i++) {
        atomic_store_explicit(&vm->force_on[i], 0, memory_order_relaxed);
        atomic_store_explicit(&vm->force_val[i], 0.0, memory_order_relaxed);
    }
    return 0;
}

/* =====================================================================
 *  表达式求值
 * ===================================================================== */
static PlcVal eval(VmRun *r, PlcExpr_t *e);

static PlcVal make_b(int b)
{
    PlcVal v; v.type = PLC_T_BOOL; v.v.i = b ? 1 : 0; return v;
}

static PlcVal eval_bin(VmRun *r, PlcExpr_t *e)
{
    PlcVal l = eval(r, e->u.bin.l);
    if (r->err) return l;
    PlcVal rv = eval(r, e->u.bin.r);
    if (r->err) return rv;

    switch (e->u.bin.op) {
    case XO_AND: return make_b(l.v.i & rv.v.i);
    case XO_OR:  return make_b(l.v.i | rv.v.i);
    case XO_XOR: return make_b(l.v.i ^ rv.v.i);
    case XO_EQ: case XO_NE: {
        int eq;
        if (l.type == PLC_T_REAL || rv.type == PLC_T_REAL)
            eq = (val_d(&l) == val_d(&rv));
        else
            eq = (val_i32(&l) == val_i32(&rv));
        return make_b((e->u.bin.op == XO_EQ) ? eq : !eq);
    }
    case XO_LT: case XO_LE: case XO_GT: case XO_GE: {
        double a = val_d(&l), b = val_d(&rv);
        int res;
        if      (e->u.bin.op == XO_LT) res = (a <  b);
        else if (e->u.bin.op == XO_LE) res = (a <= b);
        else if (e->u.bin.op == XO_GT) res = (a >  b);
        else                           res = (a >= b);
        return make_b(res);
    }
    default: break;    /* 算术 */
    }

    /* 算术: REAL 提升双精度; 整数在 int64 */
    int is_real = (l.type == PLC_T_REAL || rv.type == PLC_T_REAL);
    uint8_t rt = e->type;

    if (e->u.bin.op == XO_DIV || e->u.bin.op == XO_MOD) {
        int div0 = (rv.type == PLC_T_REAL) ? (rv.v.d == 0.0) : (rv.v.i == 0);
        if (div0) {
            vm_err(r, e->line, "运行期除数为零");
            return l;
        }
    }

    PlcVal out;
    out.type = rt;
    if (is_real) {
        double a = val_d(&l), b = val_d(&rv);
        double d = 0.0;
        switch (e->u.bin.op) {
        case XO_ADD: d = a + b; break;
        case XO_SUB: d = a - b; break;
        case XO_MUL: d = a * b; break;
        case XO_DIV: d = a / b; break;
        case XO_MOD: d = (double)((int64_t)a % (int64_t)b); break;
        default: break;
        }
        out.v.d = d;
        return out;
    }
    int64_t a = val_i32(&l), b = val_i32(&rv);
    int64_t x = 0;
    switch (e->u.bin.op) {
    case XO_ADD: x = a + b; break;
    case XO_SUB: x = a - b; break;
    case XO_MUL: x = a * b; break;
    case XO_DIV: x = (b != 0) ? (a / b) : 0; break;
    case XO_MOD: x = (b != 0) ? (a % b) : 0; break;
    default: break;
    }
    if (rt == PLC_T_INT) {
        if (x > 32767)  x = 32767;
        if (x < -32768) x = -32768;
        out.v.i = (int32_t)x;
    } else {
        out.v.i = clamp_i32(x);
    }
    return out;
}

static PlcVal eval(VmRun *r, PlcExpr_t *e)
{
    PlcVal zero;
    zero.type = PLC_T_BOOL; zero.v.i = 0; zero.v.d = 0.0;
    if (r->err || e == NULL) return zero;

    switch (e->kind) {
    case XE_LIT:
        return e->u.lit;

    case XE_VAR:
        return r->vm->cells[e->u.vr.var_idx];

    case XE_BIT: {
        int32_t v = r->vm->cells[e->u.bit.var_idx].v.i;
        return make_b((v >> e->u.bit.bit) & 1);
    }

    case XE_FIELD: {
        int s = r->vm->fb_of[e->u.fld.var_idx];
        PlcFbState_t *f = &r->vm->fb[s];
        switch (e->u.fld.field) {
        case PLC_FF_Q:  return make_b(f->q);
        case PLC_FF_CV: { PlcVal v; v.type = PLC_T_DINT; v.v.i = f->cv; return v; }
        case PLC_FF_ET: { PlcVal v; v.type = PLC_T_DINT;
                          v.v.i = clamp_i32(f->elapsed); return v; }
        default: return zero;
        }
    }

    case XE_BIN:
        return eval_bin(r, e);

    case XE_UN: {
        PlcVal x = eval(r, e->u.un.e);
        if (r->err) return x;
        if (e->u.un.op == XU_NOT) {
            x.v.i = x.v.i ? 0 : 1;
            return x;
        }
        if (x.type == PLC_T_REAL) x.v.d = -x.v.d;
        else                      x.v.i = -x.v.i;
        return x;
    }

    case XE_FUNC: {
        PlcVal a0 = eval(r, e->u.func.a[0]);
        if (r->err) return a0;
        PlcVal a1, a2;
        if (e->u.func.n >= 2) { a1 = eval(r, e->u.func.a[1]); if (r->err) return a1; }
        if (e->u.func.n >= 3) { a2 = eval(r, e->u.func.a[2]); if (r->err) return a2; }

        switch (e->u.func.fn) {
        case XF_MOVE:
            return conv_to(&a0, e->type);
        case XF_ABS:
            if (a0.type == PLC_T_REAL) { a0.v.d = (a0.v.d < 0) ? -a0.v.d : a0.v.d; return a0; }
            a0.v.i = (a0.v.i < 0) ? -a0.v.i : a0.v.i;
            return a0;
        case XF_MIN: {
            PlcVal out; out.type = e->type;
            out.v.d = (val_d(&a0) < val_d(&a1)) ? val_d(&a0) : val_d(&a1);
            return conv_to(&out, e->type);
        }
        case XF_MAX: {
            PlcVal out; out.type = e->type;
            out.v.d = (val_d(&a0) > val_d(&a1)) ? val_d(&a0) : val_d(&a1);
            return conv_to(&out, e->type);
        }
        case XF_LIMIT: {
            double lo = val_d(&a0), v = val_d(&a1), hi = val_d(&a2);
            PlcVal out; out.type = e->type;
            out.v.d = (v < lo) ? lo : ((v > hi) ? hi : v);
            return conv_to(&out, e->type);
        }
        default: return zero;
        }
    }

    default:
        return zero;
    }
}

/* =====================================================================
 *  FB 转移
 * ===================================================================== */
static void fb_call(VmRun *r, PlcStmt_t *s)
{
    PlcFbState_t *f = &r->vm->fb[r->vm->fb_of[s->u.fb.var_idx]];

    /* 求值所有已赋值参数 (保持 IEC "未赋值沿用上次") */
    int has[8] = {0};
    PlcVal pv[8];
    for (int i = 0; i < s->u.fb.n_args; i++) {
        pv[s->u.fb.args[i].param] = eval(r, s->u.fb.args[i].val);
        has[s->u.fb.args[i].param] = 1;
        if (r->err) return;
    }
    int in  = has[PLC_FP_IN]  ? (pv[PLC_FP_IN].v.i  != 0)  : f->prev_in;
    int clk = has[PLC_FP_CLK] ? (pv[PLC_FP_CLK].v.i != 0)  : f->prev_in;
    int cu  = has[PLC_FP_CU]  ? (pv[PLC_FP_CU].v.i  != 0)  : f->prev_in;
    int rst = has[PLC_FP_R]   ? (pv[PLC_FP_R].v.i   != 0)  : 0;
    if (has[PLC_FP_PT]) f->pt = val_i32(&pv[PLC_FP_PT]);
    if (has[PLC_FP_PV]) f->pv = val_i32(&pv[PLC_FP_PV]);

    switch (f->kind) {
    case PLC_FB_TON:
        if (!in) { f->elapsed = 0; f->q = 0; }
        else {
            f->elapsed += r->dt_ms;
            if (f->elapsed >= f->pt) f->q = 1;
        }
        f->prev_in = (uint8_t)in;
        break;
    case PLC_FB_TOF:
        if (in) { f->q = 1; f->elapsed = 0; }
        else {
            f->elapsed += r->dt_ms;
            if (f->q && f->elapsed >= f->pt) f->q = 0;
        }
        f->prev_in = (uint8_t)in;
        break;
    case PLC_FB_CTU:
        if (rst) f->cv = 0;
        else if (cu && !f->prev_in) {
            if (f->cv < 2147483647) f->cv++;
        }
        f->q = (f->cv >= f->pv) ? 1 : 0;
        f->prev_in = (uint8_t)cu;
        break;
    case PLC_FB_R_TRIG:
        f->q = (clk && !f->prev_in) ? 1 : 0;
        f->prev_in = (uint8_t)clk;
        break;
    case PLC_FB_F_TRIG:
        f->q = (!clk && f->prev_in) ? 1 : 0;
        f->prev_in = (uint8_t)clk;
        break;
    default:
        break;
    }
}

/* =====================================================================
 *  语句执行
 * ===================================================================== */
static void exec_stmts(VmRun *r, PlcStmt_t *s);

static void exec_stmt(VmRun *r, PlcStmt_t *s)
{
    if (r->err) return;

    switch (s->kind) {
    case PS_ASSIGN: {
        PlcVal v = eval(r, s->u.as.rhs);
        if (r->err) return;
        PlcVal *cell = &r->vm->cells[s->u.as.var_idx];
        uint8_t vt = r->vm->prog->vars[s->u.as.var_idx].type;
        if (s->u.as.bit >= 0) {
            int32_t x = cell->v.i;
            int32_t m = (int32_t)1 << s->u.as.bit;
            cell->v.i = v.v.i ? (x | m) : (x & ~m);
        } else {
            *cell = conv_to(&v, vt);
        }
        break;
    }
    case PS_IF: {
        PlcVal c = eval(r, s->u.ifs.b_then.cond);
        if (r->err) return;
        if (c.v.i) { exec_stmts(r, s->u.ifs.b_then.body); return; }
        for (int i = 0; i < s->u.ifs.n_elsif; i++) {
            PlcVal ec = eval(r, s->u.ifs.elsif[i].cond);
            if (r->err) return;
            if (ec.v.i) { exec_stmts(r, s->u.ifs.elsif[i].body); return; }
        }
        if (s->u.ifs.body_else != NULL) exec_stmts(r, s->u.ifs.body_else);
        break;
    }
    case PS_WHILE: {
        for (;;) {
            PlcVal c = eval(r, s->u.wh.cond);
            if (r->err) return;
            if (!c.v.i) break;
            exec_stmts(r, s->u.wh.body);
            if (r->err) return;
            if (now_ns() > r->deadline_ns) {
                vm_err(r, s->line, "WHILE 执行超时 (2× 扫描周期)");
                return;
            }
        }
        break;
    }
    case PS_FBCALL:
        fb_call(r, s);
        break;
    default:
        break;
    }
}

static void exec_stmts(VmRun *r, PlcStmt_t *s)
{
    for (; s != NULL && !r->err; s = s->next)
        exec_stmt(r, s);
}

/* =====================================================================
 *  周期入口
 * ===================================================================== */
int PlcVm_Cycle(PlcVm_t *vm, const PeriphG_t *g, int64_t dt_ms,
                uint64_t deadline_ns)
{
    if (vm == NULL || vm->prog == NULL || g == NULL) return -1;

    /* G 区刷新 (序号 = SV_*) */
    PlcVal *c = vm->cells;
    c[SV_G_RUNNING].v.i      = (g->flags & PERIPH_G_FLAG_RUNNING)    ? 1 : 0;
    c[SV_G_HOLD].v.i         = (g->flags & PERIPH_G_FLAG_HOLD)       ? 1 : 0;
    c[SV_G_ALARM].v.i        = (g->flags & PERIPH_G_FLAG_ALARM)      ? 1 : 0;
    c[SV_G_PAUSED].v.i       = (g->flags & PERIPH_G_FLAG_PAUSED)     ? 1 : 0;
    c[SV_G_WAIT_MCODE].v.i   = (g->flags & PERIPH_G_FLAG_WAIT_MCODE) ? 1 : 0;
    c[SV_G_LASER_KILL].v.i   = (g->flags & PERIPH_G_FLAG_LASER_KILL) ? 1 : 0;
    c[SV_G_MCODE].v.i        = g->mcode_value;
    c[SV_G_MCODE_STROBE].v.i = g->mcode_strobe;
    c[SV_G_SPINDLE_MODE].v.i = g->spindle_mode;
    c[SV_G_COOLANT_STATE].v.i = g->coolant_state;
    c[SV_G_LASER_GATE].v.i   = g->laser_gate;
    c[SV_G_LASER_INTERLOCK].v.i = g->laser_interlock;
    {
        double u = g->uptime_ms;
        if (u > 2147483647.0) u = 2147483647.0;
        if (u < 0.0) u = 0.0;
        c[SV_G_UPTIME_MS].v.i = (int32_t)u;
    }
    /* P1.5-1: AI 工程量 (REAL cell) */
    c[SV_G_AI0].v.d = g->ai[0];
    c[SV_G_AI1].v.d = g->ai[1];
    c[SV_G_AI2].v.d = g->ai[2];
    c[SV_G_AI3].v.d = g->ai[3];

    /* P1-3: M 码握手电平语义 — 无选通期间持续清零 FIN 寄存器。
     * 时序: RT 置 strobe=1 (M80 段入等待) → PLC 程序完成动作后写
     * F_MCODE_FIN := 码值 → RT 见 FIN==码 完成等待 → strobe 回 0 →
     * VM 下一周期清 FIN。电平清零保证上一次握手的 FIN 不会在空闲期
     * 残留到下一次 (stale-FIN 提前完成)。
     * 限制: 两个相同 M8x 间隔 < 1 扫描周期 (10ms) 属病态程序, 不保证。 */
    if (!c[SV_G_MCODE_STROBE].v.i) {
        c[SV_F_MCODE_FIN].v.i = 0;
    }

    VmRun r;
    r.vm = vm;
    r.dt_ms = dt_ms;
    r.deadline_ns = deadline_ns;
    r.err = 0;
    exec_stmts(&r, vm->prog->stmts);
    return r.err ? -1 : 0;
}

/* P1.5-2: 周期末应用强制表 (见 plc_vm.h 头注释) */
void PlcVm_ApplyForce(PlcVm_t *vm)
{
    if (vm == NULL || vm->prog == NULL) return;
    int base = PlcSysVarCount();
    int nuser = vm->prog->n_vars - base;
    if (nuser > PLC_USER_VAR_MAX) nuser = PLC_USER_VAR_MAX;
    for (int u = 0; u < nuser; u++) {
        if (!atomic_load_explicit(&vm->force_on[u], memory_order_acquire))
            continue;
        PlcVal v;
        v.type = PLC_T_REAL;
        v.v.d = atomic_load_explicit(&vm->force_val[u], memory_order_acquire);
        vm->cells[base + u] = conv_to(&v, vm->prog->vars[base + u].type);
    }
}
