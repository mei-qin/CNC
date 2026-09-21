/* =====================================================================
 *  plc_core.c —— P1 软 PLC 核心实现 (程序槽 + 10ms 扫描线程)
 *
 *  设计见 inc/plc_core.h 头注释 + docs/P1_SOFTPLC_DESIGN_SPEC.md §1/§6。
 *
 *  线程模型 (P1-2):
 *    RPC handler 线程: download/ctrl/status/snapshot — g_plc_mu 保护
 *      程序槽与状态字; 快照读取另用 g_snap_mu (扫描线程每周期覆盖)。
 *    扫描线程 (SCHED_OTHER, 10ms 绝对定时):
 *      ReadG → PlcVm_Cycle → F 区发布 (atomic) → 快照更新 → sleep
 *    运行期错误: PlcVm_Cycle 返回 -1 → 状态 ERROR, 线程自行退出。
 *
 *  boot autoload (D5): plc_core_init 时 ./plc_prog.st 存在则
 *    下载编译并自动 start (实机断电自恢复语义)。
 * ===================================================================== */

#include "plc_core.h"
#include "plc_vm.h"
#include "plc_trace.h"      /* P1.5-3: plc_trace_tick/arm */
#include "periph_bus.h"
#include "event_logger.h"   /* P1-4 接入; 现仅预留 include 位 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

/* ---- 程序槽 ---- */
static pthread_mutex_t g_plc_mu = PTHREAD_MUTEX_INITIALIZER;
static PlcProgram_t   *g_plc_prog = NULL;
static char            g_plc_text[PLC_PROG_TEXT_MAX + 1];
static int             g_plc_text_len = 0;
static int             g_plc_state = PLC_STATE_EMPTY;

/* ---- 扫描线程 + VM ---- */
static pthread_t  g_scan_thread;
static volatile int g_scan_run = 0;
static int         g_thread_alive = 0;   /* 创建后置 1, join 后清 0 (ERROR 自退也需 join 防泄漏) */
static PlcVm_t     g_vm;
static uint32_t    g_heartbeat = 0;
static uint32_t    g_cycle_us = 0;

/* ---- P1-3: RT 看门狗镜像 (relaxed atomic, RT 无锁读) + sim 冻结钩子 ---- */
static _Atomic int       g_state_pub = 0;     /* PlcRunState 镜像 */
static _Atomic uint32_t  g_hb_pub = 0;        /* heartbeat 镜像 */
static volatile int      g_freeze_hb = 0;     /* sim 测试: 冻结 heartbeat 发布 */

static void set_state_locked(int v)
{
    g_plc_state = v;
    atomic_store_explicit(&g_state_pub, v, memory_order_relaxed);
}

int plc_rt_is_running(void)
{
    return atomic_load_explicit(&g_state_pub, memory_order_relaxed)
           == PLC_STATE_RUNNING;
}

uint32_t plc_rt_heartbeat(void)
{
    return atomic_load_explicit(&g_hb_pub, memory_order_relaxed);
}

int plc_core_test_freeze(int on)
{
    extern int g_sim_mode;
    if (!g_sim_mode) return -2;
    g_freeze_hb = on ? 1 : 0;
    printf("[PLC] 测试钩子: heartbeat 发布 %s\n", on ? "冻结" : "恢复");
    return 0;
}

/* P1.5-2: 用户变量强制 (RPC 0x007E / OPC UA PLC.ForceVar) */
int plc_core_force_var(const char *name, double val, int on)
{
    pthread_mutex_lock(&g_plc_mu);
    if (g_plc_state != PLC_STATE_RUNNING || g_plc_prog == NULL) {
        pthread_mutex_unlock(&g_plc_mu);
        return -3;
    }
    if ((name == NULL || name[0] == '\0') && on == 0) {
        for (int i = 0; i < PLC_USER_VAR_MAX; i++)
            atomic_store_explicit(&g_vm.force_on[i], 0, memory_order_release);
        pthread_mutex_unlock(&g_plc_mu);
        printf("[PLC] 强制全部清除\n");
        return 0;
    }
    /* 名字大写规范化后查用户区 */
    char up[PLC_NAME_MAX];
    int n = 0;
    for (; name[n] != '\0' && n < PLC_NAME_MAX - 1; n++)
        up[n] = (char)toupper((unsigned char)name[n]);
    up[n] = '\0';
    int base = PlcSysVarCount();
    int found = -1;
    for (int i = base; i < g_plc_prog->n_vars; i++) {
        if (strcmp(g_plc_prog->vars[i].name, up) == 0) { found = i; break; }
    }
    if (found < 0 || found - base >= PLC_USER_VAR_MAX) {
        pthread_mutex_unlock(&g_plc_mu);
        return -1;
    }
    int u = found - base;
    atomic_store_explicit(&g_vm.force_val[u], val, memory_order_release);
    atomic_store_explicit(&g_vm.force_on[u], on ? 1 : 0, memory_order_release);
    pthread_mutex_unlock(&g_plc_mu);
    printf("[PLC] 强制 %s = %.6g %s\n", up, val, on ? "(on)" : "(off)");
    return 0;
}

/* P1.5-3: Trace arm — 名字解析 (系统 G_/F_ + 用户全表) + 启动录制。
 * 锁序: g_plc_mu → g_tr_mu (单向; tick 仅持 g_tr_mu, 无反序死锁)。 */
int plc_core_trace_arm(void)
{
    pthread_mutex_lock(&g_plc_mu);
    if (g_plc_state != PLC_STATE_RUNNING || g_plc_prog == NULL) {
        pthread_mutex_unlock(&g_plc_mu);
        return -3;
    }
    PlcTraceCfg_t cfg;
    plc_trace_get_cfg(&cfg);
    int16_t idx[PLC_TRACE_CH_MAX];
    int n_vars = g_plc_prog->n_vars;
    for (int c = 0; c < cfg.n_ch && c < PLC_TRACE_CH_MAX; c++) {
        char up[PLC_NAME_MAX];
        int k = 0;
        for (; cfg.names[c][k] != '\0' && k < PLC_NAME_MAX - 1; k++)
            up[k] = (char)toupper((unsigned char)cfg.names[c][k]);
        up[k] = '\0';
        int found = -1;
        for (int i = 0; i < n_vars; i++) {
            if (strcmp(g_plc_prog->vars[i].name, up) == 0) { found = i; break; }
        }
        if (found < 0) {
            pthread_mutex_unlock(&g_plc_mu);
            printf("[PLC] trace arm 失败: 通道 %d '%s' 不存在\n", c, up);
            return -1;
        }
        idx[c] = (int16_t)found;
    }
    int rc = plc_trace_arm(idx);
    pthread_mutex_unlock(&g_plc_mu);
    return rc;
}

/* F 系统变量在变量表中的序号 (init 时按名解析, 免受表序变更影响) */
static int g_idx_f_ok = -1, g_idx_f_fin = -1, g_idx_f_st = -1;

/* ---- 变量快照 (RPC 读 / 扫描线程写) ---- */
static pthread_mutex_t g_snap_mu = PTHREAD_MUTEX_INITIALIZER;
static PlcVarSnap_t g_snap[PLC_USER_VAR_MAX + 4];
static int g_snap_n = 0;

/* =====================================================================
 *  时间工具
 * ===================================================================== */
static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void sleep_until(uint64_t deadline_ns)
{
    struct timespec ts;
    ts.tv_sec = (time_t)(deadline_ns / 1000000000ull);
    ts.tv_nsec = (long)(deadline_ns % 1000000000ull);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* =====================================================================
 *  扫描线程
 * ===================================================================== */
static void publish_f_region(void)
{
    PeriphF_t *f = PeriphBus_GetF();
    atomic_store_explicit(&f->heartbeat,
                          atomic_load_explicit(&g_hb_pub, memory_order_relaxed),
                          memory_order_relaxed);
    atomic_store_explicit(&f->f_version, 1, memory_order_relaxed);
    if (g_idx_f_ok >= 0)
        atomic_store_explicit(&f->peripheral_ok,
                              (uint16_t)g_vm.cells[g_idx_f_ok].v.i,
                              memory_order_release);
    if (g_idx_f_fin >= 0)
        atomic_store_explicit(&f->mcode_fin,
                              (uint16_t)g_vm.cells[g_idx_f_fin].v.i,
                              memory_order_release);
    if (g_idx_f_st >= 0)
        atomic_store_explicit(&f->f_status,
                              (uint16_t)g_vm.cells[g_idx_f_st].v.i,
                              memory_order_release);
}

static void update_snapshot(void)
{
    pthread_mutex_lock(&g_snap_mu);
    g_snap_n = 0;
    if (g_plc_prog != NULL) {
        for (int i = PlcSysVarCount(); i < g_plc_prog->n_vars
             && g_snap_n < PLC_USER_VAR_MAX; i++) {
            PlcVarSnap_t *s = &g_snap[g_snap_n++];
            strncpy(s->name, g_plc_prog->vars[i].name, 31);
            s->name[31] = '\0';
            s->type = g_plc_prog->vars[i].type;
            s->cls  = g_plc_prog->vars[i].cls;
            s->fb   = g_plc_prog->vars[i].fb;
            s->forced = atomic_load_explicit(
                &g_vm.force_on[i - PlcSysVarCount()], memory_order_acquire);
            s->value = (g_plc_prog->vars[i].type == PLC_T_REAL)
                       ? g_vm.cells[i].v.d : (double)g_vm.cells[i].v.i;
        }
        /* F 系统变量 3 个附加在尾 (watch 可见 PLC 侧发布值) */
        int fidx[3] = { g_idx_f_ok, g_idx_f_fin, g_idx_f_st };
        for (int k = 0; k < 3; k++) {
            if (fidx[k] < 0 || g_snap_n >= PLC_USER_VAR_MAX + 3) continue;
            PlcVarSnap_t *s = &g_snap[g_snap_n++];
            strncpy(s->name, g_plc_prog->vars[fidx[k]].name, 31);
            s->name[31] = '\0';
            s->type = PLC_T_INT;
            s->cls  = PLC_CLS_SYS_F;
            s->fb   = PLC_FB_NONE;
            s->value = (double)g_vm.cells[fidx[k]].v.i;
        }
    }
    pthread_mutex_unlock(&g_snap_mu);
}

static void *scan_thread_main(void *arg)
{
    (void)arg;
    PeriphG_t g;
    memset(&g, 0, sizeof(g));
    uint64_t last = now_ns();

    printf("[PLC] 扫描线程启动 (10ms, SCHED_OTHER)\n");

    while (g_scan_run) {
        uint64_t t0 = now_ns();
        int64_t dt_ms = (int64_t)((t0 - last) / 1000000ull);
        last = t0;
        if (dt_ms < 1)   dt_ms = 1;
        if (dt_ms > 1000) dt_ms = 1000;

        PeriphG_t g_try;
        if (PeriphBus_ReadG(&g_try) == 0) g = g_try;   /* 失败沿用上周期 */

        /* WHILE 超时界限: 2× 周期 */
        int rc = PlcVm_Cycle(&g_vm, &g, dt_ms, t0 + 20ull * 1000000ull);
        if (rc != 0) {
            pthread_mutex_lock(&g_plc_mu);
            set_state_locked(PLC_STATE_ERROR);
            pthread_mutex_unlock(&g_plc_mu);
            update_snapshot();    /* 保留出错前状态供诊断 */
            printf("[PLC] 运行期错误: 行 %d %s (线程退出, 需 reset)\n",
                   g_vm.err_line, g_vm.err_msg);
            /* P1-4: 运行期错误事件 (扫描线程 SCHED_OTHER 上下文, Push 安全) */
            EventLogger_Push(SEVERITY_ALARM, SOURCE_PLC, 0x0006,
                             g_vm.err_line, "PLC runtime error");
            g_scan_run = 0;
            break;
        }

        /* P1.5-2: 强制表应用 (须在 F 发布与快照之前 — 呈现强制值) */
        PlcVm_ApplyForce(&g_vm);

        /* P1.5-3: Trace 采样 (强制值之后 → trace 与 watch 同源) */
        plc_trace_tick(g_plc_prog, g_vm.cells, g_heartbeat);

        if (!g_freeze_hb) {
            g_heartbeat++;
            atomic_store_explicit(&g_hb_pub, g_heartbeat, memory_order_relaxed);
        }
        publish_f_region();
        update_snapshot();

        uint64_t t1 = now_ns();
        g_cycle_us = (uint32_t)((t1 - t0) / 1000ull);

        sleep_until(t0 + 10ull * 1000000ull);
    }

    printf("[PLC] 扫描线程退出 (heartbeat=%u)\n", g_heartbeat);
    return NULL;
}

/* =====================================================================
 *  init / autoload
 * ===================================================================== */
static void resolve_f_indexes(void)
{
    int n = PlcSysVarCount();
    for (int i = 0; i < n; i++) {
        const char *nm = PlcSysVarName(i);
        if (nm == NULL) continue;
        if      (strcmp(nm, "F_PERIPHERAL_OK") == 0) g_idx_f_ok  = i;
        else if (strcmp(nm, "F_MCODE_FIN") == 0)     g_idx_f_fin = i;
        else if (strcmp(nm, "F_STATUS") == 0)        g_idx_f_st  = i;
    }
}

int plc_core_init(void)
{
    pthread_mutex_lock(&g_plc_mu);
    g_scan_run = 0;
    if (g_plc_prog != NULL) {
        PlcProgramFree(g_plc_prog);
        g_plc_prog = NULL;
    }
    g_plc_text[0] = '\0';
    g_plc_text_len = 0;
    set_state_locked(PLC_STATE_EMPTY);
    pthread_mutex_unlock(&g_plc_mu);

    g_heartbeat = 0;
    g_cycle_us = 0;
    g_freeze_hb = 0;
    atomic_store_explicit(&g_hb_pub, 0, memory_order_relaxed);
    resolve_f_indexes();

    /* boot autoload (D5): ./plc_prog.st 存在即加载并启动 */
    FILE *fp = fopen("plc_prog.st", "rb");
    if (fp == NULL) return 0;
    static char buf[PLC_PROG_TEXT_MAX];
    size_t n = fread(buf, 1, PLC_PROG_TEXT_MAX, fp);
    fclose(fp);
    if (n == 0) {
        printf("[PLC] plc_prog.st 为空, 忽略 autoload\n");
        return 0;
    }
    int nd = 0;
    int rc = plc_core_download(buf, (int)n, plc_crc32((const uint8_t *)buf, (int)n),
                               NULL, 0, &nd);
    if (rc != 0) {
        printf("[PLC] plc_prog.st autoload 编译失败 (rc=%d diag=%d), 忽略\n", rc, nd);
        return 0;
    }
    printf("[PLC] plc_prog.st autoload (%zu 字节)\n", n);
    plc_core_ctrl(1);
    return 0;
}

/* =====================================================================
 *  下载 / 上传
 * ===================================================================== */
int plc_core_download(const char *text, int len, uint32_t crc_expect,
                      PlcDiag_t *diags, int diag_max, int *n_diag)
{
    if (n_diag != NULL) *n_diag = 0;

    if (text == NULL || len < 1 || len > PLC_PROG_TEXT_MAX) return -2;
    if (plc_crc32((const uint8_t *)text, len) != crc_expect) return -3;

    pthread_mutex_lock(&g_plc_mu);
    if (g_plc_state == PLC_STATE_RUNNING) {
        pthread_mutex_unlock(&g_plc_mu);
        return -4;
    }

    PlcProgram_t *prog = NULL;
    int nd = 0;
    int rc = PlcCompile(text, len, &prog, diags, diag_max, &nd);
    if (n_diag != NULL) *n_diag = nd;
    if (rc != 0) {
        /* 编译失败: 原槽保持不变 (旧程序继续有效) */
        pthread_mutex_unlock(&g_plc_mu);
        /* P1-4: 编译错误事件 (value=首错误行; 常量串) */
        EventLogger_Push(SEVERITY_WARN, SOURCE_PLC, 0x0001,
                         (nd > 0 && diags != NULL) ? diags[0].line : 0,
                         "PLC program compile error");
        return -1;
    }

    if (g_plc_prog != NULL) PlcProgramFree(g_plc_prog);
    g_plc_prog = prog;
    memcpy(g_plc_text, text, (size_t)len);
    g_plc_text[len] = '\0';
    g_plc_text_len = len;
    set_state_locked(PLC_STATE_STOPPED);
    pthread_mutex_unlock(&g_plc_mu);

    printf("[PLC] 程序下载编译成功: %d 字节, %d 用户变量, %d 语句 (未启动)\n",
           len, prog->n_user_vars, prog->n_stmts);
    return 0;
}

int plc_core_get_text(const char **out_text, int *out_len)
{
    if (out_text == NULL || out_len == NULL) return -1;
    pthread_mutex_lock(&g_plc_mu);
    if (g_plc_text_len <= 0) {
        pthread_mutex_unlock(&g_plc_mu);
        *out_text = NULL;
        *out_len = 0;
        return -1;
    }
    *out_text = g_plc_text;
    *out_len = g_plc_text_len;
    pthread_mutex_unlock(&g_plc_mu);
    return 0;
}

/* =====================================================================
 *  启停 / 状态 / 快照
 * ===================================================================== */
int plc_core_ctrl(int action)
{
    pthread_mutex_lock(&g_plc_mu);

    if (action == 1) {    /* start */
        if (g_plc_prog == NULL || g_plc_state == PLC_STATE_RUNNING) {
            pthread_mutex_unlock(&g_plc_mu);
            return -1;
        }
        if (PlcVm_Init(&g_vm, g_plc_prog) != 0) {
            pthread_mutex_unlock(&g_plc_mu);
            return -1;
        }
        g_heartbeat = 0;
        g_scan_run = 1;
        if (pthread_create(&g_scan_thread, NULL, scan_thread_main, NULL) != 0) {
            g_scan_run = 0;
            pthread_mutex_unlock(&g_plc_mu);
            printf("[PLC] 扫描线程创建失败\n");
            return -1;
        }
        g_thread_alive = 1;
        set_state_locked(PLC_STATE_RUNNING);
        pthread_mutex_unlock(&g_plc_mu);
        /* P1-4: 启动审计 (value=语句数) */
        EventLogger_Push(SEVERITY_INFO, SOURCE_PLC, 0x0002,
                         g_plc_prog->n_stmts, "PLC program started");
        return 0;
    }

    if (action == 0) {    /* stop */
        if (g_plc_state != PLC_STATE_RUNNING) {
            pthread_mutex_unlock(&g_plc_mu);
            return -1;
        }
        g_scan_run = 0;
        pthread_mutex_unlock(&g_plc_mu);
        if (g_thread_alive) { pthread_join(g_scan_thread, NULL); g_thread_alive = 0; }
        pthread_mutex_lock(&g_plc_mu);
        if (g_plc_state == PLC_STATE_RUNNING) set_state_locked(PLC_STATE_STOPPED);
        pthread_mutex_unlock(&g_plc_mu);
        /* P1-4: 停止审计 (value=累计扫描周期数) */
        EventLogger_Push(SEVERITY_INFO, SOURCE_PLC, 0x0003,
                         (int32_t)g_heartbeat, "PLC program stopped");
        return 0;
    }

    if (action == 2) {    /* reset: ERROR/STOPPED → STOPPED, 变量回初值 */
        if (g_plc_state == PLC_STATE_RUNNING || g_thread_alive) {
            g_scan_run = 0;
            pthread_mutex_unlock(&g_plc_mu);
            if (g_thread_alive) { pthread_join(g_scan_thread, NULL); g_thread_alive = 0; }
            pthread_mutex_lock(&g_plc_mu);
        }
        if (g_plc_prog != NULL) {
            PlcVm_Init(&g_vm, g_plc_prog);   /* 重置变量/FB 状态 */
            g_heartbeat = 0;
        }
        if (g_plc_state != PLC_STATE_EMPTY) set_state_locked(PLC_STATE_STOPPED);
        pthread_mutex_unlock(&g_plc_mu);
        printf("[PLC] reset: 变量回初值, 状态 STOPPED\n");
        return 0;
    }

    pthread_mutex_unlock(&g_plc_mu);
    return -1;
}

int plc_core_get_status(PlcCoreStatus_t *out)
{
    if (out == NULL) return -1;
    pthread_mutex_lock(&g_plc_mu);
    out->state    = g_plc_state;
    out->n_vars   = (g_plc_prog != NULL) ? g_plc_prog->n_user_vars : 0;
    out->n_stmts  = (g_plc_prog != NULL) ? g_plc_prog->n_stmts : 0;
    out->text_len = g_plc_text_len;
    out->heartbeat = g_heartbeat;
    out->cycle_us  = g_cycle_us;
    out->err_line  = g_vm.runtime_err ? g_vm.err_line : 0;
    strncpy(out->err_msg, g_vm.err_msg, 63);
    out->err_msg[63] = '\0';
    pthread_mutex_unlock(&g_plc_mu);
    return 0;
}

int plc_core_get_state(int *out_state, int *out_n_vars, int *out_n_stmts,
                       int *out_text_len)
{
    PlcCoreStatus_t st;
    if (plc_core_get_status(&st) != 0) return -1;
    if (out_state)    *out_state    = st.state;
    if (out_n_vars)   *out_n_vars   = st.n_vars;
    if (out_n_stmts)  *out_n_stmts  = st.n_stmts;
    if (out_text_len) *out_text_len = st.text_len;
    return 0;
}

int plc_core_snapshot_vars(PlcVarSnap_t *out, int max, int *n_total)
{
    pthread_mutex_lock(&g_snap_mu);
    int n = g_snap_n;
    if (n_total != NULL) *n_total = n;
    if (out != NULL && max > 0) {
        int ncopy = (n < max) ? n : max;
        memcpy(out, g_snap, (size_t)ncopy * sizeof(PlcVarSnap_t));
        pthread_mutex_unlock(&g_snap_mu);
        return ncopy;
    }
    pthread_mutex_unlock(&g_snap_mu);
    return 0;
}

/* =====================================================================
 *  CRC
 * ===================================================================== */
uint32_t plc_crc32(const uint8_t *data, int len)
{
    uint32_t c = 0xFFFFFFFFu;
    for (int i = 0; i < len; i++) {
        c ^= data[i];
        for (int k = 0; k < 8; k++) {
            if (c & 1u) c = (c >> 1) ^ 0xEDB88320u;
            else        c >>= 1;
        }
    }
    return c ^ 0xFFFFFFFFu;
}
