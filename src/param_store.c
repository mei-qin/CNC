/* =====================================================================
 *  param_store.c —— 机床参数档案持久化 (P1+P2, 2026-08-28)
 *
 *  设计要点:
 *    1. 编译期默认档案 ps_default_profile() = 原 rpc_server.c g_hw_* 表
 *       的等价物 (前期实机开发测试值), 保证无文件/文件损坏时行为与
 *       改造前完全一致 (零回归风险)。
 *    2. INI 解析为"默认值 + 文件覆盖"合并语义: 缺 key 保默认, 文件里
 *       显式 PS_TODO_* 哨兵维持 IBN 填空分级 (A 拒启动 / B 跳过警告)。
 *    3. notify 钩子按"影子旧值 vs 新值"判变化 — 相等则完全 no-op,
 *       boot 应用期 (file→SMC_Config*) 天然静默, 无需 in_boot 标志。
 *    4. 写入即存: 有变化的 notify → 审计 0x0003 + 原子落盘 (非 sim)。
 *
 *  @Context: 全部 Non-RealTime (boot / RPC handler / OPC UA server 线程)
 *  @Thread-Safety: g_ps_mutex 串行化影子读写与落盘。
 * ===================================================================== */

#include "param_store.h"
#include "event_logger.h"
#include "global_def.h"      /* g_sim_mode */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <stddef.h>         /* offsetof */

#define PS_LINE_BUF   256
#define PS_PATH_BUF   512

/* ---- 全局状态 ---- */
static MachineProfile_t g_profile;          /* 影子档案 (mutex 保护) */
static pthread_mutex_t  g_ps_mutex = PTHREAD_MUTEX_INITIALIZER;
static int              g_ps_saved_sim_warn = 0;  /* sim 拒写只告警一次 */

/* =====================================================================
 *  编译期默认档案 (原 rpc_server.c g_hw_* 表原值, 单一维护点)
 * ===================================================================== */
static void ps_default_profile(MachineProfile_t *p)
{
    memset(p, 0, sizeof(*p));

    p->axis_count = 5;
    strcpy(p->axis_order[0], "X");
    strcpy(p->axis_order[1], "Y");
    strcpy(p->axis_order[2], "Z");
    strcpy(p->axis_order[3], "C");
    strcpy(p->axis_order[4], "B");

    /* 房间号顺序 X,Y,Z,C,B — 与 kernel_init_hw 原始 apply 序列一致 */
    /* X: 单驱 slave 5 */
    p->ax[0].is_dual = 0;  p->ax[0].master_id = 5;  p->ax[0].slave_id = -1;
    p->ax[0].ppu = 10000.0;
    p->ax[0].dyn_type = 0; p->ax[0].max_speed = 50.0;
    p->ax[0].max_acc = 200.0; p->ax[0].max_dec = 200.0; p->ax[0].eq_radius = 0.0;
    p->ax[0].sl_enable = 0; p->ax[0].sl_neg = 0.0; p->ax[0].sl_pos = 0.0;
    p->ax[0].gs_tol_pulse = 0; p->ax[0].gs_max_err_pulse = 0; p->ax[0].gs_err_time_ms = 0;
    p->ax[0].hm_enable = 1; p->ax[0].hm_method = 35;
    p->ax[0].hm_direction = 1; p->ax[0].hm_timeout_ms = 10000; p->ax[0].hm_speed = 10.0;

    /* Y: 双驱龙门 slave 3+4 */
    p->ax[1].is_dual = 1;  p->ax[1].master_id = 3;  p->ax[1].slave_id = 4;
    p->ax[1].ppu = 10000.0;
    p->ax[1].dyn_type = 0; p->ax[1].max_speed = 50.0;
    p->ax[1].max_acc = 200.0; p->ax[1].max_dec = 200.0; p->ax[1].eq_radius = 0.0;
    p->ax[1].sl_enable = 0; p->ax[1].sl_neg = 0.0; p->ax[1].sl_pos = 0.0;
    p->ax[1].gs_tol_pulse = 1000; p->ax[1].gs_max_err_pulse = 8000;
    p->ax[1].gs_err_time_ms = 100;
    p->ax[1].hm_enable = 1; p->ax[1].hm_method = 35;
    p->ax[1].hm_direction = 1; p->ax[1].hm_timeout_ms = 10000; p->ax[1].hm_speed = 10.0;

    /* Z: 单驱 slave 6, 软限位 (-500,+200) */
    p->ax[2].is_dual = 0;  p->ax[2].master_id = 6;  p->ax[2].slave_id = -1;
    p->ax[2].ppu = 1000.0;
    p->ax[2].dyn_type = 0; p->ax[2].max_speed = 30.0;
    p->ax[2].max_acc = 100.0; p->ax[2].max_dec = 100.0; p->ax[2].eq_radius = 0.0;
    p->ax[2].sl_enable = 1; p->ax[2].sl_neg = -500.0; p->ax[2].sl_pos = 200.0;
    p->ax[2].gs_tol_pulse = 0; p->ax[2].gs_max_err_pulse = 0; p->ax[2].gs_err_time_ms = 0;
    p->ax[2].hm_enable = 1; p->ax[2].hm_method = 35;
    p->ax[2].hm_direction = 1; p->ax[2].hm_timeout_ms = 10000; p->ax[2].hm_speed = 10.0;

    /* C: 旋转, 等效半径 50mm */
    p->ax[3].is_dual = 0;  p->ax[3].master_id = 1;  p->ax[3].slave_id = -1;
    p->ax[3].ppu = 2777.7778;
    p->ax[3].dyn_type = 1; p->ax[3].max_speed = 18.0;
    p->ax[3].max_acc = 72.0; p->ax[3].max_dec = 72.0; p->ax[3].eq_radius = 50.0;
    p->ax[3].sl_enable = 0; p->ax[3].sl_neg = 0.0; p->ax[3].sl_pos = 0.0;
    p->ax[3].gs_tol_pulse = 0; p->ax[3].gs_max_err_pulse = 0; p->ax[3].gs_err_time_ms = 0;
    p->ax[3].hm_enable = 1; p->ax[3].hm_method = 35;
    p->ax[3].hm_direction = 1; p->ax[3].hm_timeout_ms = 10000; p->ax[3].hm_speed = 10.0;

    /* B: 旋转, 等效半径 80mm */
    p->ax[4].is_dual = 0;  p->ax[4].master_id = 2;  p->ax[4].slave_id = -1;
    p->ax[4].ppu = 2777.7778;
    p->ax[4].dyn_type = 1; p->ax[4].max_speed = 18.0;
    p->ax[4].max_acc = 72.0; p->ax[4].max_dec = 72.0; p->ax[4].eq_radius = 80.0;
    p->ax[4].sl_enable = 0; p->ax[4].sl_neg = 0.0; p->ax[4].sl_pos = 0.0;
    p->ax[4].gs_tol_pulse = 0; p->ax[4].gs_max_err_pulse = 0; p->ax[4].gs_err_time_ms = 0;
    p->ax[4].hm_enable = 1; p->ax[4].hm_method = 35;
    p->ax[4].hm_direction = 1; p->ax[4].hm_timeout_ms = 10000; p->ax[4].hm_speed = 10.0;

    /* 五轴运动学 (Head-Head 原值) */
    p->tool_len = 150.0;
    p->pivot_x = 0.0; p->pivot_y = 0.0; p->pivot_z = 200.0;

    /* 规划器 (保守默认) */
    p->corner_tolerance = 0.05;
    p->max_centripetal_acc = 500.0;

    /* Z 安全抬升 */
    p->safe_z_mm = 50.0;
    p->lift_speed_mm_s = 20.0;
    /* auto_on_alarm 默认 0 (2026-08-28 实机禁用): 报警自动抬 Z 在寸动调机期
     * 反复误触发 (任一轴跟随误差即抬 Z, 且 DONE 态锁死全轴寸动)。
     * 机制保留: 档案改 1 重新启用; 手动抬升 (RPC 0x0057 / SMC_SafeLiftZ) 不受影响 */
    p->auto_on_alarm = 0;

    /* 回零全局 */
    strcpy(p->homing_order, "ZXYBC");
    p->auto_on_init = 0;
}

/* =====================================================================
 *  轴字段表 — INI 解析 / 档案写出 / 轴查找 共用的单一映射
 * ===================================================================== */
typedef struct {
    const char *key;
    size_t      offset;      /* offsetof(ProfileAxis_t, ...) */
    int         is_double;
} PsAxisField_t;

static const PsAxisField_t g_axis_fields[] = {
    {"is_dual",              offsetof(ProfileAxis_t, is_dual),         0},
    {"master_id",            offsetof(ProfileAxis_t, master_id),       0},
    {"slave_id",             offsetof(ProfileAxis_t, slave_id),        0},
    {"pulse_per_unit",       offsetof(ProfileAxis_t, ppu),             1},
    {"axis_type",            offsetof(ProfileAxis_t, dyn_type),        0},
    {"max_speed",            offsetof(ProfileAxis_t, max_speed),       1},
    {"max_acc",              offsetof(ProfileAxis_t, max_acc),         1},
    {"max_dec",              offsetof(ProfileAxis_t, max_dec),         1},
    {"eq_radius",            offsetof(ProfileAxis_t, eq_radius),       1},
    {"soft_limit_enable",    offsetof(ProfileAxis_t, sl_enable),       0},
    {"soft_limit_neg",       offsetof(ProfileAxis_t, sl_neg),          1},
    {"soft_limit_pos",       offsetof(ProfileAxis_t, sl_pos),          1},
    {"gantry_tol_pulse",     offsetof(ProfileAxis_t, gs_tol_pulse),    0},
    {"gantry_max_err_pulse", offsetof(ProfileAxis_t, gs_max_err_pulse),0},
    {"gantry_err_time_ms",   offsetof(ProfileAxis_t, gs_err_time_ms),  0},
    {"homing_enable",        offsetof(ProfileAxis_t, hm_enable),       0},
    {"homing_method",        offsetof(ProfileAxis_t, hm_method),       0},
    {"homing_direction",     offsetof(ProfileAxis_t, hm_direction),    0},
    {"homing_timeout_ms",    offsetof(ProfileAxis_t, hm_timeout_ms),   0},
    {"homing_speed",         offsetof(ProfileAxis_t, hm_speed),        1},
};

/* 轴字母 → 档案下标 (axis_order 顺序); 找不到返回 -1 */
static int ps_find_axis(const MachineProfile_t *p, char letter)
{
    char c = (char)toupper((unsigned char)letter);
    for (int i = 0; i < p->axis_count && i < AXIS_NUM; i++) {
        if (toupper((unsigned char)p->axis_order[i][0]) == c) return i;
    }
    return -1;
}

/* 单行工具: 去首尾空白; 返回修剪后指针 */
static char *ps_trim(char *s)
{
    while (isspace((unsigned char)*s)) s++;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) *--end = '\0';
    return s;
}

/* =====================================================================
 *  INI 解析 (默认值 + 文件覆盖)
 * ===================================================================== */
static int ps_parse_axis_line(MachineProfile_t *p, char axis_letter,
                              const char *key, const char *val, int lineno)
{
    int ai = ps_find_axis(p, axis_letter);
    if (ai < 0) {
        printf("[param_store] 行 %d: 轴 '%c' 不在 axis_order 中, 拒绝\n",
               lineno, axis_letter);
        return -1;
    }
    for (size_t f = 0; f < sizeof(g_axis_fields) / sizeof(g_axis_fields[0]); f++) {
        if (strcmp(key, g_axis_fields[f].key) != 0) continue;
        void *field = (char *)&p->ax[ai] + g_axis_fields[f].offset;
        if (g_axis_fields[f].is_double)
            *(double *)field = strtod(val, NULL);
        else
            *(int *)field = (int)strtol(val, NULL, 10);
        return 0;
    }
    printf("[param_store] 行 %d: 未知轴参数 '%s.%s' (忽略)\n",
           lineno, p->axis_order[ai], key);
    return 0;   /* 未知轴 key 容忍 (前向兼容) */
}

static int ps_parse_global_line(MachineProfile_t *p, const char *section,
                                const char *key, const char *val, int lineno)
{
    if (strcmp(section, "general") == 0) {
        /* version/axis_order 在 ps_parse_file 专门处理, 此处兜底忽略 */
        (void)key; (void)val;
        return 0;
    }
    if (strcmp(section, "kinematics") == 0) {
        if      (strcmp(key, "tool_len") == 0) p->tool_len = strtod(val, NULL);
        else if (strcmp(key, "pivot_x") == 0)  p->pivot_x  = strtod(val, NULL);
        else if (strcmp(key, "pivot_y") == 0)  p->pivot_y  = strtod(val, NULL);
        else if (strcmp(key, "pivot_z") == 0)  p->pivot_z  = strtod(val, NULL);
        else { printf("[param_store] 行 %d: 未知 [kinematics] key '%s' (忽略)\n", lineno, key); }
        return 0;
    }
    if (strcmp(section, "planner") == 0) {
        if      (strcmp(key, "corner_tolerance") == 0)   p->corner_tolerance   = strtod(val, NULL);
        else if (strcmp(key, "max_centripetal_acc") == 0) p->max_centripetal_acc = strtod(val, NULL);
        else { printf("[param_store] 行 %d: 未知 [planner] key '%s' (忽略)\n", lineno, key); }
        return 0;
    }
    if (strcmp(section, "safelift") == 0) {
        if      (strcmp(key, "safe_z_mm") == 0)        p->safe_z_mm       = strtod(val, NULL);
        else if (strcmp(key, "lift_speed_mm_s") == 0)  p->lift_speed_mm_s = strtod(val, NULL);
        else if (strcmp(key, "auto_on_alarm") == 0)    p->auto_on_alarm   = (int)strtol(val, NULL, 10);
        else { printf("[param_store] 行 %d: 未知 [safelift] key '%s' (忽略)\n", lineno, key); }
        return 0;
    }
    if (strcmp(section, "homing") == 0) {
        if      (strcmp(key, "order") == 0)        strncpy(p->homing_order, val, sizeof(p->homing_order) - 1);
        else if (strcmp(key, "auto_on_init") == 0) p->auto_on_init = (int)strtol(val, NULL, 10);
        else { printf("[param_store] 行 %d: 未知 [homing] key '%s' (忽略)\n", lineno, key); }
        return 0;
    }
    return -1;   /* 未知 section — 由调用方决定是否致命 */
}

/* @return 0=解析成功; -1=致命 (版本不符/未知轴/坏 axis_order) */
static int ps_parse_file(MachineProfile_t *p, FILE *fp)
{
    char line[PS_LINE_BUF];
    char section[32] = "";
    char cur_axis = '\0';
    int  lineno = 0;
    int  version_ok = 0;
    int  order_ok = 0;

    while (fgets(line, sizeof(line), fp)) {
        lineno++;
        char *s = ps_trim(line);
        if (*s == '\0' || *s == ';' || *s == '#') continue;

        if (*s == '[') {
            char *close = strchr(s, ']');
            if (!close) { printf("[param_store] 行 %d: 坏 section 头\n", lineno); return -1; }
            *close = '\0';
            strncpy(section, ps_trim(s + 1), sizeof(section) - 1);
            section[sizeof(section) - 1] = '\0';

            cur_axis = '\0';
            if (strncmp(section, "axis.", 5) == 0 && section[5] != '\0') {
                cur_axis = section[5];
                if (strlen(section) != 6) {   /* 仅接受单字母 [axis.X] */
                    printf("[param_store] 行 %d: 轴 section 必须单字母 '%s'\n", lineno, section);
                    return -1;
                }
                if (ps_find_axis(p, cur_axis) < 0) {
                    printf("[param_store] 行 %d: 轴 '%c' 不在 axis_order 中\n", lineno, cur_axis);
                    return -1;
                }
            }
            continue;
        }

        char *eq = strchr(s, '=');
        if (!eq) { printf("[param_store] 行 %d: 非 key=value 行\n", lineno); return -1; }
        *eq = '\0';
        char *key = ps_trim(s);
        char *val = ps_trim(eq + 1);
        if (*key == '\0') { printf("[param_store] 行 %d: 空 key\n", lineno); return -1; }

        if (strcmp(section, "general") == 0) {
            if (strcmp(key, "version") == 0) {
                version_ok = ((int)strtol(val, NULL, 10) == PS_PROFILE_VERSION);
            } else if (strcmp(key, "axis_order") == 0) {
                /* 解析逗号分隔轴序; 要求 1..AXIS_NUM 个不重复单字母 */
                int n = 0, dup = 0;
                char buf[64];
                strncpy(buf, val, sizeof(buf) - 1);
                buf[sizeof(buf) - 1] = '\0';
                for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
                    char *t = ps_trim(tok);
                    if (*t == '\0' || strlen(t) != 1 || n >= AXIS_NUM) { dup = 1; break; }
                    for (int i = 0; i < n; i++)
                        if (p->axis_order[i][0] == *t) { dup = 1; break; }
                    if (dup) break;
                    p->axis_order[n][0] = (char)toupper((unsigned char)*t);
                    p->axis_order[n][1] = '\0';
                    n++;
                }
                if (dup || n < 1) {
                    printf("[param_store] 行 %d: 非法 axis_order '%s'\n", lineno, val);
                    return -1;
                }
                p->axis_count = n;
                order_ok = 1;
            }
            continue;
        }

        if (cur_axis != '\0') {
            if (ps_parse_axis_line(p, cur_axis, key, val, lineno) != 0) return -1;
        } else {
            if (ps_parse_global_line(p, section, key, val, lineno) != 0) {
                printf("[param_store] 行 %d: 未知 section '%s'\n", lineno, section);
                return -1;
            }
        }
    }

    if (!version_ok) {
        printf("[param_store] 缺 version=%d 头或版本不符, 拒绝加载\n", PS_PROFILE_VERSION);
        return -1;
    }
    if (!order_ok) {
        printf("[param_store] 缺 axis_order, 拒绝加载\n");
        return -1;
    }
    return 0;
}

/* =====================================================================
 *  档案写出 (tmp + rename + fsync 原子)
 * ===================================================================== */
static void ps_write_axis_block(FILE *fp, const MachineProfile_t *p, int i)
{
    const ProfileAxis_t *a = &p->ax[i];
    fprintf(fp, "[axis.%s]\n", p->axis_order[i]);
    fprintf(fp, "is_dual=%d\n",              a->is_dual);
    fprintf(fp, "master_id=%d\n",            a->master_id);
    fprintf(fp, "slave_id=%d\n",             a->slave_id);
    fprintf(fp, "pulse_per_unit=%.6f\n",     a->ppu);
    fprintf(fp, "axis_type=%d\n",            a->dyn_type);
    fprintf(fp, "max_speed=%.6f\n",          a->max_speed);
    fprintf(fp, "max_acc=%.6f\n",            a->max_acc);
    fprintf(fp, "max_dec=%.6f\n",            a->max_dec);
    fprintf(fp, "eq_radius=%.6f\n",          a->eq_radius);
    fprintf(fp, "soft_limit_enable=%d\n",    a->sl_enable);
    fprintf(fp, "soft_limit_neg=%.6f\n",     a->sl_neg);
    fprintf(fp, "soft_limit_pos=%.6f\n",     a->sl_pos);
    fprintf(fp, "gantry_tol_pulse=%d\n",     a->gs_tol_pulse);
    fprintf(fp, "gantry_max_err_pulse=%d\n", a->gs_max_err_pulse);
    fprintf(fp, "gantry_err_time_ms=%d\n",   a->gs_err_time_ms);
    fprintf(fp, "homing_enable=%d\n",        a->hm_enable);
    fprintf(fp, "homing_method=%d\n",        a->hm_method);
    fprintf(fp, "homing_direction=%d\n",     a->hm_direction);
    fprintf(fp, "homing_timeout_ms=%d\n",    a->hm_timeout_ms);
    fprintf(fp, "homing_speed=%.6f\n",       a->hm_speed);
    fprintf(fp, "\n");
}

/* @return 0=成功; -1=sim 拒绝; -2=I/O 失败 (调用方持锁状态之外使用) */
static int ps_save_locked(const MachineProfile_t *p)
{
    if (g_sim_mode) {
        /* 防污染语义: sim 会话写【默认路径】会覆盖实机同目录档案 → 拒绝。
         * 显式 SMC_MACHINE_PROFILE 指向的档案 (测试/联调专用路径, 如 /tmp/xx)
         * 允许落盘 — 否则 sim 下保存链路 (写入即存/SaveProfile) 无法测试。 */
        const char *env = getenv("SMC_MACHINE_PROFILE");
        if (!env || env[0] == '\0') {
            if (!g_ps_saved_sim_warn) {
                printf("[param_store] sim 模式: 默认路径档案不落盘 "
                       "(防污染实机参数; 测试请设 SMC_MACHINE_PROFILE)\n");
                g_ps_saved_sim_warn = 1;
            }
            return -1;
        }
    }

    const char *path = param_store_path();
    char tmp[PS_PATH_BUF + 8];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);

    FILE *fp = fopen(tmp, "w");
    if (!fp) {
        printf("[param_store] 打开 %s 失败: %s\n", tmp, strerror(errno));
        return -2;
    }

    fprintf(fp, "; SMC machine profile v%d — 机床参数档案 (IBN)\n", PS_PROFILE_VERSION);
    fprintf(fp, "; rpc_server 自动生成/更新; 手工修改后重启进程生效\n");
    fprintf(fp, "; 备份/恢复 = 复制本文件\n\n");

    fprintf(fp, "[general]\n");
    fprintf(fp, "version=%d\n", PS_PROFILE_VERSION);
    fprintf(fp, "axis_order=");
    for (int i = 0; i < p->axis_count; i++)
        fprintf(fp, "%s%s", (i ? "," : ""), p->axis_order[i]);
    fprintf(fp, "\n\n");

    for (int i = 0; i < p->axis_count && i < AXIS_NUM; i++)
        ps_write_axis_block(fp, p, i);

    fprintf(fp, "[kinematics]\n");
    fprintf(fp, "tool_len=%.6f\n", p->tool_len);
    fprintf(fp, "pivot_x=%.6f\n",  p->pivot_x);
    fprintf(fp, "pivot_y=%.6f\n",  p->pivot_y);
    fprintf(fp, "pivot_z=%.6f\n",  p->pivot_z);
    fprintf(fp, "\n[planner]\n");
    fprintf(fp, "corner_tolerance=%.6f\n",   p->corner_tolerance);
    fprintf(fp, "max_centripetal_acc=%.6f\n", p->max_centripetal_acc);
    fprintf(fp, "\n[safelift]\n");
    fprintf(fp, "safe_z_mm=%.6f\n",      p->safe_z_mm);
    fprintf(fp, "lift_speed_mm_s=%.6f\n", p->lift_speed_mm_s);
    fprintf(fp, "auto_on_alarm=%d\n",    p->auto_on_alarm);
    fprintf(fp, "\n[homing]\n");
    fprintf(fp, "order=%s\n",        p->homing_order);
    fprintf(fp, "auto_on_init=%d\n", p->auto_on_init);
    fprintf(fp, "\n");

    if (fflush(fp) != 0 || fsync(fileno(fp)) != 0) {
        printf("[param_store] flush/fsync %s 失败\n", tmp);
        fclose(fp);
        unlink(tmp);
        return -2;
    }
    fclose(fp);

    if (rename(tmp, path) != 0) {
        printf("[param_store] rename %s→%s 失败: %s\n", tmp, path, strerror(errno));
        unlink(tmp);
        return -2;
    }
    /* 目录项持久化 (掉电安全 best-effort) */
    int dfd = open(".", O_RDONLY | O_DIRECTORY);
    if (dfd >= 0) { fsync(dfd); close(dfd); }
    return 0;
}

/* =====================================================================
 *  公开 API
 * ===================================================================== */
const char *param_store_path(void)
{
    const char *env = getenv("SMC_MACHINE_PROFILE");
    static char path[PS_PATH_BUF];
    if (env && env[0] != '\0') {
        strncpy(path, env, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
        return path;
    }
    return "machine_profile.ini";
}

void param_store_load(void)
{
    pthread_mutex_lock(&g_ps_mutex);
    ps_default_profile(&g_profile);

    const char *path = param_store_path();
    FILE *fp = fopen(path, "r");
    if (!fp) {
        g_profile.loaded = 1;
        g_profile.dirty = 0;
        pthread_mutex_unlock(&g_ps_mutex);
        printf("[param_store] 无档案文件 %s, 用编译期默认 (首次变更时自动生成)\n", path);
        EventLogger_Push(SEVERITY_WARN, SOURCE_CONFIG, PS_EV_LOAD_FALLBACK, 0,
                         "profile file missing, factory defaults in use");
        return;
    }

    /* axis_order 先复位为空, 由文件 [general] 提供 (解析器校验唯一性) */
    if (ps_parse_file(&g_profile, fp) != 0) {
        fclose(fp);
        ps_default_profile(&g_profile);   /* 损坏 → 完整回退默认 */
        g_profile.loaded = 1;
        g_profile.dirty = 0;
        pthread_mutex_unlock(&g_ps_mutex);
        printf("[param_store] 档案解析失败, 回退编译期默认!\n");
        EventLogger_Push(SEVERITY_WARN, SOURCE_CONFIG, PS_EV_LOAD_FALLBACK, 1,
                         "profile parse failed, factory defaults in use");
        return;
    }
    fclose(fp);

    g_profile.loaded = 1;
    g_profile.dirty = 0;
    pthread_mutex_unlock(&g_ps_mutex);
    printf("[param_store] 档案已加载: %s (%d 轴)\n", path, g_profile.axis_count);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_LOAD_OK,
                     g_profile.axis_count, "machine profile loaded from file");
}

const MachineProfile_t *param_store_get(void)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) ps_default_profile(&g_profile);   /* 未 load 的防御快照 */
    pthread_mutex_unlock(&g_ps_mutex);
    return &g_profile;
}

int param_store_check(void)
{
    const MachineProfile_t *p = param_store_get();
    int fatal = 0;

    for (int i = 0; i < p->axis_count && i < AXIS_NUM; i++) {
        const ProfileAxis_t *a = &p->ax[i];
        /* A 级: 拓扑 / 当量 / 软限位 */
        if (a->master_id == PS_TODO_I
            || (a->is_dual == 1 && a->slave_id == PS_TODO_I)) {
            printf("[hw][A?] 拓扑未填: %s (master/slave 从站 ID)\n", p->axis_order[i]);
            fatal = 1;
        }
        if (a->ppu == PS_TODO_D || a->ppu <= 0.0) {
            printf("[hw][A?] 脉冲当量未填或非法: %s\n", p->axis_order[i]);
            fatal = 1;
        }
        if (a->sl_enable
            && (a->sl_neg == PS_TODO_D || a->sl_pos == PS_TODO_D)) {
            printf("[hw][A?] 软限位未填: %s\n", p->axis_order[i]);
            fatal = 1;
        }
        /* B 级: 动力学 */
        if (a->max_speed == PS_TODO_D)
            printf("[hw][B?] 动力学未填: %s (apply 跳过, 用系统保守默认)\n",
                   p->axis_order[i]);
    }
    if (p->safe_z_mm == PS_TODO_D) {
        printf("[hw][A?] SafeLiftZ 安全高度未填 (激光头防撞, 必填)\n");
        fatal = 1;
    }
    if (p->tool_len == PS_TODO_D || p->pivot_z == PS_TODO_D)
        printf("[hw][B?] 五轴运动学未填 (apply 跳过, RTCP 不可用)\n");

    if (fatal) {
        fprintf(stderr, "[param_store] A 级参数存在未填项, 拒绝启动 (防 sim 值上实机)\n");
        return -1;
    }
    printf("[param_store] 档案校验通过 (A 级已填满)\n");
    return 0;
}

int param_store_save(void)
{
    pthread_mutex_lock(&g_ps_mutex);
    int rc = ps_save_locked(&g_profile);
    if (rc == 0) {
        g_profile.dirty = 0;
        EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_SAVE, 0,
                         "machine profile saved");
    } else if (rc == -2) {
        EventLogger_Push(SEVERITY_ALARM, SOURCE_CONFIG, PS_EV_SAVE, -1,
                         "machine profile save FAILED");
    }
    pthread_mutex_unlock(&g_ps_mutex);
    return rc;
}

int param_store_dirty(void)
{
    pthread_mutex_lock(&g_ps_mutex);
    int d = g_profile.dirty;
    pthread_mutex_unlock(&g_ps_mutex);
    return d;
}

void param_store_mark_saved(void)
{
    pthread_mutex_lock(&g_ps_mutex);
    g_profile.dirty = 0;
    pthread_mutex_unlock(&g_ps_mutex);
}

int param_store_set_axis_ppu(char axis_letter, double ppu)
{
    if (ppu <= 0.0) return -4;
    pthread_mutex_lock(&g_ps_mutex);
    int ai = ps_find_axis(&g_profile, axis_letter);
    if (ai < 0) { pthread_mutex_unlock(&g_ps_mutex); return -1; }
    double old = g_profile.ax[ai].ppu;
    char name[8];
    strncpy(name, g_profile.axis_order[ai], sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    g_profile.ax[ai].ppu = ppu;
    g_profile.dirty = 1;
    int rc = ps_save_locked(&g_profile);
    if (rc == 0) g_profile.dirty = 0;
    pthread_mutex_unlock(&g_ps_mutex);

    if (rc == 0) {
        char msg[SMC_EVENT_MSG_LEN];
        snprintf(msg, sizeof(msg), "ppu %s: %.4f->%.4f (pending restart)",
                 name, old, ppu);
        EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE,
                         (int32_t)(ppu * 1000.0), msg);
    }
    return (rc == 0) ? 0 : rc;
}

/* =====================================================================
 *  notify 钩子族 — 影子旧值 vs 新值判变化
 * =====================================================================
 * 公共尾处理: changed!=0 → dirty + 审计 0x0003 + 写入即存 (非 sim)。
 * EventLogger_Push 在锁外调 (其内部无锁, 但保持与项目其他调用点一致的
 * "锁外推送" 习惯, 避免潜在重入)。
 */
static int ps_commit_change(void)
{
    g_profile.dirty = 1;
    int rc = ps_save_locked(&g_profile);
    if (rc == 0) g_profile.dirty = 0;
    return rc;
}

void param_store_notify_topo(const char *axis_name, int is_dual, int master, int slave)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    int ai = (axis_name && axis_name[0]) ? ps_find_axis(&g_profile, axis_name[0]) : -1;
    if (ai < 0) { pthread_mutex_unlock(&g_ps_mutex); return; }
    ProfileAxis_t *a = &g_profile.ax[ai];
    if (a->is_dual == is_dual && a->master_id == master && a->slave_id == slave) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;   /* 无变化 (boot 应用期常态) */
    }
    char name[8];
    strncpy(name, g_profile.axis_order[ai], sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    int old_m = a->master_id, old_s = a->slave_id;
    a->is_dual = is_dual; a->master_id = master; a->slave_id = slave;
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "topo %s: M%d/S%d -> M%d/S%d",
             name, old_m, old_s, master, slave);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE, master, msg);
}

void param_store_notify_ppu(char axis_letter, double new_ppu)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    int ai = ps_find_axis(&g_profile, axis_letter);
    if (ai < 0) { pthread_mutex_unlock(&g_ps_mutex); return; }
    double old = g_profile.ax[ai].ppu;
    if (old == new_ppu) { pthread_mutex_unlock(&g_ps_mutex); return; }
    g_profile.ax[ai].ppu = new_ppu;
    char name[8];
    strncpy(name, g_profile.axis_order[ai], sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "ppu %s: %.4f -> %.4f", name, old, new_ppu);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE,
                     (int32_t)(new_ppu * 1000.0), msg);
}

void param_store_notify_dyn(char axis_letter, int type,
                            double v, double a, double d, double r)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    int ai = ps_find_axis(&g_profile, axis_letter);
    if (ai < 0) { pthread_mutex_unlock(&g_ps_mutex); return; }
    ProfileAxis_t *ax = &g_profile.ax[ai];
    if (ax->dyn_type == type && ax->max_speed == v && ax->max_acc == a
        && ax->max_dec == d && ax->eq_radius == r) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    double old_v = ax->max_speed;
    ax->dyn_type = type; ax->max_speed = v; ax->max_acc = a;
    ax->max_dec = d; ax->eq_radius = r;
    char name[8];
    strncpy(name, g_profile.axis_order[ai], sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "dyn %s: v %.1f->%.1f (a=%.1f d=%.1f r=%.1f t=%d)",
             name, old_v, v, a, d, r, type);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE,
                     (int32_t)(v * 100.0), msg);
}

void param_store_notify_softlimit(char axis_letter, int en, double neg, double pos)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    int ai = ps_find_axis(&g_profile, axis_letter);
    if (ai < 0) { pthread_mutex_unlock(&g_ps_mutex); return; }
    ProfileAxis_t *ax = &g_profile.ax[ai];
    if (ax->sl_enable == en && ax->sl_neg == neg && ax->sl_pos == pos) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    ax->sl_enable = en; ax->sl_neg = neg; ax->sl_pos = pos;
    char name[8];
    strncpy(name, g_profile.axis_order[ai], sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "softlimit %s: en=%d [%.1f, %.1f]", name, en, neg, pos);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE, en, msg);
}

void param_store_notify_gantry_sync(char axis_letter, int tol, int max, int time_ms)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    int ai = ps_find_axis(&g_profile, axis_letter);
    if (ai < 0) { pthread_mutex_unlock(&g_ps_mutex); return; }
    ProfileAxis_t *ax = &g_profile.ax[ai];
    if (ax->gs_tol_pulse == tol && ax->gs_max_err_pulse == max
        && ax->gs_err_time_ms == time_ms) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    ax->gs_tol_pulse = tol; ax->gs_max_err_pulse = max; ax->gs_err_time_ms = time_ms;
    char name[8];
    strncpy(name, g_profile.axis_order[ai], sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "gantry_sync %s: tol=%d max=%d t=%dms",
             name, tol, max, time_ms);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE, tol, msg);
}

void param_store_notify_gantry_align(char axis_letter, int tol, int timeout_ms)
{
    /* align tol 由 apply 侧 ppu 折算派生, 不入档案 — 仅审计 */
    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "gantry_align %c: tol=%d t=%dms (not persisted)",
             (char)toupper((unsigned char)axis_letter), tol, timeout_ms);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE, tol, msg);
}

void param_store_notify_planner(double tol, double acc)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    if (g_profile.corner_tolerance == tol && g_profile.max_centripetal_acc == acc) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    double old_t = g_profile.corner_tolerance;
    g_profile.corner_tolerance = tol;
    g_profile.max_centripetal_acc = acc;
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "planner: tol %.4f->%.4f acc=%.1f", old_t, tol, acc);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE,
                     (int32_t)(tol * 10000.0), msg);
}

void param_store_notify_kinematics(double tool, double px, double py, double pz)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    if (g_profile.tool_len == tool && g_profile.pivot_x == px
        && g_profile.pivot_y == py && g_profile.pivot_z == pz) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    double old_tool = g_profile.tool_len;
    g_profile.tool_len = tool;
    g_profile.pivot_x = px; g_profile.pivot_y = py; g_profile.pivot_z = pz;
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "kinematics: tool %.1f->%.1f pivot=(%.1f,%.1f,%.1f)",
             old_tool, tool, px, py, pz);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE,
                     (int32_t)(tool * 100.0), msg);
}

void param_store_notify_safelift(double safe_z, double speed, int auto_on)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    if (g_profile.safe_z_mm == safe_z && g_profile.lift_speed_mm_s == speed
        && g_profile.auto_on_alarm == auto_on) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    double old_z = g_profile.safe_z_mm;
    g_profile.safe_z_mm = safe_z;
    g_profile.lift_speed_mm_s = speed;
    g_profile.auto_on_alarm = auto_on;
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "safelift: z %.1f->%.1f v=%.1f auto=%d",
             old_z, safe_z, speed, auto_on);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE,
                     (int32_t)(safe_z * 100.0), msg);
}

void param_store_notify_homing(char axis_letter, int method, int timeout, int dir)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    int ai = ps_find_axis(&g_profile, axis_letter);
    if (ai < 0) { pthread_mutex_unlock(&g_ps_mutex); return; }
    ProfileAxis_t *ax = &g_profile.ax[ai];
    if (ax->hm_enable == 1 && ax->hm_method == method
        && ax->hm_timeout_ms == timeout && ax->hm_direction == dir) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    ax->hm_enable = 1; ax->hm_method = method;
    ax->hm_timeout_ms = timeout; ax->hm_direction = dir;
    char name[8];
    strncpy(name, g_profile.axis_order[ai], sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "homing %s: m=%d t=%dms dir=%d", name, method, timeout, dir);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE, method, msg);
}

void param_store_notify_homing_all(const char *order, int auto_on_init)
{
    pthread_mutex_lock(&g_ps_mutex);
    if (!g_profile.loaded) { pthread_mutex_unlock(&g_ps_mutex); return; }
    if (strcmp(g_profile.homing_order, order) == 0
        && g_profile.auto_on_init == auto_on_init) {
        pthread_mutex_unlock(&g_ps_mutex);
        return;
    }
    char old_order[16];
    strncpy(old_order, g_profile.homing_order, sizeof(old_order) - 1);
    old_order[sizeof(old_order) - 1] = '\0';
    strncpy(g_profile.homing_order, order, sizeof(g_profile.homing_order) - 1);
    g_profile.homing_order[sizeof(g_profile.homing_order) - 1] = '\0';
    g_profile.auto_on_init = auto_on_init;
    ps_commit_change();
    pthread_mutex_unlock(&g_ps_mutex);

    char msg[SMC_EVENT_MSG_LEN];
    snprintf(msg, sizeof(msg), "homing_order: %s -> %s (auto=%d)",
             old_order, order, auto_on_init);
    EventLogger_Push(SEVERITY_INFO, SOURCE_CONFIG, PS_EV_CHANGE, auto_on_init, msg);
}
