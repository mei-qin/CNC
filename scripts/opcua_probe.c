/* =====================================================================
 *  opcua_probe.c  ——  OPC UA Server 冒烟测试客户端 (WSL2 内运行)
 *
 *  用途:
 *    验证 rpc/opcua_server.c 暴露的地址空间与 movecontrol 期望的契约
 *    (doc/CNC_OPCUA_地址空间契约.md) 一致。测试序列完全模拟 movecontrol
 *    P2/P3 客户端行为: connect (None/匿名) → Browse 轴发现 → Read 契约
 *    节点 → Write 倍率 → CallMethod (Load/Start/Pause/Resume/Stop/Reset)。
 *
 *  编译: make probe   (纯客户端, 链接 third_party/open62541, 不依赖内核/SOEM)
 *  运行: sudo ./rpc_server sim &   然后任意目录 ./opcua_probe [url]
 *  退出码: 0=全部 PASS, 1=存在 FAIL
 * ===================================================================== */

#include "open62541.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/stat.h>

#define PROBE_URL_DEFAULT  "opc.tcp://127.0.0.1:4840"
#define PROBE_NC_PATH      "/tmp/opcua_probe_test.nc"
/* CNC 端程序根目录 (与 rpc/opcua_server.c 对齐): SMC_PROGRAM_DIR 环境变量
 * 优先, 默认开发机路径。main() 开头初始化 (实机用户名可能不同, 如 server) */
#define PROBE_NC_DIR_DEFAULT "/home/server/nc"
/* CiA402 状态字 bit3 = 故障 (与 CNC Core 的 SW_ERROR 同值; 探针不依赖内核头文件) */
#define SW_ERROR_PROBE 0x0008
static char PROBE_NC_SHARED[512];
static char PROBE_NC_DIR[448];

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *name, const char *detail)
{
    if (cond) {
        g_pass++;
        printf("  [PASS] %-42s %s\n", name, detail ? detail : "");
    } else {
        g_fail++;
        printf("  [FAIL] %-42s %s\n", name, detail ? detail : "");
    }
}

static UA_StatusCode read_variant(UA_Client *c, const char *id, UA_Variant *v)
{
    UA_NodeId n = UA_NODEID_STRING(1, (char *)id);
    return UA_Client_readValueAttribute(c, n, v);
}

static int read_double(UA_Client *c, const char *id, double *out)
{
    UA_Variant v;
    UA_Variant_init(&v);
    UA_StatusCode sc = read_variant(c, id, &v);
    int ok = (sc == UA_STATUSCODE_GOOD) && UA_Variant_hasScalarType(&v, &UA_TYPES[UA_TYPES_DOUBLE]);
    if (ok) *out = *(double *)v.data;
    UA_Variant_clear(&v);
    return ok ? 0 : -1;
}

static int read_int(UA_Client *c, const char *id, int32_t *out)
{
    UA_Variant v;
    UA_Variant_init(&v);
    UA_StatusCode sc = read_variant(c, id, &v);
    int ok = (sc == UA_STATUSCODE_GOOD) && UA_Variant_hasScalarType(&v, &UA_TYPES[UA_TYPES_INT32]);
    if (ok) *out = *(int32_t *)v.data;
    UA_Variant_clear(&v);
    return ok ? 0 : -1;
}

static int read_string(UA_Client *c, const char *id, char *out, size_t outsz)
{
    UA_Variant v;
    UA_Variant_init(&v);
    UA_StatusCode sc = read_variant(c, id, &v);
    int ok = (sc == UA_STATUSCODE_GOOD) && UA_Variant_hasScalarType(&v, &UA_TYPES[UA_TYPES_STRING]);
    if (ok) {
        UA_String *s = (UA_String *)v.data;
        size_t len = (s->length < outsz - 1) ? s->length : outsz - 1;
        memcpy(out, s->data, len);
        out[len] = '\0';
    } else {
        out[0] = '\0';
    }
    UA_Variant_clear(&v);
    return ok ? 0 : -1;
}

/* 伺服状态节点 (2026-11-27): UInt16 状态字 (Boolean 用下方既有的 read_bool) */
static int read_u16(UA_Client *c, const char *id, uint16_t *out)
{
    UA_Variant v;
    UA_Variant_init(&v);
    UA_StatusCode sc = read_variant(c, id, &v);
    int ok = (sc == UA_STATUSCODE_GOOD) && UA_Variant_hasScalarType(&v, &UA_TYPES[UA_TYPES_UINT16]);
    if (ok) *out = *(UA_UInt16 *)v.data;
    UA_Variant_clear(&v);
    return ok ? 0 : -1;
}

static int call_bool_method(UA_Client *c, const char *obj, const char *meth)
{
    UA_NodeId o = UA_NODEID_STRING(1, (char *)obj);
    UA_NodeId m = UA_NODEID_STRING(1, (char *)meth);
    size_t out_sz = 0;
    UA_Variant *out = NULL;
    UA_StatusCode sc = UA_Client_call(c, o, m, 0, NULL, &out_sz, &out);
    int ok = 0;
    if (sc == UA_STATUSCODE_GOOD && out_sz > 0 &&
        UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
        ok = (*(UA_Boolean *)out[0].data) != 0;
    if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
    return ok ? 0 : -1;
}

/* 协议层方法调用: 返回 0=调用成功且拿到 Boolean 出参 (业务值写 *out_val),
 * -1=调用失败/出参类型不对。业务成功与否由调用方按语义断言。 */
static int call_method_bool_out(UA_Client *c, const char *obj, const char *meth,
                                int *out_val)
{
    UA_NodeId o = UA_NODEID_STRING(1, (char *)obj);
    UA_NodeId m = UA_NODEID_STRING(1, (char *)meth);
    size_t out_sz = 0;
    UA_Variant *out = NULL;
    UA_StatusCode sc = UA_Client_call(c, o, m, 0, NULL, &out_sz, &out);
    int ok = 0;
    if (sc == UA_STATUSCODE_GOOD && out_sz > 0 &&
        UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN])) {
        *out_val = (*(UA_Boolean *)out[0].data) != 0;
        ok = 1;
    }
    if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
    return ok ? 0 : -1;
}

static void write_probe_gcode(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return;
    /* 极简直线程序: load/preview 与 run 都能走通 */
    fprintf(f, "G90\n");
    fprintf(f, "G0 X5\n");
    fprintf(f, "G1 X10 F600\n");
    fprintf(f, "M30\n");
    fclose(f);
}

/* 配置方法调用 (契约 §10): 返回协议层 0=拿到 Int32 出参 (业务码写 *out_rc),
 * -1=传输失败/出参类型不对 */
static int call_method_int32(UA_Client *c, const char *obj, const char *meth,
                             size_t n_in, UA_Variant *in, int32_t *out_rc)
{
    UA_NodeId o = UA_NODEID_STRING(1, (char *)obj);
    UA_NodeId m = UA_NODEID_STRING(1, (char *)meth);
    size_t out_sz = 0;
    UA_Variant *out = NULL;
    UA_StatusCode sc = UA_Client_call(c, o, m, n_in, in, &out_sz, &out);
    int ok = 0;
    if (sc == UA_STATUSCODE_GOOD && out_sz > 0 &&
        UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_INT32])) {
        *out_rc = *(int32_t *)out[0].data;
        ok = 1;
    }
    if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
    return ok ? 0 : -1;
}

/* §11 Laser: 带输入参 + Boolean 出参的方法调用。
 * 返回 0=调用成功且拿到 Boolean 出参 (业务值写 *out_val), -1=传输/类型失败 */
static int call_method_bool_args(UA_Client *c, const char *obj, const char *meth,
                                 size_t n_in, UA_Variant *in, int *out_val)
{
    UA_NodeId o = UA_NODEID_STRING(1, (char *)obj);
    UA_NodeId m = UA_NODEID_STRING(1, (char *)meth);
    size_t out_sz = 0;
    UA_Variant *out = NULL;
    UA_StatusCode sc = UA_Client_call(c, o, m, n_in, in, &out_sz, &out);
    int ok = 0;
    if (sc == UA_STATUSCODE_GOOD && out_sz > 0 &&
        UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN])) {
        *out_val = (*(UA_Boolean *)out[0].data) != 0;
        ok = 1;
    }
    if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
    return ok ? 0 : -1;
}

static UA_StatusCode write_bool(UA_Client *c, const char *id, UA_Boolean v)
{
    UA_Variant w;
    UA_Variant_init(&w);
    UA_Variant_setScalarCopy(&w, &v, &UA_TYPES[UA_TYPES_BOOLEAN]);
    UA_NodeId n = UA_NODEID_STRING(1, (char *)id);
    UA_StatusCode sc = UA_Client_writeValueAttribute(c, n, &w);
    UA_Variant_clear(&w);
    return sc;
}

static int read_bool(UA_Client *c, const char *id, UA_Boolean *out)
{
    UA_Variant v;
    UA_Variant_init(&v);
    UA_StatusCode sc = read_variant(c, id, &v);
    int ok = (sc == UA_STATUSCODE_GOOD) &&
             UA_Variant_hasScalarType(&v, &UA_TYPES[UA_TYPES_BOOLEAN]);
    if (ok) *out = *(UA_Boolean *)v.data;
    UA_Variant_clear(&v);
    return ok ? 0 : -1;
}

int main(int argc, char *argv[])
{
    const char *url = (argc > 1) ? argv[1] : PROBE_URL_DEFAULT;
    char buf[256];

    /* 程序根目录: 与 server 端 SMC_PROGRAM_DIR 对齐 (实机用户名差异适配) */
    {
        const char *env = getenv("SMC_PROGRAM_DIR");
        snprintf(PROBE_NC_DIR, sizeof(PROBE_NC_DIR), "%s",
                 (env && env[0]) ? env : PROBE_NC_DIR_DEFAULT);
        snprintf(PROBE_NC_SHARED, sizeof(PROBE_NC_SHARED),
                 "%s/opcua_probe_test.nc", PROBE_NC_DIR);
    }

    printf("=== OPC UA probe: %s ===\n", url);

    UA_Client *client = UA_Client_new();
    UA_ClientConfig_setDefault(UA_Client_getConfig(client));

    /* ---- 1. 连接 (SecurityPolicy None + 匿名, 与 movecontrol 一致) ---- */
    UA_StatusCode sc = UA_Client_connect(client, url);
    if (sc != UA_STATUSCODE_GOOD) {
        printf("  [FAIL] connect (%s)\n", UA_StatusCode_name(sc));
        UA_Client_delete(client);
        return 1;
    }
    printf("  [PASS] connect\n");
    g_pass++;

    /* ---- 2. Browse 轴发现 (movecontrol ScanAxesAndSubscribe 同款逻辑) ---- */
    {
        UA_BrowseRequest bReq;
        UA_BrowseRequest_init(&bReq);
        bReq.requestedMaxReferencesPerNode = 0;
        bReq.nodesToBrowseSize = 1;
        bReq.nodesToBrowse = (UA_BrowseDescription *)UA_Array_new(1, &UA_TYPES[UA_TYPES_BROWSEDESCRIPTION]);
        UA_BrowseDescription_init(&bReq.nodesToBrowse[0]);
        bReq.nodesToBrowse[0].nodeId = UA_NODEID_STRING_ALLOC(1, "CNC.Axis");
        bReq.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL;

        UA_BrowseResponse bResp = UA_Client_Service_browse(client, bReq);
        int axes_found = 0;
        if (bResp.responseHeader.serviceResult == UA_STATUSCODE_GOOD &&
            bResp.resultsSize == 1) {
            for (size_t i = 0; i < bResp.results[0].referencesSize; i++) {
                UA_ReferenceDescription *ref = &bResp.results[0].references[i];
                if (ref->nodeClass != UA_NODECLASS_OBJECT) continue;
                char name[32] = {0};
                UA_String *bn = &ref->browseName.name;
                size_t len = (bn->length < sizeof(name) - 1) ? bn->length : sizeof(name) - 1;
                if (bn->length == 0) continue;
                memcpy(name, bn->data, len);
                printf("        axis discovered: %s\n", name);
                axes_found++;
            }
        }
        UA_BrowseRequest_clear(&bReq);
        UA_BrowseResponse_clear(&bResp);
        snprintf(buf, sizeof(buf), "%d axes", axes_found);
        check(axes_found >= 3, "Browse CNC.Axis objects", buf);
    }

    /* ---- 3. Read 契约节点 ---- */
    {
        int32_t status = -1, mode = -1, cur_line = -1, line_cnt = -1, alarm_cnt = -1;
        double speed = -1, fovr = -1, rovr = -1, sovr = -1;
        char name[128] = {0}, dt[64] = {0}, span[64] = {0};

        read_int   (client, "CNC.Machine.Status",      &status);
        read_int   (client, "CNC.Machine.Mode",        &mode);
        read_string(client, "CNC.Machine.DateTime",    dt,   sizeof(dt));
        read_string(client, "CNC.Machine.PowerOnSpan", span, sizeof(span));
        read_string(client, "CNC.Program.Name",        name, sizeof(name));
        read_int   (client, "CNC.Program.CurrentLine", &cur_line);
        read_int   (client, "CNC.Program.LineCount",   &line_cnt);
        read_int   (client, "CNC.Alarm.Count",         &alarm_cnt);
        /* 速度/倍率节点 2026-11-26 自 CNC 根迁入 CNC.Machine (契约 §13-1),
         * 单位改 mm/min 与 % (契约 §13-2, 与上位机 ICncAxis 同单位) */
        read_double(client, "CNC.Machine.ActualSpeed",    &speed);
        read_double(client, "CNC.Machine.FeedOverride",   &fovr);
        read_double(client, "CNC.Machine.RapidOverride",  &rovr);
        read_double(client, "CNC.Machine.SpindleOverride",&sovr);

        printf("        Status=%d Mode=%d FeedOvr=%.1f%% RapidOvr=%.1f%% SpindleOvr=%.1f%%\n",
               status, mode, fovr, rovr, sovr);
        printf("        DateTime=%s PowerOn=%s Program=%s Line=%d/%d Alarm=%d\n",
               dt, span, name, cur_line, line_cnt, alarm_cnt);

        check(status >= 0 && status <= 5, "Read CNC.Machine.Status (0..5)", NULL);
        check(mode == 0 || mode == 1,      "Read CNC.Machine.Mode (0/1)", NULL);
        check(dt[0] == '2',                "Read CNC.Machine.DateTime (ISO8601)", dt);
        check(strchr(span, 'h') != NULL,   "Read CNC.Machine.PowerOnSpan (NhNm)", span);
        check(fovr >= 0.0 && fovr <= 100.0, "Read CNC.Machine.FeedOverride (%, 0~100)", NULL);
        check(rovr >= 0.0 && rovr <= 100.0, "Read CNC.Machine.RapidOverride (%, 0~100)", NULL);
        check(sovr >= 0.0 && sovr <= 120.0, "Read CNC.Machine.SpindleOverride (%, 0~120)", NULL);
        check(speed >= 0.0,                 "Read CNC.Machine.ActualSpeed (mm/min)", NULL);
        check(alarm_cnt >= 0,               "Read CNC.Alarm.Count", NULL);

        /* 旧路径 (CNC 根下) 必须不存在, 防止回退成与服务端、客户端不一致的双路径 */
        UA_Variant vold;
        UA_Variant_init(&vold);
        UA_StatusCode sco = read_variant(client, "CNC.FeedOverride", &vold);
        UA_Variant_clear(&vold);
        check(sco == UA_STATUSCODE_BADNODEIDUNKNOWN,
              "Old path CNC.FeedOverride removed", UA_StatusCode_name(sco));
    }

    /* ---- 3b. 轴节点读 (首轴 + 硬编码 X 验证) ---- */
    {
        double pos = -999999.0, prog = -999999.0, remain = 0.0;
        UA_Boolean active = 0;
        char id[128];
        int ok_pos  = read_double(client, "CNC.Axis.X.ActualPosition", &pos) == 0;
        int ok_prog = read_double(client, "CNC.Axis.X.ProgramPosition", &prog) == 0;
        int ok_rem  = read_double(client, "CNC.Axis.X.RemainingDistance", &remain) == 0;

        UA_Variant v;
        UA_Variant_init(&v);
        int ok_act = 0;
        snprintf(id, sizeof(id), "CNC.Axis.X.Active");
        if (read_variant(client, id, &v) == UA_STATUSCODE_GOOD &&
            UA_Variant_hasScalarType(&v, &UA_TYPES[UA_TYPES_BOOLEAN])) {
            active = *(UA_Boolean *)v.data;
            ok_act = 1;
        }
        UA_Variant_clear(&v);

        printf("        X: actual=%.4f prog=%.4f remain=%.4f active=%d\n",
               pos, prog, remain, (int)active);
        check(ok_pos,  "Read CNC.Axis.X.ActualPosition (Double mm)", NULL);
        check(ok_prog, "Read CNC.Axis.X.ProgramPosition (Double mm)", NULL);
        check(ok_rem,  "Read CNC.Axis.X.RemainingDistance (Double)", NULL);
        check(ok_act,  "Read CNC.Axis.X.Active (Boolean)", NULL);

        /* ---- 伺服状态 (2026-11-27, 契约 §24/§25)。
         *     默认模式: 9527 开始监听时各轴已到终态, sim 无故障注入 → 全轴 Operation enabled。
         *     故障场景 (环境变量 OPCUA_PROBE_SERVO_FAULT=1, 服务端以 SIM_BOOT_FAULT=Z 启动):
         *       Z 轴故障锁存 → Z.Active=false / ServoOk=false / 报警列表有 Z 轴文字, 其余轴正常 ---- */
        {
            int servo_fault_mode = (getenv("OPCUA_PROBE_SERVO_FAULT") != NULL);
            UA_Boolean servo_ok = 0, z_active = 1;
            uint16_t sw = 0, z_sw = 0;
            int32_t alarm_code = -12345, alarm_cnt2 = -1;
            int ok_sw   = read_u16(client, "CNC.Axis.X.StatusWord", &sw) == 0;
            int ok_code = read_int(client, "CNC.Axis.X.DriveAlarmCode", &alarm_code) == 0;
            int ok_serv = read_bool(client, "CNC.Safety.ServoOk", &servo_ok) == 0;
            int ok_zsw  = read_u16(client, "CNC.Axis.Z.StatusWord", &z_sw) == 0;
            int ok_zact = read_bool(client, "CNC.Axis.Z.Active", &z_active) == 0;
            read_int(client, "CNC.Alarm.Count", &alarm_cnt2);
            printf("        X: StatusWord=0x%04X DriveAlarmCode=%d ServoOk=%d | "
                   "Z: Active=%d StatusWord=0x%04X Alarm=%d\n",
                   sw, alarm_code, (int)servo_ok, (int)z_active, z_sw, alarm_cnt2);
            check(ok_act && active == 1,        "CNC.Axis.X.Active == true (enabled)", NULL);
            check(ok_sw,                         "Read CNC.Axis.X.StatusWord (UInt16)", NULL);
            check(ok_sw && (sw & 0x006F) == 0x0027,
                  "CNC.Axis.X.StatusWord == Operation enabled", NULL);
            check(ok_code,                       "Read CNC.Axis.X.DriveAlarmCode (Int32)", NULL);
            check(ok_code && alarm_code == 0,    "CNC.Axis.X.DriveAlarmCode == 0", NULL);
            check(ok_serv,                       "Read CNC.Safety.ServoOk (Boolean)", NULL);
            check(ok_serv && servo_ok == !servo_fault_mode,
                  servo_fault_mode ? "CNC.Safety.ServoOk == false (Z latched)"
                                   : "CNC.Safety.ServoOk == true", NULL);
            check(ok_zact && z_active == !servo_fault_mode,
                  servo_fault_mode ? "CNC.Axis.Z.Active == false (latched)"
                                   : "CNC.Axis.Z.Active == true", NULL);
            if (!servo_fault_mode) {
                check(ok_zsw && (z_sw & 0x006F) == 0x0027,
                      "CNC.Axis.Z.StatusWord == Operation enabled", NULL);
            } else {
                /* sim 驱动器 Fault 态状态字 = SW_ERROR (0x0008); 实机各驱动器 Fault 态位定义以其手册为准 */
                check(ok_zsw && (z_sw & SW_ERROR_PROBE) != 0,
                      "CNC.Axis.Z.StatusWord has fault bit", NULL);
                check(alarm_cnt2 >= 1, "CNC.Alarm.Count >= 1 (servo fault)", NULL);
                {
                    UA_Variant lv;
                    UA_Variant_init(&lv);
                    int has_text = 0;
                    if (read_variant(client, "CNC.Alarm.List", &lv) == UA_STATUSCODE_GOOD &&
                        UA_Variant_hasArrayType(&lv, &UA_TYPES[UA_TYPES_STRING])) {
                        UA_String *arr = (UA_String *)lv.data;
                        for (size_t k = 0; k < lv.arrayLength; k++) {
                            char row[160] = {0};
                            size_t len = (arr[k].length < sizeof(row) - 1) ? arr[k].length : sizeof(row) - 1;
                            memcpy(row, arr[k].data, len);
                            if (strstr(row, "Z servo fault") || strstr(row, "servo fault latched"))
                                has_text = 1;
                        }
                    }
                    UA_Variant_clear(&lv);
                    check(has_text, "CNC.Alarm.List has 'Z servo fault' text", NULL);
                }
            }
            if (servo_fault_mode) {
                /* 故障场景只核对伺服状态面: 程序/寸动等后续段落在此模式下按设计应被拒绝 */
                printf("=== RESULT: %d PASS, %d FAIL === (servo fault scenario)\n", g_pass, g_fail);
                UA_Client_disconnect(client);
                UA_Client_delete(client);
                return (g_fail == 0) ? 0 : 1;
            }
        }
    }

    /* ---- 3c. Alarm.List 数组节点 (可为空数组, 但类型必须是 String[]) ---- */
    {
        UA_Variant v;
        UA_Variant_init(&v);
        UA_StatusCode sc2 = read_variant(client, "CNC.Alarm.List", &v);
        /* 有报警: String[]; 无报警: 全空 variant (data=NULL, movecontrol
         * ReadNodeStringArray 同样按空列表处理) */
        int ok = (sc2 == UA_STATUSCODE_GOOD) &&
                 (v.data == NULL ||
                  v.type == &UA_TYPES[UA_TYPES_STRING]);
        if (ok) {
            printf("        Alarm.List: %zu entries\n", v.arrayLength);
            for (size_t i = 0; i < v.arrayLength && i < 3; i++) {
                UA_String *s = &((UA_String *)v.data)[i];
                printf("          [%zu] %.*s\n", i, (int)s->length, (char *)s->data);
            }
        } else {
            printf("        [diag] Alarm.List sc=%s type=%p arrayLen=%zu\n",
                   UA_StatusCode_name(sc2), (void *)v.type, v.arrayLength);
        }
        UA_Variant_clear(&v);
        check(ok, "Read CNC.Alarm.List (String[])", NULL);
    }

    /* ---- 4. Write FeedOverride (80%) + 回读 (契约单位 %) ---- */
    {
        UA_Variant w;
        UA_Double val = 80.0;
        UA_Variant_setScalarCopy(&w, &val, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_NodeId n = UA_NODEID_STRING(1, (char *)"CNC.Machine.FeedOverride");
        UA_StatusCode sc3 = UA_Client_writeValueAttribute(client, n, &w);
        UA_Variant_clear(&w);

        double back = -1.0;
        /* 写生效需穿越 RT 周期 + snapshot 重发, 留足时间再回读 */
        for (int i = 0; i < 10 && (back < 79.0 || back > 81.0); i++) {
            usleep(100000);
            read_double(client, "CNC.Machine.FeedOverride", &back);
        }
        snprintf(buf, sizeof(buf), "write=%s readback=%.1f%%",
                 UA_StatusCode_name(sc3), back);
        check(sc3 == UA_STATUSCODE_GOOD && back > 79.0 && back < 81.0,
              "Write CNC.Machine.FeedOverride(80%) + readback", buf);
    }

    /* ---- 5. 方法链路: Load → Start → Pause → Resume → Stop ---- */
    {
        /* 5a-0. 准备两份相同程序: /tmp (Linux 绝对路径用) + 程序根目录
         * (模拟 SMB 共享写入, 供 Windows 形式路径 basename 回退命中) */
        mkdir(PROBE_NC_DIR, 0755);   /* 已存在 no-op */
        write_probe_gcode(PROBE_NC_PATH);
        write_probe_gcode(PROBE_NC_SHARED);

        /* 5a. Program.Load(String): Linux 绝对路径 (直连) */
        UA_NodeId o = UA_NODEID_STRING(1, (char *)"CNC.Program");
        UA_NodeId m = UA_NODEID_STRING(1, (char *)"CNC.Program.Load");
        UA_Variant in[1];
        UA_String path = UA_STRING(PROBE_NC_PATH);
        UA_Variant_setScalarCopy(&in[0], &path, &UA_TYPES[UA_TYPES_STRING]);
        size_t out_sz = 0;
        UA_Variant *out = NULL;
        UA_StatusCode sc4 = UA_Client_call(client, o, m, 1, in, &out_sz, &out);
        int load_ok = 0;
        if (sc4 == UA_STATUSCODE_GOOD && out_sz > 0 &&
            UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
            load_ok = (*(UA_Boolean *)out[0].data) != 0;
        if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
        UA_Variant_clear(&in[0]);
        check(load_ok, "Call CNC.Program.Load(linux path)", NULL);

        /* 5a-2. Program.Load(Windows 形式路径): basename 回退到程序根目录
         * (Syntec 生态对等: 上位经共享写文件, 传 Windows 路径也能命中) */
        {
            /* 等 parser 空闲 (上一次 Load 解析完成) */
            for (int i = 0; i < 100; i++) {
                int32_t lc = 1;
                read_int(client, "CNC.Program.LineCount", &lc);
                if (lc > 0) break;
                usleep(100000);
            }
            usleep(200000);
            UA_String wpath = UA_STRING((char *)"\\\\LINUX-PC\\cnc-programs\\opcua_probe_test.nc");
            UA_Variant_setScalarCopy(&in[0], &wpath, &UA_TYPES[UA_TYPES_STRING]);
            out_sz = 0; out = NULL;
            sc4 = UA_Client_call(client, o, m, 1, in, &out_sz, &out);
            int load2_ok = 0;
            if (sc4 == UA_STATUSCODE_GOOD && out_sz > 0 &&
                UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
                load2_ok = (*(UA_Boolean *)out[0].data) != 0;
            if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
            UA_Variant_clear(&in[0]);
            check(load2_ok, "Program.Load(windows path) -> shared dir", NULL);
        }

        /* Load 是异步解析, 等它完成再 Start (server 端 SMC_LoadProgram 语义) */
        printf("        waiting preview parse...\n");
        int32_t line_cnt = 0;
        for (int i = 0; i < 100; i++) {
            usleep(100000);
            read_int(client, "CNC.Program.LineCount", &line_cnt);
            if (line_cnt > 0) break;
        }
        snprintf(buf, sizeof(buf), "LineCount=%d", line_cnt);
        check(line_cnt > 0, "Read CNC.Program.LineCount after Load", buf);

        check(call_bool_method(client, "CNC.Program", "CNC.Program.Start")  == 0,
              "Call CNC.Program.Start -> bool", NULL);
        /* 运行期单位检查 (契约 §13-2): 程序为 G0 X5 → G1 X10 F600, 倍率 80%
         * ⇒ G1 进给 ≈ 480 mm/min, G0 快移至 X 轴最大速度。采样约 300ms 取最大值:
         * mm/min 应 ≥ 300; 若节点仍输出 mm/s 或 m/s 则 ≤ 60。上界取 X 轴
         * 最大速度 (mm/s) × 60 的 1.1 倍, 与具体机型无关。 */
        {
            double spd_max = 0.0, x_max = 0.0;
            read_double(client, "CNC.Config.Axis.X.MaxSpeed", &x_max);
            for (int i = 0; i < 150; i++) {
                double s = 0.0;
                if (read_double(client, "CNC.Machine.ActualSpeed", &s) == 0 && s > spd_max)
                    spd_max = s;
                usleep(2000);
            }
            double upper = (x_max > 0.0) ? x_max * 60.0 * 1.1 : 1e9;
            snprintf(buf, sizeof(buf), "max=%.1f mm/min (X.MaxSpeed=%.1f mm/s)",
                     spd_max, x_max);
            check(spd_max >= 300.0 && spd_max <= upper,
                  "ActualSpeed unit mm/min during run", buf);
        }
        check(call_bool_method(client, "CNC.Program", "CNC.Program.Pause")  == 0,
              "Call CNC.Program.Pause -> bool", NULL);
        usleep(200000);
        check(call_bool_method(client, "CNC.Program", "CNC.Program.Resume") == 0,
              "Call CNC.Program.Resume -> bool", NULL);
        check(call_bool_method(client, "CNC.Program", "CNC.Program.Stop")   == 0,
              "Call CNC.Program.Stop -> bool", NULL);

        /* ---- 6. System.Reset ---- */
        check(call_bool_method(client, "CNC.System", "CNC.System.Reset") == 0,
              "Call CNC.System.Reset -> bool", NULL);
    }

    /* ---- 7. EStop → Reset 恢复闭环 (安全链路) ---- */
    {
        check(call_bool_method(client, "CNC.System", "CNC.System.EStop") == 0,
              "Call CNC.System.EStop -> true", NULL);

        int32_t st = 0;
        /* 轮询等待快照同步 (EStop 原子置位 → 下个 RT 周期 snapshot 才带上) */
        for (int i = 0; i < 20 && st != 3; i++) {
            usleep(50000);
            read_int(client, "CNC.Machine.Status", &st);
        }
        snprintf(buf, sizeof(buf), "Status=%d (expect 3=ALARM)", st);
        check(st == 3, "EStop -> Machine.Status ALARM", buf);

        check(call_bool_method(client, "CNC.System", "CNC.System.Reset") == 0,
              "EStop recovered by System.Reset", NULL);
    }

    /* ---- 8. Jog.Move(String axis, Int32 dir, Double speed) 回归 ----
     * (置于 Home 之前: 异步回零会按互斥抢占 JOG 状态机, 两者需隔离) */
    {
        UA_NodeId o = UA_NODEID_STRING(1, (char *)"CNC.Jog");
        UA_NodeId m = UA_NODEID_STRING(1, (char *)"CNC.Jog.Move");
        UA_Variant in[3];
        UA_String axis = UA_STRING((char *)"X");
        UA_Int32 dir = 1;
        UA_Double speed = 5.0;
        UA_Variant_setScalarCopy(&in[0], &axis, &UA_TYPES[UA_TYPES_STRING]);
        UA_Variant_setScalarCopy(&in[1], &dir, &UA_TYPES[UA_TYPES_INT32]);
        UA_Variant_setScalarCopy(&in[2], &speed, &UA_TYPES[UA_TYPES_DOUBLE]);
        size_t out_sz = 0;
        UA_Variant *out = NULL;
        UA_StatusCode sc6 = UA_Client_call(client, o, m, 3, in, &out_sz, &out);
        UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]); UA_Variant_clear(&in[2]);
        int jok = -1;
        if (sc6 == UA_STATUSCODE_GOOD && out_sz > 0 &&
            UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
            jok = (*(UA_Boolean *)out[0].data) != 0;
        if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
        snprintf(buf, sizeof(buf), "ret=%d", jok);
        check(jok == 1, "Call CNC.Jog.Move(start X+) -> true", buf);

        /* 300ms 后停止 (dir=0) */
        usleep(300000);
        dir = 0;
        UA_Variant_setScalarCopy(&in[0], &axis, &UA_TYPES[UA_TYPES_STRING]);
        UA_Variant_setScalarCopy(&in[1], &dir, &UA_TYPES[UA_TYPES_INT32]);
        UA_Variant_setScalarCopy(&in[2], &speed, &UA_TYPES[UA_TYPES_DOUBLE]);
        out_sz = 0; out = NULL;
        sc6 = UA_Client_call(client, o, m, 3, in, &out_sz, &out);
        UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]); UA_Variant_clear(&in[2]);
        int jstop = -1;
        if (sc6 == UA_STATUSCODE_GOOD && out_sz > 0 &&
            UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
            jstop = (*(UA_Boolean *)out[0].data) != 0;
        if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
        snprintf(buf, sizeof(buf), "ret=%d", jstop);
        check(jstop == 1, "Call CNC.Jog.Move(dir=0 stop) -> true", buf);

        double px = -1e9;
        read_double(client, "CNC.Axis.X.ActualPosition", &px);
        snprintf(buf, sizeof(buf), "X=%.4f (moved from 0)", px);
        check(px > 0.0, "Jog moved X axis (+direction)", buf);
    }

    /* ---- 8b. Jog.MoveInc(String, Int32, Double, Double) 增量寸动回归 ----
     * (依赖 8 的 JOG 已停; 增量段 2mm@5mm/s, 验证精确到位 + 基准=当前实际位置) */
    {
        UA_NodeId o = UA_NODEID_STRING(1, (char *)"CNC.Jog");
        UA_NodeId m = UA_NODEID_STRING(1, (char *)"CNC.Jog.MoveInc");

        double x0 = -1e9;
        read_double(client, "CNC.Axis.X.ActualPosition", &x0);

        UA_Variant in[4];
        UA_String axis = UA_STRING((char *)"X");
        UA_Int32 dir = 1;
        UA_Double dist = 2.0;
        UA_Double speed = 5.0;
        UA_Variant_setScalarCopy(&in[0], &axis, &UA_TYPES[UA_TYPES_STRING]);
        UA_Variant_setScalarCopy(&in[1], &dir, &UA_TYPES[UA_TYPES_INT32]);
        UA_Variant_setScalarCopy(&in[2], &dist, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Variant_setScalarCopy(&in[3], &speed, &UA_TYPES[UA_TYPES_DOUBLE]);
        size_t out_sz = 0;
        UA_Variant *out = NULL;
        UA_StatusCode sc = UA_Client_call(client, o, m, 4, in, &out_sz, &out);
        UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
        UA_Variant_clear(&in[2]); UA_Variant_clear(&in[3]);
        int iok = -1;
        if (sc == UA_STATUSCODE_GOOD && out_sz > 0 &&
            UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
            iok = (*(UA_Boolean *)out[0].data) != 0;
        if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
        snprintf(buf, sizeof(buf), "ret=%d x0=%.4f", iok, x0);
        check(iok == 1, "Call CNC.Jog.MoveInc(X +2mm) -> true", buf);

        /* 等段消费完成: 2mm@5mm/s ≈ 0.4s + 加减速裕量, 上限 2s */
        const double target = x0 + 2.0;
        double x1 = -1e9;
        for (int i = 0; i < 100; i++) {
            usleep(20000);
            read_double(client, "CNC.Axis.X.ActualPosition", &x1);
            if (fabs(x1 - target) < 0.01) break;
        }
        snprintf(buf, sizeof(buf), "x1=%.4f target=%.4f", x1, target);
        check(fabs(x1 - target) < 0.05, "MoveInc moved X exactly +2mm", buf);
    }

    /* ---- 9. Home: 状态读 + 方法 (协议层; sim 未配 homing 业务值可为 false) ---- */
    {
        /* 诊断: Browse CNC.Home 子节点 */
        {
            UA_BrowseRequest bReq;
            UA_BrowseRequest_init(&bReq);
            bReq.requestedMaxReferencesPerNode = 0;
            bReq.nodesToBrowseSize = 1;
            bReq.nodesToBrowse = (UA_BrowseDescription *)UA_Array_new(1, &UA_TYPES[UA_TYPES_BROWSEDESCRIPTION]);
            UA_BrowseDescription_init(&bReq.nodesToBrowse[0]);
            bReq.nodesToBrowse[0].nodeId = UA_NODEID_STRING_ALLOC(1, "CNC.Home");
            bReq.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL;
            UA_BrowseResponse bResp = UA_Client_Service_browse(client, bReq);
            if (bResp.responseHeader.serviceResult == UA_STATUSCODE_GOOD && bResp.resultsSize == 1) {
                for (size_t i = 0; i < bResp.results[0].referencesSize; i++) {
                    UA_ReferenceDescription *ref = &bResp.results[0].references[i];
                    char idbuf[128] = {0};
                    if (ref->nodeId.nodeId.identifierType == UA_NODEIDTYPE_STRING) {
                        size_t len = ref->nodeId.nodeId.identifier.string.length;
                        if (len > sizeof(idbuf) - 1) len = sizeof(idbuf) - 1;
                        memcpy(idbuf, ref->nodeId.nodeId.identifier.string.data, len);
                    }
                    char nb[32] = {0};
                    size_t bl = ref->browseName.name.length;
                    if (bl > sizeof(nb) - 1) bl = sizeof(nb) - 1;
                    memcpy(nb, ref->browseName.name.data, bl);
                    printf("        Home child: class=%d browse=\"%s\" id=%s\n",
                           (int)ref->nodeClass, nb, idbuf);
                }
            }
            UA_BrowseRequest_clear(&bReq);
            UA_BrowseResponse_clear(&bResp);
        }

        int32_t hstate = -1;
        double hprog = -1.0;
        read_int(client, "CNC.Home.State", &hstate);
        read_double(client, "CNC.Home.Progress", &hprog);
        snprintf(buf, sizeof(buf), "State=%d Progress=%.2f", hstate, hprog);
        check(hstate >= 0 && hstate <= 4 && hprog >= 0.0 && hprog <= 1.0,
              "Read CNC.Home.State/Progress", buf);

        int hv = -1;
        int call_ok = call_method_bool_out(client, "CNC.Home", "CNC.Home.All", &hv);
        snprintf(buf, sizeof(buf), "ret=%d", hv);
        check(call_ok == 0, "Call CNC.Home.All -> bool", buf);

        /* Home.Axis(String): 带参方法 */
        UA_NodeId o = UA_NODEID_STRING(1, (char *)"CNC.Home");
        UA_NodeId m = UA_NODEID_STRING(1, (char *)"CNC.Home.Axis");
        UA_Variant in[1];
        UA_String axis = UA_STRING((char *)"X");
        UA_Variant_setScalarCopy(&in[0], &axis, &UA_TYPES[UA_TYPES_STRING]);
        size_t out_sz = 0;
        UA_Variant *out = NULL;
        UA_StatusCode sc5 = UA_Client_call(client, o, m, 1, in, &out_sz, &out);
        UA_Variant_clear(&in[0]);
        int hv2 = -1;
        if (sc5 == UA_STATUSCODE_GOOD && out_sz > 0 &&
            UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
            hv2 = (*(UA_Boolean *)out[0].data) != 0;
        if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
        snprintf(buf, sizeof(buf), "ret=%d sc=%s", hv2, UA_StatusCode_name(sc5));
        check(hv2 >= 0, "Call CNC.Home.Axis(String) -> bool", buf);

        /* Home.Order(String): 子集串行回零 (旋转回零 "BC") */
        UA_NodeId mo = UA_NODEID_STRING(1, (char *)"CNC.Home.Order");
        UA_String axes = UA_STRING((char *)"BC");
        UA_Variant_setScalarCopy(&in[0], &axes, &UA_TYPES[UA_TYPES_STRING]);
        out_sz = 0; out = NULL;
        UA_StatusCode sc7 = UA_Client_call(client, o, mo, 1, in, &out_sz, &out);
        UA_Variant_clear(&in[0]);
        int hv3 = -1;
        if (sc7 == UA_STATUSCODE_GOOD && out_sz > 0 &&
            UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_BOOLEAN]))
            hv3 = (*(UA_Boolean *)out[0].data) != 0;
        if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
        snprintf(buf, sizeof(buf), "ret=%d sc=%s", hv3, UA_StatusCode_name(sc7));
        check(hv3 >= 0, "Call CNC.Home.Order(\"BC\") -> bool", buf);
    }

    /* ---- 10. CNC.Config 配置面 (契约 §10) ----
     * 序列: 读变量 → 写保护拒收 (-2) → 解锁 → SetAxisDynamics 生效+回读
     * → SetSoftLimit → SetPulsePerUnit (待重启 2 / sim 落盘拒绝 -1)
     * → ProfileDirty → SaveProfile → 恢复写保护。
     * 前置: 等回零 (§9) 离开 RUNNING, 避免 SMC 静止闸门误拒。 */
    {
        /* 10-0. 等待回零静止 (上限 5s; IDLE/DONE/FAULT 均可) */
        int32_t hs = 2;
        for (int i = 0; i < 100 && hs == 2; i++) {
            usleep(50000);
            read_int(client, "CNC.Home.State", &hs);
        }

        /* 10-1. 配置变量读 (X 轴 + 全局) */
        double ms = -1, sl_pos = -1, tool = -1;
        int32_t atype = -1, dirty = -1;
        UA_Boolean wp = 0;
        int ok_ms  = read_double(client, "CNC.Config.Axis.X.MaxSpeed", &ms) == 0;
        int ok_at  = read_int(client, "CNC.Config.Axis.X.AxisType", &atype) == 0;
        int ok_sl  = read_double(client, "CNC.Config.Axis.X.SoftLimitPos", &sl_pos) == 0;
        int ok_tl  = read_double(client, "CNC.Config.ToolLen", &tool) == 0;
        int ok_dy  = read_int(client, "CNC.Config.ProfileDirty", &dirty) == 0;
        int ok_wp  = read_bool(client, "CNC.Config.WriteProtect", &wp) == 0;
        printf("        X: MaxSpeed=%.2f AxisType=%d SoftLimitPos=%.1f "
               "ToolLen=%.1f Dirty=%d WP=%d\n", ms, atype, sl_pos, tool, dirty, (int)wp);
        check(ok_ms,  "Read CNC.Config.Axis.X.MaxSpeed (Double)", NULL);
        check(ok_at,  "Read CNC.Config.Axis.X.AxisType (Int32)", NULL);
        check(ok_sl,  "Read CNC.Config.Axis.X.SoftLimitPos", NULL);
        check(ok_tl,  "Read CNC.Config.ToolLen", NULL);
        check(ok_dy,  "Read CNC.Config.ProfileDirty (Int32)", NULL);
        check(ok_wp && wp, "Read CNC.Config.WriteProtect (default true)", NULL);

        /* 10-2. 写保护拒收: SetPlannerParams → Int32 -2 */
        int32_t rc = 99;
        {
            UA_Variant in[2];
            UA_Double tol = 0.05, acc = 500.0;
            UA_Variant_setScalarCopy(&in[0], &tol, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[1], &acc, &UA_TYPES[UA_TYPES_DOUBLE]);
            int cok = call_method_int32(client, "CNC.Config", "CNC.Config.SetPlannerParams",
                                        2, in, &rc);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
            snprintf(buf, sizeof(buf), "rc=%d (expect -2)", rc);
            check(cok == 0 && rc == -2, "SetPlannerParams write-protected -> -2", buf);
        }

        /* 10-3. 解锁: WriteProtect=false (Boolean 写, ds_write 按 kind 分派) */
        {
            UA_StatusCode scw = write_bool(client, "CNC.Config.WriteProtect", 0);
            UA_Boolean back = 1;
            for (int i = 0; i < 10 && back; i++) {
                usleep(50000);
                read_bool(client, "CNC.Config.WriteProtect", &back);
            }
            snprintf(buf, sizeof(buf), "write=%s readback=%d",
                     UA_StatusCode_name(scw), (int)back);
            check(scw == UA_STATUSCODE_GOOD && !back,
                  "Write CNC.Config.WriteProtect=false", buf);
        }

        /* 10-4. SetAxisDynamics(X, 0, 60, 200, 200, 0) → 0 + 回读 60 */
        {
            UA_Variant in[6];
            UA_String axis = UA_STRING((char *)"X");
            UA_Int32 type = 0;
            UA_Double v = 60.0, a = 200.0, d = 200.0, r = 0.0;
            UA_Variant_setScalarCopy(&in[0], &axis, &UA_TYPES[UA_TYPES_STRING]);
            UA_Variant_setScalarCopy(&in[1], &type, &UA_TYPES[UA_TYPES_INT32]);
            UA_Variant_setScalarCopy(&in[2], &v, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[3], &a, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[4], &d, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[5], &r, &UA_TYPES[UA_TYPES_DOUBLE]);
            int cok = call_method_int32(client, "CNC.Config", "CNC.Config.SetAxisDynamics",
                                        6, in, &rc);
            for (int i = 0; i < 6; i++) UA_Variant_clear(&in[i]);
            snprintf(buf, sizeof(buf), "rc=%d (expect 0)", rc);
            check(cok == 0 && rc == 0, "SetAxisDynamics(X 60mm/s) -> 0", buf);

            double back = -1;
            for (int i = 0; i < 10 && back < 59.0; i++) {
                usleep(50000);
                read_double(client, "CNC.Config.Axis.X.MaxSpeed", &back);
            }
            snprintf(buf, sizeof(buf), "MaxSpeed=%.2f (expect 60)", back);
            check(back > 59.0 && back < 61.0, "Readback MaxSpeed after set", buf);
        }

        /* 10-5. SetSoftLimit(X, 1, -10, 800) → 0 + 回读 800 */
        {
            UA_Variant in[4];
            UA_String axis = UA_STRING((char *)"X");
            UA_Int32 en = 1;
            UA_Double neg = -10.0, pos = 800.0;
            UA_Variant_setScalarCopy(&in[0], &axis, &UA_TYPES[UA_TYPES_STRING]);
            UA_Variant_setScalarCopy(&in[1], &en, &UA_TYPES[UA_TYPES_INT32]);
            UA_Variant_setScalarCopy(&in[2], &neg, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[3], &pos, &UA_TYPES[UA_TYPES_DOUBLE]);
            int cok = call_method_int32(client, "CNC.Config", "CNC.Config.SetSoftLimit",
                                        4, in, &rc);
            for (int i = 0; i < 4; i++) UA_Variant_clear(&in[i]);
            snprintf(buf, sizeof(buf), "rc=%d (expect 0)", rc);
            check(cok == 0 && rc == 0, "SetSoftLimit(X -10..800) -> 0", buf);

            double back = -1;
            read_double(client, "CNC.Config.Axis.X.SoftLimitPos", &back);
            snprintf(buf, sizeof(buf), "SoftLimitPos=%.1f (expect 800)", back);
            check(back > 799.0 && back < 801.0, "Readback SoftLimitPos", buf);
        }

        /* 10-6. SetPulsePerUnit(X, 10000) → 2=待重启 (sim 下档案拒绝落盘 → -1) */
        {
            UA_Variant in[2];
            UA_String axis = UA_STRING((char *)"X");
            UA_Double ppu = 10000.0;
            UA_Variant_setScalarCopy(&in[0], &axis, &UA_TYPES[UA_TYPES_STRING]);
            UA_Variant_setScalarCopy(&in[1], &ppu, &UA_TYPES[UA_TYPES_DOUBLE]);
            int cok = call_method_int32(client, "CNC.Config", "CNC.Config.SetPulsePerUnit",
                                        2, in, &rc);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
            snprintf(buf, sizeof(buf), "rc=%d (2=pending restart / -1=sim no-persist)", rc);
            check(cok == 0 && (rc == 2 || rc == -1),
                  "SetPulsePerUnit -> 2 (or -1 in sim)", buf);
            /* ppu 运行值必须未变 (重启级: 不动 g_axis) */
            double ppu_now = -1;
            read_double(client, "CNC.Config.Axis.X.PulsePerUnit", &ppu_now);
            snprintf(buf, sizeof(buf), "ppu=%.4f (unchanged)", ppu_now);
            check(ppu_now > 9999.0 && ppu_now < 10001.0,
                  "PulsePerUnit runtime value unchanged", buf);
        }

        /* 10-7. ProfileDirty + SaveProfile (sim 拒绝落盘 → -1; hw → 0) */
        {
            int32_t d2 = -1;
            read_int(client, "CNC.Config.ProfileDirty", &d2);
            int cok = call_method_int32(client, "CNC.Config", "CNC.Config.SaveProfile",
                                        0, NULL, &rc);
            snprintf(buf, sizeof(buf), "dirty=%d save_rc=%d", d2, rc);
            check(cok == 0 && (rc == 0 || rc == -1),
                  "SaveProfile -> 0 (or -1 in sim)", buf);
        }

        /* 10-8. 恢复写保护 → 再改必拒 (现场收尾习惯: 测完锁回) */
        {
            write_bool(client, "CNC.Config.WriteProtect", 1);
            UA_Variant in[2];
            UA_Double tol = 0.05, acc = 500.0;
            UA_Variant_setScalarCopy(&in[0], &tol, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[1], &acc, &UA_TYPES[UA_TYPES_DOUBLE]);
            int cok = call_method_int32(client, "CNC.Config", "CNC.Config.SetPlannerParams",
                                        2, in, &rc);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
            snprintf(buf, sizeof(buf), "rc=%d (expect -2)", rc);
            check(cok == 0 && rc == -2, "Re-protect -> Set rejected (-2)", buf);
        }
    }

    /* ==================== §11 Laser 面契约 (2026-09-03, P0-3) ====================
     * CNC.Laser 15 只读状态变量 + 6 运行时工艺方法。
     * 跑法: sudo ./rpc_server sim & 后 ./opcua_probe (激光需 sim 默认
     * SMC_ConfigLaserIO(0,0,0) 已配置, 否则状态全零仍应通过值域断言)。 */
    {
        printf("\n--- §11 Laser nodes & methods ---\n");
        int32_t vi = -1;
        double   vd = -1.0;

        /* 11-0. 前置: ConfigRange 配激光量程 (sim 下 kernel_init 不配, 量程=0,
         * SetPower/SetFrequency 会被防呆拒)。freq_max 50000 覆盖 movecontrol
         * 常用 20000Hz (对齐 scripts/test_laser_config_rpc.py 默认量程)。 */
        {
            UA_Variant in[3];
            UA_Double pmax = 3000.0, fmax = 50000.0, pmin = 50.0;
            UA_Variant_setScalarCopy(&in[0], &pmax, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[1], &fmax, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Variant_setScalarCopy(&in[2], &pmin, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.ConfigRange", 3, in, &bv);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]); UA_Variant_clear(&in[2]);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d", cok, bv);
            check(cok == 0 && bv == 1, "ConfigRange(3000,50000,50) accepted", buf);
        }

        /* 11-1. 状态变量契约: 类型 + 值域 (未配置激光时全零也满足) */
        read_int(client, "CNC.Laser.Enable", &vi);
        snprintf(buf, sizeof(buf), "Enable=%d", vi);
        check(vi == 0 || vi == 1, "Laser.Enable in {0,1}", buf);
        read_int(client, "CNC.Laser.Shutter", &vi);
        snprintf(buf, sizeof(buf), "Shutter=%d", vi);
        check(vi == 0 || vi == 1, "Laser.Shutter in {0,1}", buf);
        read_double(client, "CNC.Laser.PowerW", &vd);
        snprintf(buf, sizeof(buf), "PowerW=%.1f", vd);
        check(vd >= 0.0, "Laser.PowerW >= 0", buf);
        read_double(client, "CNC.Laser.FreqHz", &vd);
        snprintf(buf, sizeof(buf), "FreqHz=%.1f", vd);
        check(vd >= 0.0, "Laser.FreqHz >= 0", buf);
        read_int(client, "CNC.Laser.GasSelect", &vi);
        snprintf(buf, sizeof(buf), "GasSelect=%d", vi);
        check(vi >= 0 && vi <= 3, "Laser.GasSelect in 0..3", buf);
        read_double(client, "CNC.Laser.GasPressure", &vd);
        snprintf(buf, sizeof(buf), "GasPressure=%.2f", vd);
        check(vd >= 0.0, "Laser.GasPressure >= 0", buf);
        read_int(client, "CNC.Laser.Interlock", &vi);
        snprintf(buf, sizeof(buf), "Interlock=0x%X", vi);
        check(vi >= 0, "Laser.Interlock >= 0", buf);
        read_int(client, "CNC.Laser.EmergencyKill", &vi);
        snprintf(buf, sizeof(buf), "EKill=%d", vi);
        check(vi == 0 || vi == 1, "Laser.EmergencyKill in {0,1}", buf);
        read_double(client, "CNC.Laser.BasePowerW", &vd);
        snprintf(buf, sizeof(buf), "BasePowerW=%.1f", vd);
        check(vd >= 0.0, "Laser.BasePowerW >= 0", buf);
        read_double(client, "CNC.Laser.ActualSpeed", &vd);
        snprintf(buf, sizeof(buf), "ActualSpeed=%.3f", vd);
        check(vd >= 0.0, "Laser.ActualSpeed >= 0", buf);
        read_int(client, "CNC.Laser.CouplingMode", &vi);
        snprintf(buf, sizeof(buf), "CouplingMode=%d", vi);
        check(vi == 0 || vi == 1, "Laser.CouplingMode in {0,1}", buf);
        read_int(client, "CNC.Laser.IsPiercing", &vi);
        snprintf(buf, sizeof(buf), "IsPiercing=%d", vi);
        check(vi == 0 || vi == 1, "Laser.IsPiercing in {0,1}", buf);
        read_int(client, "CNC.Laser.SegFlags", &vi);
        snprintf(buf, sizeof(buf), "SegFlags=%d", vi);
        check(vi >= 0 && vi <= 3, "Laser.SegFlags in 0..3", buf);
        read_int(client, "CNC.Laser.PierceCount", &vi);
        snprintf(buf, sizeof(buf), "PierceCount=%d", vi);
        check(vi >= 0, "Laser.PierceCount >= 0", buf);
        {
            UA_Variant v;
            UA_Variant_init(&v);
            UA_StatusCode sc = read_variant(client, "CNC.Laser.OnTimeMs", &v);
            int ok64 = (sc == UA_STATUSCODE_GOOD)
                       && UA_Variant_hasScalarType(&v, &UA_TYPES[UA_TYPES_INT64]);
            UA_Variant_clear(&v);
            check(ok64, "Laser.OnTimeMs Int64 readable", "");
        }

        /* 11-2. SetCoupling(1, 5.0) → true + CouplingMode 即时回读 ==1 */
        {
            UA_Variant in[2];
            UA_Int32 mode = 1;
            UA_Double vt = 5.0;
            UA_Variant_setScalarCopy(&in[0], &mode, &UA_TYPES[UA_TYPES_INT32]);
            UA_Variant_setScalarCopy(&in[1], &vt, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.SetCoupling", 2, in, &bv);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
            int32_t cm = -1;
            read_int(client, "CNC.Laser.CouplingMode", &cm);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d CouplingMode=%d", cok, bv, cm);
            check(cok == 0 && bv == 1 && cm == 1,
                  "SetCoupling(1,5.0) effective", buf);
        }
        /* 11-3. SetPower 合法值接受 + 越界拒绝 */
        {
            UA_Variant in[1];
            UA_Double w = 1500.0;
            UA_Variant_setScalarCopy(&in[0], &w, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.SetPower", 1, in, &bv);
            UA_Variant_clear(&in[0]);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d", cok, bv);
            check(cok == 0 && bv == 1, "SetPower(1500) accepted", buf);
        }
        {
            UA_Variant in[1];
            UA_Double w = 99999.0;
            UA_Variant_setScalarCopy(&in[0], &w, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.SetPower", 1, in, &bv);
            UA_Variant_clear(&in[0]);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d (expect 0)", cok, bv);
            check(cok == 0 && bv == 0, "SetPower(99999) rejected", buf);
        }
        /* 11-4. ConfigGasRange(2, 10.0) + SetGas(1, 2.5) → GasSelect 回读 ==1 */
        {
            UA_Variant in[2];
            UA_Int32 ch = 2;
            UA_Double mb = 10.0;
            UA_Variant_setScalarCopy(&in[0], &ch, &UA_TYPES[UA_TYPES_INT32]);
            UA_Variant_setScalarCopy(&in[1], &mb, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.ConfigGasRange", 2, in, &bv);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d", cok, bv);
            check(cok == 0 && bv == 1, "ConfigGasRange(2,10.0) accepted", buf);
        }
        {
            UA_Variant in[2];
            UA_Int32 sel = 1;
            UA_Double p = 2.5;
            UA_Variant_setScalarCopy(&in[0], &sel, &UA_TYPES[UA_TYPES_INT32]);
            UA_Variant_setScalarCopy(&in[1], &p, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.SetGas", 2, in, &bv);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
            int32_t gs = -1;
            read_int(client, "CNC.Laser.GasSelect", &gs);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d GasSelect=%d", cok, bv, gs);
            /* 注: GasSelect 是 parser pending (下段快照生效), 空闲态 g_laser_rt
             * 镜像可能仍 0 — 仅断言方法层成功, 镜像由 sim 用例覆盖 */
            check(cok == 0 && bv == 1, "SetGas(1,2.5) accepted", buf);
        }
        /* 11-5. ConfigMicroJoint(0.4) → true */
        {
            UA_Variant in[1];
            UA_Double rj = 0.4;
            UA_Variant_setScalarCopy(&in[0], &rj, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.ConfigMicroJoint", 1, in, &bv);
            UA_Variant_clear(&in[0]);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d", cok, bv);
            check(cok == 0 && bv == 1, "ConfigMicroJoint(0.4) accepted", buf);
        }
        /* 11-5b. SetFrequency(20000) → true (I-4: movecontrol int→double 转换语义) */
        {
            UA_Variant in[1];
            UA_Double fz = 20000.0;
            UA_Variant_setScalarCopy(&in[0], &fz, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            int cok = call_method_bool_args(client, "CNC.Laser",
                                            "CNC.Laser.SetFrequency", 1, in, &bv);
            UA_Variant_clear(&in[0]);
            double fhz = -1;
            read_double(client, "CNC.Laser.FreqHz", &fhz);
            snprintf(buf, sizeof(buf), "cok=%d ok=%d FreqHz=%.1f", cok, bv, fhz);
            check(cok == 0 && bv == 1, "SetFrequency(20000) accepted", buf);
        }
        /* 11-6. 收尾还原: SetCoupling(0, 5.0) + ConfigMicroJoint(1.0) */
        {
            UA_Variant in[2];
            UA_Int32 mode = 0;
            UA_Double vt = 5.0;
            UA_Variant_setScalarCopy(&in[0], &mode, &UA_TYPES[UA_TYPES_INT32]);
            UA_Variant_setScalarCopy(&in[1], &vt, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            call_method_bool_args(client, "CNC.Laser",
                                  "CNC.Laser.SetCoupling", 2, in, &bv);
            UA_Variant_clear(&in[0]); UA_Variant_clear(&in[1]);
            check(bv == 1, "restore SetCoupling(0,5.0)", "");
        }
        {
            UA_Variant in[1];
            UA_Double rj = 1.0;
            UA_Variant_setScalarCopy(&in[0], &rj, &UA_TYPES[UA_TYPES_DOUBLE]);
            int bv = -1;
            call_method_bool_args(client, "CNC.Laser",
                                  "CNC.Laser.ConfigMicroJoint", 1, in, &bv);
            UA_Variant_clear(&in[0]);
            check(bv == 1, "restore ConfigMicroJoint(1.0)", "");
        }
    }

    /* ==================== §12 PLC watch 面 (2026-09-12, P1-5) ====================
     * CNC.PLC 13 只读变量 + ReadVar 方法。probe 环境 plc_prog.st 不存在 →
     * 槽 EMPTY (State=0 / NVars=0); ReadVar 任意名 → ret=-1。
     * (有 autoload 的机器上 State 可能=2, 值域断言兼容两种。) */
    {
        printf("\n--- §12 PLC watch nodes & method ---\n");
        int32_t vi = -1;

        read_int(client, "CNC.PLC.State", &vi);
        snprintf(buf, sizeof(buf), "State=%d", vi);
        check(vi >= 0 && vi <= 3, "PLC.State in 0..3", buf);
        read_int(client, "CNC.PLC.NVars", &vi);
        snprintf(buf, sizeof(buf), "NVars=%d", vi);
        check(vi >= 0, "PLC.NVars >= 0", buf);
        read_int(client, "CNC.PLC.Heartbeat", &vi);
        snprintf(buf, sizeof(buf), "Heartbeat=%d", vi);
        check(vi >= 0, "PLC.Heartbeat >= 0", buf);
        read_int(client, "CNC.PLC.ErrLine", &vi);
        snprintf(buf, sizeof(buf), "ErrLine=%d", vi);
        check(vi >= 0, "PLC.ErrLine >= 0", buf);
        read_int(client, "CNC.PLC.GMcode", &vi);
        snprintf(buf, sizeof(buf), "GMcode=%d", vi);
        check(vi >= 0, "PLC.GMcode >= 0", buf);
        read_int(client, "CNC.PLC.GMcodeStrobe", &vi);
        snprintf(buf, sizeof(buf), "GMcodeStrobe=%d", vi);
        check(vi == 0 || vi == 1, "PLC.GMcodeStrobe in {0,1}", buf);
        {
            char emsg[80];
            int rok = read_string(client, "CNC.PLC.ErrMsg", emsg, sizeof(emsg));
            snprintf(buf, sizeof(buf), "read=%d msg=%s", rok, emsg);
            check(rok == 0, "PLC.ErrMsg String readable", buf);
        }
        /* ReadVar: 空槽 → ret=-1 (2 出参: Double value + Int32 ret) */
        {
            UA_Variant in[1];
            UA_String nm = UA_STRING("cnt");
            UA_Variant_setScalarCopy(&in[0], &nm, &UA_TYPES[UA_TYPES_STRING]);
            UA_NodeId o = UA_NODEID_STRING(1, (char *)"CNC.PLC");
            UA_NodeId m = UA_NODEID_STRING(1, (char *)"CNC.PLC.ReadVar");
            size_t out_sz = 0;
            UA_Variant *out = NULL;
            UA_StatusCode sc = UA_Client_call(client, o, m, 1, in, &out_sz, &out);
            int32_t ret = -99;
            if (sc == UA_STATUSCODE_GOOD && out_sz == 2 &&
                UA_Variant_hasScalarType(&out[0], &UA_TYPES[UA_TYPES_DOUBLE]) &&
                UA_Variant_hasScalarType(&out[1], &UA_TYPES[UA_TYPES_INT32]))
                ret = *(int32_t *)out[1].data;
            if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_TYPES_VARIANT]);
            UA_Variant_clear(&in[0]);
            snprintf(buf, sizeof(buf), "sc=0x%x ret=%d", sc, ret);
            check(ret == -1, "PLC.ReadVar(empty) ret=-1", buf);
        }
    }

    UA_Client_disconnect(client);
    UA_Client_delete(client);

    printf("=== RESULT: %d PASS, %d FAIL ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
