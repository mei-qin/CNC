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
#define PROBE_NC_DIR_DEFAULT "/home/meiqin/nc"
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
        read_double(client, "CNC.ActualSpeed",         &speed);
        read_double(client, "CNC.FeedOverride",        &fovr);
        read_double(client, "CNC.RapidOverride",       &rovr);
        read_double(client, "CNC.SpindleOverride",     &sovr);

        printf("        Status=%d Mode=%d FeedOvr=%.2f RapidOvr=%.2f SpindleOvr=%.2f\n",
               status, mode, fovr, rovr, sovr);
        printf("        DateTime=%s PowerOn=%s Program=%s Line=%d/%d Alarm=%d\n",
               dt, span, name, cur_line, line_cnt, alarm_cnt);

        check(status >= 0 && status <= 5, "Read CNC.Machine.Status (0..5)", NULL);
        check(mode == 0 || mode == 1,      "Read CNC.Machine.Mode (0/1)", NULL);
        check(dt[0] == '2',                "Read CNC.Machine.DateTime (ISO8601)", dt);
        check(strchr(span, 'h') != NULL,   "Read CNC.Machine.PowerOnSpan (NhNm)", span);
        check(fovr >= 0.0 && fovr <= 1.5,  "Read CNC.FeedOverride (0~1.5)", NULL);
        check(speed >= 0.0,                "Read CNC.ActualSpeed (m/s)", NULL);
        check(alarm_cnt >= 0,              "Read CNC.Alarm.Count", NULL);
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

    /* ---- 4. Write FeedOverride (0.80) + 回读 ---- */
    {
        UA_Variant w;
        UA_Double val = 0.80;
        UA_Variant_setScalarCopy(&w, &val, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_NodeId n = UA_NODEID_STRING(1, (char *)"CNC.FeedOverride");
        UA_StatusCode sc3 = UA_Client_writeValueAttribute(client, n, &w);
        UA_Variant_clear(&w);

        double back = -1.0;
        /* 写生效需穿越 RT 周期 + snapshot 重发, 留足时间再回读 */
        for (int i = 0; i < 10 && (back < 0.7 || back > 0.9); i++) {
            usleep(100000);
            read_double(client, "CNC.FeedOverride", &back);
        }
        snprintf(buf, sizeof(buf), "write=%s readback=%.2f",
                 UA_StatusCode_name(sc3), back);
        check(sc3 == UA_STATUSCODE_GOOD && back > 0.7 && back < 0.9,
              "Write CNC.FeedOverride + readback", buf);
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
        usleep(300000);
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
        if (out) UA_Array_delete(out, out_sz, &UA_TYPES[UA_VARIANT]);
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
    }

    UA_Client_disconnect(client);
    UA_Client_delete(client);

    printf("=== RESULT: %d PASS, %d FAIL ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
