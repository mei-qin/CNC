%
(H-1 MIN REPRO: WORK_OFFSET MID-FLIGHT POLLUTION)

(==== Phase 0: 仅初始化 G54 偏置 X=100 ====)
#5221 = 100.0
G17 G21 G40 G49 G54 G80 G90 G94

(==== Phase 1: 起点定位 ====)
G54 G00 X0 Y0

(==== Phase 2: 慢速长运动 100mm @ F100 = 60s ====)
G01 X100 F100

(==== Phase 3: 中途改偏置 -> H-1 核心触发点 ====)
#5221 = 999.0

(==== Phase 4: parser-time 宏探针 ====)
#100 = #5031

(==== Phase 5: 新偏置下的运动 ====)
G01 X0 F500

M30
%
