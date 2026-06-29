(Phase 2B M3 验证: WHILE/DO/END 块循环)
(预期: 与 test_if_loop.nc 等价, 5 次 G01, X 坐标 0/10/20/30/40)
(判据: 控制台 5 次 "WHILE 条件真" + 1 次 "条件假" + 5 次 "END 跳回 WHILE")
#1 = 0
WHILE [#1 LT 5] DO 1
G01 X[#1 * 10] F500
#1 = #1 + 1
END 1
M30
