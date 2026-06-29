(Phase 2B M5 T7 验证: WHILE 循环内调子程序)
(预期: WHILE 3 次循环, 每次调 O100, O200 跑 G91 X1, X 累计到 3)
(判据: [Loader] WHILE/DO/END 配对完成: 1 对)
(     [Loader] O/M99 子程序配对完成: 1 个)
(     3 次 WHILE 条件真 + 3 次 M98 调用 + 1 次条件假)
(     调用栈与 WHILE PC 跳转正交)
(     CSV 末行 X = 3.000000)
#10 = 0
WHILE [#10 LT 3] DO 1
M98 P100
#10 = #10 + 1
END 1
M30

O0100
G91 G01 X1 F100
M99
