# L1.5 pred/obs 报告 (capture_r2.csv)
- 采样 11214 行, ts=1.000 ms; 块 6, 窗口 6

| line | cmd | L | F | L*F | fu_med | P1 | rv_med | P2 | P3 |
|---|---|---|---|---|---|---|---|---|---|
| 13 | G1 | 10.00000 | 60 | 600.00 | 600.00 | OK | 10.0000 | OK | -- |
| 14 | G1 | 10.00000 | 30 | 300.00 | 300.00 | OK | 5.0000 | OK | OK |
| 15 | G1 | 10.00000 | 60 | 600.00 | 600.00 | OK | 10.0000 | OK | -- |
| 16 | G2 | 15.70796 | 120 | 1884.96 | 600.00 | FAIL 0.3183 | 31.4159 | OK | -- |
| 17 | G1 | 14.14214 | 30 | 424.26 | 424.26 | OK | 7.0711 | OK | -- |
| 18 | G1 | 10.00000 | 60 | 600.00 | 600.00 | OK | 10.0000 | OK | -- |

## P1-P6
| # | 描述 | 判定 |
|---|---|---|
| P1 | feed_upm == L*F (正常态) | FAIL |
| P2 | requested_vel == L*F/60, 不随 override 变 | PASS |
| P3 | override 窗口: feed_upm 缩放 == ovr, requested_vel 不变 | PASS |
| P4 | program-line 块身份 1:1/单调/连续 | PASS |
| P5 | H2/H3 = UNOBSERVABLE (不重建证据) | PASS |
| P6 | feedhold 特征窗口存在 (fu==0 >=0.1s) | PASS |

## H1
| 轴 | v_ref_max | INI 限 | OK |
|---|---|---|---|
| X | 31.36 | 200.0 | True |
| Y | 31.36 | 200.0 | True |
| Z | 0.0 | 200.0 | True |

## holds: [{'i0': 4705, 'i1': 5705, 'dur_s': 1.001}]
## M_obs = {'H1': 'OBSERVABLE', 'H2': 'UNOBSERVABLE', 'H3': 'UNOBSERVABLE', 'H4': 'PARTIAL', 'H5': 'PARTIAL'}

M_PRED = [H1:OBS, H2:UNO*, H3:UNO, H4:PARTIAL, H5:PARTIAL]