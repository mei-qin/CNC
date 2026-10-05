# L1.5 pred/obs 报告 (capture_DryT0c.csv)
- 采样 65535 行, ts=1.000 ms; 块 6, 窗口 0

| line | cmd | L | F | L*F | fu_med | P1 | rv_med | P2 | P3 |
|---|---|---|---|---|---|---|---|---|---|
| 13 | G1 | 10.00000 | 24 | 240.00 | nan | -- | nan | -- | -- |
| 14 | G1 | 10.00000 | 30 | 300.00 | nan | -- | nan | -- | -- |
| 15 | G1 | 10.00000 | 60 | 600.00 | nan | -- | nan | -- | -- |
| 16 | G1 | 10.00000 | 60 | 600.00 | nan | -- | nan | -- | -- |
| 17 | G1 | 14.14214 | 30 | 424.26 | nan | -- | nan | -- | -- |
| 18 | G1 | 14.14214 | 60 | 848.53 | nan | -- | nan | -- | -- |

## P1-P6
| # | 描述 | 判定 |
|---|---|---|
| P1 | feed_upm == L*F (正常态) | FAIL |
| P2 | requested_vel == L*F/60, 不随 override 变 | FAIL |
| P3 | override 窗口: feed_upm 缩放 == ovr, requested_vel 不变 | FAIL |
| P4 | program-line 块身份 1:1/单调/连续 | FAIL |
| P5 | H2/H3 = UNOBSERVABLE (不重建证据) | PASS |
| P6 | feedhold 特征窗口存在 (fu==0 >=0.1s) | FAIL |

## H1
| 轴 | v_ref_max | INI 限 | OK |
|---|---|---|---|
| X | 0.0 | 200.0 | True |
| Y | 0.0 | 200.0 | True |
| Z | 0.0 | 200.0 | True |

## holds: []
## M_obs = {'H1': 'OBSERVABLE', 'H2': 'UNOBSERVABLE', 'H3': 'UNOBSERVABLE', 'H4': 'PARTIAL', 'H5': 'PARTIAL'}

M_PRED = [H1:OBS, H2:UNO*, H3:UNO, H4:PARTIAL, H5:PARTIAL]