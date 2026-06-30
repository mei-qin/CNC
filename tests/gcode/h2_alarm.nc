(Hazard 2: Alarm trigger + abort sequence test)
(Phase 1: normal motion)
G90 G17 G54
G00 X0 Y0 Z10
G01 X10 F500

(Phase 2: trigger user alarm via #3000)
#3000 = 1

(Phase 3: this should never execute - parser should abort)
G01 X20 F500
M30
