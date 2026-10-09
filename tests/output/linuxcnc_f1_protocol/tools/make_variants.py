#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""F1 protocol variant constructor (text-only, no builds).

Reconstructs the four prospective source variants from pristine bytes and
emits unified diffs. Byte-exactness requirements:
  - W patch replicates apply_f1_patch.py (F-1 G2 prereg) semantics; the
    patched interp_convert.cc MUST reproduce historical sha 6ab767c7...
  - T patch mirrors apply_wpb_seed.py style on the arc twin in
    interp_inverse.cc (anchor unique to the arc function).
"""
import difflib
import hashlib
import sys

BASE = r"D:\code\CNC\tests\output\linuxcnc_f1_protocol"
HIST_W_SHA = "6ab767c785b1d402e4795c67766c405e6fd8c2390de63bda7262688696fcda20"

W_SITES = [
    ("inverse_time_rate_arc(*current1, *current2, *current3, center1, center2,\n"
     "                        turn, end1, end2, end3, block, settings);",
     "  write_canon_state_tag(block, settings);"),
    ("inverse_time_rate_arc(cx, cy, cz, center_x, center_y,\n"
     "                          turn, end_x, end_y, end_z, block, settings);",
     "    write_canon_state_tag(block, settings);"),
    ("inverse_time_rate_arc(opx, opy, opz, centerx, centery,\n"
     "                          turn, end_x, end_y, end_z, block, settings);",
     "    write_canon_state_tag(block, settings);"),
]

T_OLD = (
    "  } else {\n"
    "    rate = length * block->f_number;\n"
    "  }\n"
    "  enqueue_SET_FEED_RATE(rate);"
)

T_NEW = (
    "  } else {\n"
    "    rate = length * block->f_number;\n"
    "    /* F1-T1: preregistered seeded arc transformation violation (positive\n"
    "       control, delta = 0.05). Multiplies the post-conversion inverse-time\n"
    "       arc rate by (1 + delta) ONLY for the preregistered target block\n"
    "       (settings->sequence_number == 16, i.e. source line 16 of\n"
    "       g93_probe.ngc). EXPERIMENTAL INTERVENTION, not a LinuxCNC defect.\n"
    "       Prereg: linuxcnc_f1_protocol/. */\n"
    "    if (settings->sequence_number == 16) {\n"
    "      rate = rate * 1.05;\n"
    "    }\n"
    "  }\n"
    "  enqueue_SET_FEED_RATE(rate);"
)


def sha(t):
    return hashlib.sha256(t.encode("utf-8")).hexdigest()


def apply_w(text):
    if "write_canon_state_tag(block, settings);\n\n  ARC_FEED" in text:
        sys.exit("FATAL: pristine file already contains W patch")
    for anchor, insert in W_SITES:
        if text.count(anchor) != 1:
            sys.exit("FATAL: W anchor not unique (%d)" % text.count(anchor))
        text = text.replace(anchor, anchor + "\n" + insert)
    return text


def apply_t(text):
    if "F1-T1" in text:
        sys.exit("FATAL: pristine file already contains T seed")
    if text.count(T_OLD) != 1:
        sys.exit("FATAL: T anchor not unique (%d)" % text.count(T_OLD))
    if text.count("find_arc_length(") < 1:
        sys.exit("FATAL: arc context missing")
    return text.replace(T_OLD, T_NEW)


def diff(a_name, b_name, a, b):
    return "".join(difflib.unified_diff(
        a.splitlines(True), b.splitlines(True), a_name, b_name, n=3))


def main():
    inv = open(BASE + r"\pristine\interp_inverse.cc", encoding="utf-8").read()
    cnv = open(BASE + r"\pristine\interp_convert.cc", encoding="utf-8").read()
    inv_t = apply_t(inv)
    cnv_w = apply_w(cnv)

    w_sha = sha(cnv_w)
    print("W patched interp_convert sha256:", w_sha)
    print("  reproduces historical 6ab767c7:", "YES" if w_sha == HIST_W_SHA else "NO <-- INVESTIGATE")
    print("T seeded  interp_inverse  sha256:", sha(inv_t))

    open(BASE + r"\variants\T0W0_interp_convert.cc", "w", encoding="utf-8", newline="").write(cnv_w)
    open(BASE + r"\variants\T1W1_interp_inverse.cc", "w", encoding="utf-8", newline="").write(inv_t)

    pd = diff("a/src/emc/rs274ngc/interp_convert.cc", "b/src/emc/rs274ngc/interp_convert.cc", cnv, cnv_w)
    td = diff("a/src/emc/rs274ngc/interp_inverse.cc", "b/src/emc/rs274ngc/interp_inverse.cc", inv, inv_t)
    open(BASE + r"\patches\F1_W_tag_interp_convert.patch", "w", encoding="utf-8", newline="").write(pd)
    open(BASE + r"\patches\F1_T_seed_interp_inverse.patch", "w", encoding="utf-8", newline="").write(td)
    open(BASE + r"\patches\F1_TW_combined.patch", "w", encoding="utf-8", newline="").write(td + pd)

    with open(BASE + r"\variants\HASHES.txt", "w", encoding="utf-8", newline="") as f:
        f.write("pristine/interp_inverse.cc  %s\n" % sha(inv))
        f.write("pristine/interp_convert.cc  %s\n" % sha(cnv))
        f.write("variants/T1W1_interp_inverse.cc (T seed)      %s\n" % sha(inv_t))
        f.write("variants/T0W0_interp_convert.cc (W patch)     %s\n" % sha(cnv_w))
    print("diff line counts: T=%d W=%d" % (len(td.splitlines()), len(pd.splitlines())))
    print("anchor uniqueness: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
