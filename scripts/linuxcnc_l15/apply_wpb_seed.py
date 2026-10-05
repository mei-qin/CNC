#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apply / revert the WP-B seeded transformation violation (preregistered).

Target: interp_inverse.cc, Interp::inverse_time_rate_straight — the function
that computes the post-conversion inverse-time rate f_u = L * F for straight
G93 moves (the arc twin has an identical statement but is NOT touched; the
anchor below includes the whitespace line unique to the straight function).

Seed: multiplicative bias on the post-conversion rate,
    f_u_seed = (1 + delta) * L * F,  delta = 0.05,
guarded by the preregistered target block identity
    settings->sequence_number == 16   (NC source line 16 of g93t_probe.ngc)

Idempotent-guarded; keeps a .stock backup on first apply; `revert` restores it.
The witness producer (write_canon_state_tag), the observation channel
(motion.feed-upm), the checker (adapter_l15.py) and the independent oracle
(parse_nc, L*F from NC geometry) are NOT modified by this patch.
"""
import hashlib
import shutil
import sys

SRC = "/home/l15/linuxcnc-2.9.4/src/emc/rs274ngc/interp_inverse.cc"
BAK = SRC + ".stock"

TARGET_LINE = 16
DELTA = 0.05

OLD = (
    "  } else {\n"
    "    rate = length * block->f_number;\n"
    "  }\n"
    "  \n"
    "  enqueue_SET_FEED_RATE(rate);"
)

NEW = (
    "  }} else {{\n"
    "    rate = length * block->f_number;\n"
    "    /* WP-B-T1: preregistered seeded transformation violation (positive\n"
    "       control, delta = {delta:.2f}). Multiplies the post-conversion\n"
    "       inverse-time rate by (1 + delta) ONLY for the preregistered target\n"
    "       block (settings->sequence_number == {line}, i.e. source line {line}\n"
    "       of g93t_probe.ngc). EXPERIMENTAL INTERVENTION, not a LinuxCNC\n"
    "       defect. Prereg: linuxcnc_positive_case/prereg/. */\n"
    "    if (settings->sequence_number == {line}) {{\n"
    "      rate = rate * {factor};\n"
    "    }}\n"
    "  }}\n"
    "  \n"
    "  enqueue_SET_FEED_RATE(rate);"
).format(delta=DELTA, line=TARGET_LINE, factor=1.0 + DELTA)


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "apply"
    if mode == "apply":
        text = open(SRC, encoding="utf-8").read()
        if "WP-B-T1" in text:
            print("already seeded; no action")
            return 0
        try:
            shutil.copyfile(SRC, BAK)
            print("backup:", BAK, sha(BAK))
        except FileExistsError:
            pass
        if text.count(OLD) != 1:
            print("FATAL: anchor not unique (%d matches)" % text.count(OLD))
            return 1
        open(SRC, "w", encoding="utf-8").write(text.replace(OLD, NEW))
        print("seed applied: delta=%s target_line=%d" % (DELTA, TARGET_LINE))
        print("seeded sha256:", sha(SRC))
        return 0
    if mode == "revert":
        shutil.copyfile(BAK, SRC)
        print("reverted from backup")
        print("reverted sha256:", sha(SRC))
        return 0
    print("usage: apply_wpb_seed.py [apply|revert]")
    return 2


if __name__ == "__main__":
    sys.exit(main())
