set -eu
cd /mnt/d/code/CNC
d=$(mktemp -d /tmp/cnc-drop.XXXXXX)
src="$PWD/scripts/check_servo_drop.c"
if [ "$#" = 1 ]; then cd "$1"; fi
gcc ${BASEFLAG:-} -c "$src" -o "$d/drop.o" -Iinc -Irpc -Ithird_party/soem/include -Ithird_party/soem/osal -Ithird_party/soem/osal/linux -Ithird_party/soem/oshw/linux -DLINUX -D_GNU_SOURCE
gcc -Wl,--wrap=sim_drive_get_sw,--wrap=sim_drive_step_axis -o "$d/check" "$d/drop.o" $(find src rpc third_party -name '*.o' ! -name main.o) -lpthread -lrt -lm
cd "$d"
timeout 10 ./check sim hw
