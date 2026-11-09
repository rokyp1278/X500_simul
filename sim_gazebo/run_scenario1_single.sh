#!/bin/bash
set -u; PROJ=/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo; LOG=/tmp/s1s; mkdir -p $LOG
for n in gzserver gzclient gazebo; do pkill -9 -x $n 2>/dev/null; done; sleep 1
PID=$(ss -tlnp 2>/dev/null|grep :11345|grep -oE "pid=[0-9]+"|head -1|cut -d= -f2); [ -n "$PID" ] && kill -9 $PID 2>/dev/null; sleep 1
set +u; source /usr/share/gazebo/setup.sh; set -u
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH:-}
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1
python3 $PROJ/sim_gazebo/build_scenario1_single.py 2>&1 | tee $LOG/build.log
echo "[s1s] launching gazebo GUI (single realistic drone, ~27s)..."
gazebo --verbose $HOME/ardupilot_gazebo/worlds/scenario1_single.world > $LOG/gz.log 2>&1 & GZ=$!
sleep 34; echo "[s1s] keeping window 6s"; sleep 6
grep -iE "error|unable|invalid mesh|iris.dae" $LOG/gz.log | grep -ivE "bind|already|audio|OpenAL|ALSA|skin|__default__" | head -6
kill $GZ 2>/dev/null; echo "[s1s] done"
