#!/bin/bash
# Headless record of the realistic single-drone scenario -> GIF + mp4 in outputs/sim.
set -u; PROJ=/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo; LOG=/tmp/s1rec; mkdir -p $LOG
for n in gzserver gzclient gazebo; do pkill -9 -x $n 2>/dev/null; done; sleep 1
PID=$(ss -tlnp 2>/dev/null|grep :11345|grep -oE "pid=[0-9]+"|head -1|cut -d= -f2); [ -n "$PID" ] && kill -9 $PID 2>/dev/null; sleep 1
rm -rf /tmp/obs_frames; mkdir -p /tmp/obs_frames
set +u; source /usr/share/gazebo/setup.sh; set -u
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH:-}
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1
python3 $PROJ/sim_gazebo/build_scenario1_single.py record 2>&1 | tee $LOG/build.log
echo "[rec] headless gzserver (actors play + observer saves frames)..."
gzserver --verbose $HOME/ardupilot_gazebo/worlds/scenario1_single.world > $LOG/gz.log 2>&1 & GZ=$!
# let the full 27s trajectory play, plus warmup
sleep 32
echo "[rec] frames captured: $(ls /tmp/obs_frames/*.jpg 2>/dev/null | wc -l)"
kill $GZ 2>/dev/null; sleep 1
echo "[rec] stitching -> GIF + mp4"
python3 $PROJ/sim_gazebo/make_video.py
echo "[rec] done"
