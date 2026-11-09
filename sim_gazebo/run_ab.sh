#!/bin/bash
set -u
PROJ=/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo
LOG=/tmp/ab_live; mkdir -p $LOG; : > $LOG/status.txt
say(){ echo "[$(date +%H:%M:%S)] $*" | tee -a $LOG/status.txt; }
set +u; source /usr/share/gazebo/setup.sh; set -u
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH:-}
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$HOME/gst_cam/build:${GAZEBO_PLUGIN_PATH:-}
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1
python3 $PROJ/sim_gazebo/build_ab_world.py 2>&1 | tee -a $LOG/status.txt
find /tmp/fpv_frames -maxdepth 1 -name '*.jpg' -delete 2>/dev/null
say "start"
gzserver --verbose $HOME/ardupilot_gazebo/worlds/ab_demo.world > $LOG/gz.log 2>&1 &
GZ=$!; say "gz pid=$GZ"; sleep 12
cd $HOME/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --no-mavproxy --no-rebuild > $LOG/sitl.log 2>&1 &
SITL=$!; say "sitl pid=$SITL"
for i in $(seq 1 45); do ss -tln 2>/dev/null | grep -q ':5760' && { say "5760 up"; break; }; sleep 2; done
for i in $(seq 1 30); do n=$(ls /tmp/fpv_frames/*.jpg 2>/dev/null|wc -l); [ "$n" -gt 3 ] && { say "frames=$n"; break; }; sleep 2; done
say "launch measure"; cd $PROJ; python3 sim_gazebo/ab_measure.py > $LOG/ab.log 2>&1; RC=$?
say "done rc=$RC"; tail -12 $LOG/ab.log | sed 's/^/[AB] /' | tee -a $LOG/status.txt
kill $SITL 2>/dev/null; kill $GZ 2>/dev/null; say "cleaned rc=$RC"
