#!/bin/bash
set -u; PROJ=/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo
set +u; source /usr/share/gazebo/setup.sh; set -u
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH:-}
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$HOME/gst_cam/build:${GAZEBO_PLUGIN_PATH:-}
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1
runone(){ W=$1; LOG=/tmp/single_$W; mkdir -p $LOG
  python3 $PROJ/sim_gazebo/build_single.py $W >/dev/null
  find /tmp/fpv_frames -maxdepth 1 -name '*.jpg' -delete 2>/dev/null
  gzserver --verbose $HOME/ardupilot_gazebo/worlds/single.world > $LOG/gz.log 2>&1 & GZ=$!; sleep 12
  cd $HOME/ardupilot; python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --no-mavproxy --no-rebuild > $LOG/sitl.log 2>&1 & SITL=$!
  for i in $(seq 1 45); do ss -tln 2>/dev/null|grep -q ':5760' && break; sleep 2; done
  for i in $(seq 1 30); do [ "$(ls /tmp/fpv_frames/*.jpg 2>/dev/null|wc -l)" -gt 3 ] && break; sleep 2; done
  cd $PROJ; python3 sim_gazebo/measure_single.py $W 2>&1 | grep RESULT
  kill $SITL 2>/dev/null; kill $GZ 2>/dev/null; sleep 5; }
runone clean
runone patched
echo "=== SUMMARY ==="; cat /mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim/single_clean.txt /mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/outputs/sim/single_patched.txt 2>/dev/null
