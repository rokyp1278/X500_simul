#!/bin/bash
# L5 live orchestration: headless Gazebo (camera frames) + ArduCopter SITL (5760,
# lockstep) + closed-loop track/attack/defend. Run in WSL Ubuntu-22.04.
# Only touches the two child processes it starts (by PID) -- no pkill.
set -u
PROJ=/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo
LOG=/tmp/l5_live
mkdir -p $LOG
: > $LOG/status.txt
say(){ echo "[$(date +%H:%M:%S)] $*" | tee -a $LOG/status.txt; }

# --- env ---  (setup.sh references unbound vars -> guard with set +u)
set +u; source /usr/share/gazebo/setup.sh; set -u
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH:-}
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$HOME/gst_cam/build:${GAZEBO_PLUGIN_PATH:-}
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1

find /tmp/fpv_frames -maxdepth 1 -name '*.jpg' -delete 2>/dev/null
say "start (ports pre-checked free; cleared old frames)"

# --- 1. Gazebo server (headless, software GL) ---
gzserver --verbose $HOME/ardupilot_gazebo/worlds/track_demo.world > $LOG/gz.log 2>&1 &
GZ=$!; say "gzserver pid=$GZ"
sleep 12

# --- 2. ArduCopter SITL (drives the world via lockstep, MAVLink on 5760) ---
cd $HOME/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 \
        --no-mavproxy --no-rebuild > $LOG/sitl.log 2>&1 &
SITL=$!; say "sitl pid=$SITL"

# --- 3. wait for MAVLink 5760 + fresh camera frames ---
for i in $(seq 1 45); do ss -tln 2>/dev/null | grep -q ':5760' && { say "MAVLink 5760 up (${i}x2s)"; break; }; sleep 2; done
for i in $(seq 1 30); do n=$(ls /tmp/fpv_frames/*.jpg 2>/dev/null | wc -l); [ "$n" -gt 3 ] && { say "camera frames=$n"; break; }; sleep 2; done

# --- 4. closed-loop L5 ---
say "launching L5 closed loop..."
cd $PROJ
python3 sim_gazebo/l5_track_attack_defend.py > $LOG/l5.log 2>&1
RC=$?
say "L5 finished rc=$RC"
tail -5 $LOG/l5.log | sed 's/^/[L5] /' | tee -a $LOG/status.txt

# --- 5. cleanup: only the children we started ---
kill $SITL 2>/dev/null
kill $GZ 2>/dev/null
say "cleaned up children. DONE rc=$RC"
