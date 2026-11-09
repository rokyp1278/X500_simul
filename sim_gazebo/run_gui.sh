#!/bin/bash
# Launch the A/B billboard world in the FULL Gazebo GUI (visible on the Windows
# desktop via WSLg, hardware GL). Static scene: clean bus vs EOT-patched bus + the
# FPV drone. Just for LOOKING at the sim. Ctrl-C / close window to stop.
set -u
PROJ=/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo
set +u; source /usr/share/gazebo/setup.sh; set -u
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH:-}
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$HOME/gst_cam/build:${GAZEBO_PLUGIN_PATH:-}
export DISPLAY=:0
unset LIBGL_ALWAYS_SOFTWARE   # use WSLg hardware GL (d3d12 -> RTX 5080)
python3 $PROJ/sim_gazebo/build_ab_world.py
echo "[run_gui] launching gazebo GUI..."
exec gazebo --verbose $HOME/ardupilot_gazebo/worlds/ab_demo.world
