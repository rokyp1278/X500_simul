#!/usr/bin/env python3
"""REALISTIC single-drone scenario-1, scaled to the real 3m mocap cube. One real iris
mesh drone follows a PATCHED target (static, small, no teleport) at x=2.5m, meaning to
hold standoff D0=1.5m (green safe-line at x=1.0). The adversarial patch shrinks the box
so the drone reads the target as farther -> pulled PAST the safe line to 0.69m (x=1.81).
Native actor auto-plays, no SITL. Numbers = measured r~0.46 at the 3m-cube recipe.
argv 'record' adds an offscreen observer camera that saves frames to /tmp/obs_frames."""
import os, shutil, sys
RECORD = "record" in sys.argv
HOME=os.path.expanduser("~"); MODELS=os.path.join(HOME,"ardupilot_gazebo","models")
WORLDS=os.path.join(HOME,"ardupilot_gazebo","worlds")
SG="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/sim_gazebo"
IRIS="model://iris_with_standoffs/meshes/iris.dae"
TGT_X=2.5; D0=1.5; R=0.46; Z=1.0
HOLD_X=TGT_X-D0            # 1.0  (safe standoff line)
PARK_X=TGT_X-R*D0         # 1.81 (pulled to 0.69 m from target)

# small realistic patched target panel (~0.7w x 0.9h), patch baked into bus_patched.png
name="s1s_patched"; d=os.path.join(MODELS,name)
os.makedirs(os.path.join(d,"materials","scripts"),exist_ok=True)
os.makedirs(os.path.join(d,"materials","textures"),exist_ok=True)
shutil.copy(os.path.join(SG,"bus_patched.png"), os.path.join(d,"materials","textures","bus.png"))
open(os.path.join(d,"materials","scripts","tex.material"),"w").write(
    "material S1SPatched\n{\n technique\n {\n  pass\n  {\n   texture_unit\n   {\n    texture bus.png\n   }\n  }\n }\n}\n")
open(os.path.join(d,"model.config"),"w").write('<?xml version="1.0"?>\n<model><name>%s</name><version>1.0</version><sdf version="1.6">model.sdf</sdf></model>\n'%name)
open(os.path.join(d,"model.sdf"),"w").write('<?xml version="1.0"?>\n<sdf version="1.6">\n <model name="%s">\n  <static>true</static>\n  <link name="panel">\n   <visual name="vis">\n    <geometry><box><size>0.06 0.7 0.9</size></box></geometry>\n    <material><script><uri>model://%s/materials/scripts</uri><uri>model://%s/materials/textures</uri><name>S1SPatched</name></script></material>\n   </visual>\n  </link>\n </model>\n</sdf>\n'%(name,name,name))
print("patched target:",name)

def wp(t,x,z,P=0.0,Y=0.0):   # pose x 0 z  roll=0 pitch=P yaw=Y
    return "   <waypoint><time>%.2f</time><pose>%.3f 0 %.3f 0 %.3f %.3f</pose></waypoint>\n"%(t,x,z,P,Y)
# TAKEOFF -> SEARCH(yaw-sweep, scanning) -> ACQUIRE(face target) -> FOLLOW(hold 1.5m)
# -> fooled -> pulled past the safe line to 0.69 m
traj=(wp(0,0.2,0.2)+wp(3,0.2,Z)                          # takeoff
      +wp(5,0.2,Z,0,0.6)+wp(7,0.2,Z,0,-0.6)+wp(8.5,0.2,Z,0,0)  # SEARCH: yaw sweep R/L, then face +x (acquire)
      +wp(11,HOLD_X*0.6+0.08,Z,-0.10,0)+wp(13,HOLD_X,Z,0,0)    # approach to safe standoff
      +wp(16,HOLD_X,Z,0,0)                                 # holding at safe 1.5 m
      +wp(20,(HOLD_X+PARK_X)/2,Z,-0.09,0)+wp(24,PARK_X,Z,0,0)+wp(30,PARK_X,Z,0,0))  # fooled -> pulled in
drone='<actor name="drone">\n  <link name="body"><visual name="v"><geometry><mesh><uri>%s</uri></mesh></geometry></visual></link>\n  <script><loop>false</loop><auto_start>true</auto_start>\n   <trajectory id="0" type="fly">\n%s   </trajectory>\n  </script>\n </actor>\n'%(IRIS,traj)

# green translucent safe-standoff line at x=HOLD_X (D0=1.5 m); drone should NOT cross it
safe='<model name="safe_line"><static>true</static><pose>%.3f 0 %.2f 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>0.02 1.2 1.4</size></box></geometry><material><ambient>0 0.9 0 0.3</ambient><diffuse>0 0.9 0 0.3</diffuse><transparency>0.65</transparency></material></visual></link></model>'%(HOLD_X,Z)
# 3 m cube wireframe-ish floor marker (2.9 m square outline via a thin plate), for scale
cube='<model name="cube_floor"><static>true</static><pose>1.45 0 0.005 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>2.9 2.9 0.01</size></box></geometry><material><ambient>0.2 0.2 0.25 1</ambient><diffuse>0.25 0.25 0.3 1</diffuse></material></visual></link></model>'

# offscreen observer camera (records frames to /tmp/obs_frames) -- same viewpoint as the GUI
observer=''
if RECORD:
    observer=('<model name="observer"><static>true</static><pose>1.2 -3.4 1.7 0 0.15 1.45</pose>'
              '<link name="l"><sensor name="obscam" type="camera">'
              '<camera><horizontal_fov>1.15</horizontal_fov>'
              '<image><width>960</width><height>540</height></image>'
              '<clip><near>0.05</near><far>60</far></clip>'
              '<save enabled="true"><path>/tmp/obs_frames</path></save></camera>'
              '<always_on>1</always_on><update_rate>25</update_rate></sensor></link></model>')
WORLD='<?xml version="1.0"?>\n<sdf version="1.6">\n <world name="scenario1_single">\n  <include><uri>model://sun</uri></include>\n  <include><uri>model://ground_plane</uri></include>\n  <gui><camera name="user_camera"><pose>1.2 -3.4 1.7 0 0.15 1.45</pose></camera></gui>\n  %s\n  <include><uri>model://s1s_patched</uri><name>target</name><pose>%.2f 0 %.2f 0 0 0</pose></include>\n  %s\n  %s\n  %s\n  <physics type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>\n </world>\n</sdf>\n'%(cube,TGT_X,Z,safe,drone,observer)
open(os.path.join(WORLDS,"scenario1_single.world"),"w").write(WORLD)
print("world: scenario1_single.world  (target %.1fm, safe line %.1fm, drone pulled to %.2fm gap)"%(TGT_X,D0,TGT_X-PARK_X))
