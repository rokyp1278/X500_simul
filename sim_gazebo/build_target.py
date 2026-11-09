#!/usr/bin/env python3
import os,shutil,sys
HOME=os.path.expanduser("~");MODELS=os.path.join(HOME,"ardupilot_gazebo","models");WORLDS=os.path.join(HOME,"ardupilot_gazebo","worlds")
SG="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/sim_gazebo"
which=sys.argv[1] if len(sys.argv)>1 else "clean"
tex=os.path.join(SG,"bus_%s.png"%("patched" if which=="patched" else "clean"))
name="tgt_bb";d=os.path.join(MODELS,name)
os.makedirs(os.path.join(d,"materials","scripts"),exist_ok=True);os.makedirs(os.path.join(d,"materials","textures"),exist_ok=True)
shutil.copy(tex,os.path.join(d,"materials","textures","bus.png"))
open(os.path.join(d,"materials","scripts","tex.material"),"w").write("material TgtBus\n{\n technique\n {\n  pass\n  {\n   texture_unit\n   {\n    texture bus.png\n   }\n  }\n }\n}\n")
open(os.path.join(d,"model.config"),"w").write('<?xml version="1.0"?>\n<model><name>tgt_bb</name><version>1.0</version><sdf version="1.6">model.sdf</sdf></model>\n')
open(os.path.join(d,"model.sdf"),"w").write('<?xml version="1.0"?>\n<sdf version="1.6">\n <model name="tgt_bb">\n  <static>true</static>\n  <link name="panel">\n   <visual name="vis">\n    <geometry><box><size>0.1 4.5 3.0</size></box></geometry>\n    <material><script><uri>model://tgt_bb/materials/scripts</uri><uri>model://tgt_bb/materials/textures</uri><name>TgtBus</name></script></material>\n   </visual>\n  </link>\n </model>\n</sdf>\n')
world='<?xml version="1.0"?>\n<sdf version="1.6">\n <world name="fly">\n  <include><uri>model://sun</uri></include>\n  <include><uri>model://ground_plane</uri></include>\n  <include><uri>model://tgt_bb</uri><name>tgt</name><pose>10 0 3 0 0 0</pose></include>\n  <include><uri>model://iris_fpv_cam</uri><name>iris_fpv_cam</name><pose>0 0 0 0 0 0</pose></include>\n  <physics type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>\n </world>\n</sdf>\n'
open(os.path.join(WORLDS,"fly.world"),"w").write(world)
print("fly.world built:",which)
