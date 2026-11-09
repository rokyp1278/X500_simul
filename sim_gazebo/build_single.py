#!/usr/bin/env python3
"""Single centered billboard (y=0, frontal) textured CLEAN or PATCHED via argv.
Unambiguous: no overlap, no off-axis. Run in WSL."""
import os,shutil,sys
HOME=os.path.expanduser("~");MODELS=os.path.join(HOME,"ardupilot_gazebo","models");WORLDS=os.path.join(HOME,"ardupilot_gazebo","worlds")
SG="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/sim_gazebo"
which=sys.argv[1] if len(sys.argv)>1 else "clean"
tex=os.path.join(SG,"bus_%s.png"%("patched" if which=="patched" else "clean"))
name="single_bb"
d=os.path.join(MODELS,name)
os.makedirs(os.path.join(d,"materials","scripts"),exist_ok=True);os.makedirs(os.path.join(d,"materials","textures"),exist_ok=True)
shutil.copy(tex,os.path.join(d,"materials","textures","bus.png"))
open(os.path.join(d,"materials","scripts","tex.material"),"w").write("material SingleBus\n{\n technique\n {\n  pass\n  {\n   texture_unit\n   {\n    texture bus.png\n   }\n  }\n }\n}\n")
open(os.path.join(d,"model.config"),"w").write('<?xml version="1.0"?>\n<model><name>%s</name><version>1.0</version><sdf version="1.6">model.sdf</sdf></model>\n'%name)
open(os.path.join(d,"model.sdf"),"w").write('<?xml version="1.0"?>\n<sdf version="1.6">\n <model name="%s">\n  <static>true</static>\n  <link name="panel">\n   <visual name="vis">\n    <geometry><box><size>0.1 4.5 3.0</size></box></geometry>\n    <material><script><uri>model://%s/materials/scripts</uri><uri>model://%s/materials/textures</uri><name>SingleBus</name></script></material>\n   </visual>\n  </link>\n </model>\n</sdf>\n'%(name,name,name))
open(os.path.join(WORLDS,"single.world"),"w").write('<?xml version="1.0"?>\n<sdf version="1.6">\n <world name="single">\n  <include><uri>model://sun</uri></include>\n  <include><uri>model://ground_plane</uri></include>\n  <include><uri>model://single_bb</uri><name>bb</name><pose>7 1.5 1.7 0 0 0</pose></include>\n  <include><uri>model://iris_fpv_cam</uri><name>iris_fpv_cam</name><pose>0 0 0 0 0 0</pose></include>\n  <physics type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>\n </world>\n</sdf>\n')
print("single.world built with",which,"texture")
