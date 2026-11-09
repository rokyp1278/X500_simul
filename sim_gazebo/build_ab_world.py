#!/usr/bin/env python3
"""A/B Gazebo world: a CLEAN bus billboard and an EOT-PATCHED bus billboard side by
side at the same distance, so one flight measures whether the patch shrinks the YOLO
box LIVE (vs the clean baseline). Reuses iris_fpv_cam from build_world_assets.
Run in WSL."""
import os, shutil, sys
HOME=os.path.expanduser("~"); MODELS=os.path.join(HOME,"ardupilot_gazebo","models")
WORLDS=os.path.join(HOME,"ardupilot_gazebo","worlds")
SG="/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/sim_gazebo"
CLEAN=os.path.join(SG,"bus_clean.png"); PATCHED=os.path.join(SG,"bus_patched.png")

def billboard(name, texfile, texname, matname, w, h):
    d=os.path.join(MODELS,name)
    os.makedirs(os.path.join(d,"materials","scripts"),exist_ok=True)
    os.makedirs(os.path.join(d,"materials","textures"),exist_ok=True)
    shutil.copy(texfile, os.path.join(d,"materials","textures",texname))
    open(os.path.join(d,"materials","scripts","tex.material"),"w").write(
        "material %s\n{\n technique\n {\n  pass\n  {\n   texture_unit\n   {\n    texture %s\n   }\n  }\n }\n}\n"%(matname,texname))
    open(os.path.join(d,"model.config"),"w").write(
        '<?xml version="1.0"?>\n<model>\n <name>%s</name>\n <version>1.0</version>\n <sdf version="1.6">model.sdf</sdf>\n</model>\n'%name)
    sdf='<?xml version="1.0"?>\n<sdf version="1.6">\n <model name="%s">\n  <static>true</static>\n  <link name="panel">\n   <visual name="vis">\n    <geometry><box><size>0.1 %.3f %.3f</size></box></geometry>\n    <material><script>\n     <uri>model://%s/materials/scripts</uri>\n     <uri>model://%s/materials/textures</uri>\n     <name>%s</name>\n    </script></material>\n   </visual>\n  </link>\n </model>\n</sdf>\n'%(name,w,h,name,name,matname)
    open(os.path.join(d,"model.sdf"),"w").write(sdf); print("billboard:",name)

WORLD="""<?xml version="1.0"?>
<sdf version="1.6">
 <world name="ab_demo">
  <include><uri>model://sun</uri></include>
  <include><uri>model://ground_plane</uri></include>
  <include><uri>model://bus_clean_bb</uri><name>clean</name><pose>7 2.3 1.7 0 0 0</pose></include>
  <include><uri>model://bus_patched_bb</uri><name>patched</name><pose>7 -2.3 1.7 0 0 0</pose></include>
  <include><uri>model://iris_fpv_cam</uri><name>iris_fpv_cam</name><pose>0 0 0 0 0 0</pose></include>
  <physics type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>
 </world>
</sdf>
"""
def main():
    for p in (CLEAN,PATCHED):
        if not os.path.exists(p): print("MISSING",p); sys.exit(1)
    billboard("bus_clean_bb", CLEAN, "bus_clean.png", "ABclean", 2.6, 3.2)
    billboard("bus_patched_bb", PATCHED, "bus_patched.png", "ABpatched", 2.6, 3.2)
    open(os.path.join(WORLDS,"ab_demo.world"),"w").write(WORLD)
    print("world: ab_demo.world")
if __name__=="__main__": main()
