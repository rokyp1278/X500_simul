#!/usr/bin/env python3
"""Build Gazebo Classic assets for the adversarial tracking demo (run in WSL).

Creates, under ~/ardupilot_gazebo/models:
  - iris_fpv_cam     : iris + forward camera (GstCameraPlugin, UDP 5600)
  - target_billboard : a tall panel textured with bus.jpg (YOLO detects bus)
  - patch_panel      : a panel textured with an adversarial/occluding image
And a world ~/ardupilot_gazebo/worlds/track_demo.world placing them.

Idempotent: safe to re-run.
"""
import os
import shutil
import sys

HOME = os.path.expanduser("~")
MODELS = os.path.join(HOME, "ardupilot_gazebo", "models")
WORLDS = os.path.join(HOME, "ardupilot_gazebo", "worlds")
BUS_JPG = "/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/.venv/Lib/site-packages/ultralytics/assets/bus.jpg"
PATCH_SRC = "/mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/sim_gazebo/patch_tex.png"

CAM_SNIP = """
    <link name="camera_link">
      <pose>0.15 0 0.02 0 0 0</pose>
      <inertial><mass>0.02</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="cam_vis">
        <geometry><box><size>0.03 0.03 0.03</size></box></geometry>
      </visual>
      <sensor name="fpv_cam" type="camera">
        <update_rate>30</update_rate>
        <always_on>1</always_on>
        <visualize>1</visualize>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>300</far></clip>
        </camera>
        <plugin name="gst_cam" filename="libGstCameraPlugin.so">
          <udpHost>127.0.0.1</udpHost>
          <udpPort>5600</udpPort>
          <bitrate>3000</bitrate>
        </plugin>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>iris::base_link</parent>
      <child>camera_link</child>
    </joint>
"""


def make_cam_iris():
    src = os.path.join(MODELS, "iris_with_ardupilot")
    dst = os.path.join(MODELS, "iris_fpv_cam")
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    sdf = os.path.join(dst, "model.sdf")
    s = open(sdf).read()
    i = s.rfind("</model>")
    s = s[:i] + CAM_SNIP + "\n" + s[i:]
    s = s.replace('<model name="iris_demo">', '<model name="iris_fpv_cam">')
    # ArduPilotPlugin references the IMU by scoped name prefixed with the model
    # instance name; keep it consistent with the renamed model / world instance.
    s = s.replace("iris_demo::iris::iris/imu_link::imu_sensor",
                  "iris_fpv_cam::iris::iris/imu_link::imu_sensor")
    open(sdf, "w").write(s)
    print("iris_fpv_cam:", s.count("<link"), "links,", s.count("<sensor"), "sensors")


def billboard_model(name, texname, tex_src, width, height, matname):
    d = os.path.join(MODELS, name)
    os.makedirs(os.path.join(d, "materials", "scripts"), exist_ok=True)
    os.makedirs(os.path.join(d, "materials", "textures"), exist_ok=True)
    shutil.copy(tex_src, os.path.join(d, "materials", "textures", texname))
    with open(os.path.join(d, "materials", "scripts", "tex.material"), "w") as f:
        f.write(
            "material %s\n{\n  technique\n  {\n    pass\n    {\n"
            "      texture_unit\n      {\n        texture %s\n"
            "      }\n    }\n  }\n}\n" % (matname, texname))
    with open(os.path.join(d, "model.config"), "w") as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n'
                '  <version>1.0</version>\n  <sdf version="1.6">model.sdf</sdf>\n'
                '  <description>textured panel</description>\n</model>\n' % name)
    sdf = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="%s">
    <static>true</static>
    <link name="panel">
      <visual name="vis">
        <geometry><box><size>0.1 %.3f %.3f</size></box></geometry>
        <material>
          <script>
            <uri>model://%s/materials/scripts</uri>
            <uri>model://%s/materials/textures</uri>
            <name>%s</name>
          </script>
        </material>
      </visual>
      <collision name="col">
        <geometry><box><size>0.1 %.3f %.3f</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
""" % (name, width, height, name, name, matname, width, height)
    open(os.path.join(d, "model.sdf"), "w").write(sdf)
    print("model:", name)


def make_patch_tex():
    """Generate a high-contrast occluding/adversarial-looking texture if absent."""
    if os.path.exists(PATCH_SRC):
        return
    try:
        import numpy as np
        from PIL import Image
        rng = np.random.default_rng(0)
        a = rng.integers(0, 255, (256, 256, 3), dtype="uint8")
        # add bold blocks to make it a strong, conspicuous patch
        a[40:120, 40:200] = [255, 0, 0]
        a[150:210, 60:180] = [0, 255, 0]
        Image.fromarray(a).save(PATCH_SRC)
        print("generated patch_tex.png")
    except Exception as e:
        print("patch tex gen skipped:", e)


WORLD = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="track_demo">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- target: bus billboard, centred at ~5m height, 10m ahead (+x) -->
    <include>
      <uri>model://target_billboard</uri>
      <name>target</name>
      <pose>10 0 5 0 0 -1.5708</pose>
    </include>

    <!-- patch: parked off to the side; moved in front of target at attack time -->
    <include>
      <uri>model://patch_panel</uri>
      <name>patch</name>
      <pose>9.0 -6 3 0 0 -1.5708</pose>
    </include>

    <!-- drone with forward camera -->
    <include>
      <uri>model://iris_fpv_cam</uri>
      <name>iris_fpv_cam</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
  </world>
</sdf>
"""


def main():
    os.makedirs(WORLDS, exist_ok=True)
    make_patch_tex()
    make_cam_iris()
    if not os.path.exists(BUS_JPG):
        print("ERROR: bus.jpg not found at", BUS_JPG); sys.exit(1)
    billboard_model("target_billboard", "bus.jpg", BUS_JPG, 3.0, 4.0, "Target/Bus")
    if os.path.exists(PATCH_SRC):
        billboard_model("patch_panel", "patch.png", PATCH_SRC, 2.0, 2.5, "Patch/Tex")
    open(os.path.join(WORLDS, "track_demo.world"), "w").write(WORLD)
    print("world: track_demo.world written")


if __name__ == "__main__":
    main()
