# BUILD RUNBOOK — Gazebo + ArduPilot SITL + 카메라 + perception 전체 스택

목표: **Jetson에 올릴 perception+tracking 코드를 시뮬에서 실기와 동일하게 검증.**
드론이 카메라로 타깃을 추종(standoff 유지)하다가 patch 등장 → box 축소 오판 → 충돌.
그리고 방어(앵커 일관성)가 잡는지. 실기 전이: 카메라 소스/실제 Pixhawk만 교체.

> **표기:** `[SUDO]` = 비밀번호 필요(Claude가 못 함, 당신이 실행). `[PERM]` = auto-mode가
> 외부코드 빌드/실행을 막음(권한 허용 필요). `[OK]` = Claude가 이미 확인/가능.

## 환경 실측 (2026-11-05, 이 PC WSL2)
- Ubuntu 22.04.5, ArduPilot **4.6.0-beta1** `[OK]`, Gazebo Classic **11.10.2** `[OK]`
- pymavlink 2.4.49, MAVProxy, sim_vehicle.py `[OK]`, WSLg(DISPLAY=:0) `[OK]`
- `~/ardupilot_gazebo`(khancyr) 클론됨 `[OK]`, gazebo dev 헤더·cmake·g++ 있음 `[OK]`
- 없음: ardupilot_gazebo **빌드 산출물**, gstreamer, ROS2

---
## L1. ArduPilot SITL 단독 (이미 가능) `[OK]`
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --console --map
# MAVProxy: mode guided; arm throttle; takeoff 10
```

## L2. ardupilot_gazebo 플러그인 빌드 `[PERM]`(외부코드 빌드 — auto-mode가 막음)
```bash
cd ~/ardupilot_gazebo && mkdir -p build && cd build
cmake .. && make -j4          # 몇 분
# 설치는 둘 중 하나:
sudo make install             # [SUDO]  또는 아래 env 로 대체
```
env (설치 안 하면 ~/.bashrc 에):
```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:$GAZEBO_RESOURCE_PATH
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH
```

## L3. SITL ↔ Gazebo 연동 비행 `[PERM]`(Gazebo 실행) + WSLg 창
터미널 A:
```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```
터미널 B:
```bash
cd ~/ardupilot && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
# guided/arm/takeoff → Gazebo 창에서 iris 가 뜨는지 눈으로 확인 (여기까지가 1차 마일스톤)
```
> WSLg GPU 렌더가 느리면: `export LIBGL_ALWAYS_SOFTWARE=1` 로 소프트웨어 렌더 폴백.

## L4. 카메라 + 타깃 + 영상→Python `[SUDO]` + 통합(가장 까다로움)
1. gstreamer 설치 `[SUDO]`:
```bash
sudo apt update && sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav python3-opencv
```
   > **주의:** pip 의 `opencv-python` 은 GStreamer 미지원. UDP 스트림을 OpenCV 로 받으려면
   > **apt `python3-opencv`(gstreamer 지원)** 로 WSL 쪽에서 perception 을 돌리거나,
   > 대안으로 gz/gazebo 카메라 토픽을 직접 구독. (이 분기는 아침에 같이 결정)
2. 카메라 달린 모델/월드: ardupilot `SITL_Models` 또는 플러그인 예제의 `GstCameraPlugin`
   (udp host=127.0.0.1 port=5600) 을 iris 에 부착. 타깃(사람/사물 메시)과 patch 텍스처 배치.
3. Python 수신(개념):
```python
cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp ! rtph264depay ! "
                       "avdec_h264 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
```

## L5. perception + tracking + 제어 (우리 코드 = Jetson 이식 대상)
- `src/perception/detector.py` (YOLO) `[OK, 작성됨]`
- 추종 제어: box 로 d_box 추정 → standoff d0 유지하도록 pymavlink 속도 setpoint.
  (수식·동역학은 `src/sim/kinematic_tracker.py` 로 이미 검증됨 — 이걸 실제 루프로 옮김)
- patch 등장 → box 축소 → 끌려옴 → 충돌. 방어(`d_box` vs depth 앵커 일관성) 경보.
- 카메라 소스만 추상화: 시뮬=GStreamer UDP, 실기=Jetson CSI/USB 카메라. **제어·방어 코드 동일.**

## L6. (실기) — 랩 안전감독 하에서만
mocap external vision → ArduPilot EKF, kill/geofence/net, sim 검증 시나리오만.
FlyTrap(NDSS'26)도 같은 X500 으로 **자율비행 없이 손들기/벤치**로 했음 — 그 선례를 따를 것.

---
## 대안 스택 (더 깨끗하지만 큰 설치, `[SUDO]`)
ROS 2 Humble + Gazebo Harmonic + `ros_gz` + ArduPilot DDS. 카메라→토픽이 표준화돼 L4 가 쉬움.
단 RTX 5080(Blackwell)이므로 Isaac 계열은 **Isaac Sim 5.1+**만(4.5 미지원).
