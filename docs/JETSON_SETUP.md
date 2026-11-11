# JETSON_SETUP.md — Orin Nano에 추종드론 코드 올리기 (석사생용)

X500 V2에 올린 **Jetson Orin Nano Dev Kit + RealSense D435i**에서 `fly_mission.py`를
돌려 "추종 → 패치에 속아 끌려듦 → 안전정지"를 실기로 재현하기 위한 환경 구성 순서.

> 시뮬(이 레포 전체)은 제1저자가 끝냄. 이 문서는 **젯슨 이식 = 석사생 몫**의 체크리스트.
> 실기 비행은 반드시 지도교수 감독 + geofence/킬스위치 하에서만.

---

## 0. 준비물
- Jetson Orin Nano Developer Kit (모듈 포함)
- **microSD 256GB** (UHS-1 이상) — 받아둔 것
- RealSense D435i + USB3 케이블
- KOA FC (ArduPilot/PX4) — MAVLink 연결용 (serial 또는 UDP)
- 모니터(HDMI/DP)·USB 키보드/마우스·인터넷(초기 셋업용)

---

## 1. SD카드에 JetPack 굽기

**중요 — 펌웨어 버전부터.** Orin Nano Dev Kit은 공장 펌웨어가 구버전이면 최신 JetPack SD가
**부팅 안 됨.** 펌웨어 `36.x` 이상이면 바로 1-A, 아니면 1-B 먼저.

### 1-A. 일반 경로 (펌웨어가 이미 36.x+)
1. NVIDIA에서 **Orin Nano용 JetPack SD Card Image** 다운로드 (**JetPack 6.2.1 SD 이미지**. ⚠️ 7.x는 SD 이미지가 없고 USB ISO만 제공+ML 지원 부실 → 쓰지 말 것).
   https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit
2. **balenaEtcher**로 256GB SD에 플래시 (Windows에서 바로 됨).
3. SD 꽂고 전원 → Ubuntu 초기 설정(계정·언어·네트워크).

### 1-B. 펌웨어가 구버전이라 1-A가 안 뜨면 (브리지 업데이트)
1. **JetPack 5.1.3 이미지**(`JP513-orin-nano-sd-card-image_b29.zip`) balenaEtcher로 굽고 부팅.
2. 부팅되면 **QSPI 펌웨어 업데이터 패키지** 설치 → 재부팅 (펌웨어가 36.x로 올라감).
3. 다시 **1-A의 최신 JetPack SD**로 교체 후 재부팅.

> Ubuntu x86 **네이티브** PC가 있으면 NVIDIA **SDK Manager**가 펌웨어+JetPack 한 번에 처리.
> (랩 PC는 Windows+WSL2라 USB 패스스루 문제 → SD 직접 굽는 route 권장.)

부팅 후 확인:
```bash
cat /etc/nv_tegra_release      # L4T 버전 (예: R36.x = JetPack 6.x 계열)
sudo nvpmodel -q               # 전력모드
sudo nvpmodel -m 0 && sudo jetson_clocks   # 최대 성능 (YOLO 돌릴 때)
```

---

## 2. Jetson용 PyTorch + ultralytics (YOLO)

> **일반 pip `torch` 설치 금지.** 젯슨은 ARM64+CUDA라 **Jetson 전용 PyTorch 휠**이 따로 있다.
> JetPack(=L4T) 버전에 **딱 맞는** 휠을 받아야 한다. 버전 안 맞으면 CUDA 못 잡는다.

1. L4T 버전 확인 (`cat /etc/nv_tegra_release`) → 그에 맞는 Jetson PyTorch 휠을 받는다.
   - 출처: NVIDIA Jetson 포럼 "PyTorch for Jetson" 핀 글, 또는 jetson-ai-lab의 pip 인덱스.
   - (버전이 자주 바뀌므로 여기에 URL 박지 않는다 — 설치 시점의 L4T에 맞는 걸 받을 것.)
2. 설치 후 CUDA 인식 검증:
```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# True 가 나와야 함
```
3. ultralytics 설치 (의존성 torch는 위에서 깐 Jetson 휠 유지):
```bash
pip3 install ultralytics --no-deps
pip3 install opencv-python numpy pillow pyyaml   # 런타임 의존성만
```
4. YOLO 가중치: 인터넷 되면 자동 다운로드. **오프라인이면** `yolov8n.pt`를 수동 복사.
   (이 레포엔 용량상 제외돼 있음 — 랩 PC `adversarial_uav_demo/yolov8n.pt`에서 복사.)
```bash
python3 -c "from ultralytics import YOLO; m=YOLO('yolov8n.pt'); print('ok')"
```
5. (선택, 추천) **TensorRT 엔진으로 변환**하면 Orin Nano에서 훨씬 빠름:
```bash
yolo export model=yolov8n.pt format=engine half=True   # yolov8n.engine 생성
```

---

## 3. RealSense D435i

```bash
# librealsense (Jetson용) 설치 — jetsonhacks 스크립트 또는 소스 빌드
sudo apt install ros-* 는 불필요. pyrealsense2 가 필요:
pip3 install pyrealsense2            # 안 되면 librealsense 소스에서 python 바인딩 빌드
```
확인:
```bash
python3 -c "import pyrealsense2 as rs; print([d.get_info(rs.camera_info.name) for d in rs.context().devices])"
# ['Intel RealSense D435I'] 나와야 함
```

---

## 4. fly_mission.py 포팅 (딱 두 군데만 바꾼다)

`sim_gazebo/fly_mission.py`는 **시뮬 버전**이다. 제어 수식(박스높이→거리→속도명령)은 그대로 두고
**입력 소스 2개만** 교체한다. (README §7 참고)

### ① 카메라: Gazebo 프레임 → D435i 실시간
- 바꾸기 전 (시뮬): `/tmp/fpv_frames/*.jpg` 최신 파일 읽음 (`newest()`).
- 바꾼 후 (실기): pyrealsense2 파이프라인에서 프레임 직접 받기.
```python
import pyrealsense2 as rs
pipe = rs.pipeline(); cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)
def newest():
    f = pipe.wait_for_frames().get_color_frame()
    return np.asanyarray(f.get_data()) if f else None
```
> **보너스:** D435i는 depth도 준다 → `pipe`에서 depth 프레임도 받아 **박스 중심의 실제 depth**를
> 읽으면, 레포의 "독립 depth 앵커 일관성 방어"를 **진짜 센서로** 돌릴 수 있다 (시뮬은 truth로 대체했음).
> 이게 방어의 실기 증명이 된다.

### ② FC 연결: SITL tcp:5760 → KOA FC
- 바꾸기 전: `MAV = "tcp:127.0.0.1:5760"`
- 바꾼 후: 실제 FC 연결 문자열로.
```python
MAV = "/dev/ttyACM0"        # USB 연결이면 (baud 115200 또는 921600)
# 또는 UDP 텔레메트리면: MAV = "udp:127.0.0.1:14550"
v = mavutil.mavlink_connection(MAV, baud=921600)
```
- 나머지(arm/takeoff/GUIDED 속도명령 `set_position_target_local_ned_send`)는 **동일**.
  실기 FC는 GPS/EKF가 있어 시뮬에서 불안정했던 수평 속도제어가 **정상 동작**한다.

### 거리/안전 파라미터 (3m 큐브 모캡 레시피)
`fly_mission.py` 상단 상수를 모캡 공간에 맞춰 축소:
```
TARGET_X = 2.5   # 타깃판 위치 (m)
HOVER_Z  = 1.5
D0       = 1.5   # 추종 유지 거리
D_SAFE   = 0.7   # 자동정지 거리 (타깃과 0.7m, 무충돌)
```
- 안전정지 기준(`d_true`)은 시뮬에선 적분값이었지만 **실기는 모캡 좌표**로 바꾸면 더 정확하다
  (모캡 geofence → 자동정지). 이게 1차 안전장치, 킬스위치가 2차.

---

## 5. 실기 시연 권장 순서
1. **추종만** 먼저 — 패치 없이 깨끗한 타깃(YOLO가 아는 COCO 클래스: 의자/모니터/사람판 등)으로
   D0 유지 호버·추종 확인.
2. **실물 패치 학습·프린트** — 레포 `src/attack/scale_patch_eot2.py`는 렌더강건 EOT까지.
   실카메라+조명 강건은 NPS 손실·실사진 EOT 추가 필요 (다음 작업).
3. **저속·geofence로 전체 시나리오** — "추종 → 패치 등장 → 끌려듦 → 자동정지" 자율 재현.
   사람은 킬스위치만 잡는다. 충돌 데모 시 폼/그물.

---

## 참고 (레포 내)
- 전체 흐름·결과: [`../README.md`](../README.md) (§7 = 이 핸드오프의 요약)
- 공격 실험 상세: [`SCALE_PATCH_FINDINGS.md`](SCALE_PATCH_FINDINGS.md)
- 마운트(하드웨어): [`../mount_design/README.md`](../mount_design/README.md)
