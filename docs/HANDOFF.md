# HANDOFF — 2026-11-05 밤 / 2026-11-06 업데이트

## 한 줄 요약
"patch → 드론이 거리를 속아 끌려가 충돌"을 **numerical 시뮬로 완전 재현 + 방어까지 정량화**
했습니다(돌아감, 그림 있음). **2026-11-06: 두 권한 벽을 풀고 L2·L3 완료** —
ardupilot_gazebo 플러그인 빌드, **Gazebo+SITL 연동 비행 검증(arm→takeoff→고도 5m+ 상승)**.
다음은 L4(카메라→Python) · L5(추종+patch+방어 루프).

## ✅ 2026-11-06 추가 완료 (Gazebo 실사 스택)
- **권한 해제:** WSL passwordless sudo(A) + `.claude/settings.local.json`에 `Bash(wsl *)`
  allow 규칙(B, BOM 제거 후 /hooks 리로드로 적용).
- **L2:** `ardupilot_gazebo` 플러그인 빌드 완료 — `~/ardupilot_gazebo/build/libArduPilotPlugin.so`,
  `libArduCopterIRLockPlugin.so`. (시스템 설치 대신 `GAZEBO_PLUGIN_PATH`로 인라인 사용 → 영구설정 안 함)
- **L3:** 헤드리스 검증 — `gzserver iris_arducopter_runway.world` + `sim_vehicle.py -f gazebo-iris
  --no-mavproxy`, pymavlink로 `tcp:127.0.0.1:5760` 접속 → GUIDED → **ARMED → takeoff →
  고도 0→5.07m 상승 확인**. Gazebo 물리↔ArduPilot 커플링 증명됨.
- 검증 스크립트: WSL `/tmp/verify_takeoff.py` (heartbeat→arm→takeoff→고도 로깅).
- GUI 창(WSLg)으로 보려면: 터미널에서 `gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world`.

## ✅ 2026-11-06 L4 완료 — 드론이 Gazebo 카메라로 타깃을 "본다"
- 내가 작성한 **GstCameraPlugin**(`sim_gazebo/gst_camera_plugin.cc`, 빌드 `~/gst_cam/build/libGstCameraPlugin.so`)을
  카메라 센서에 부착. 추가로 카메라 `<save>`로 디스크 프레임도 저장(`/tmp/fpv_frames`, 신뢰 가능 소스).
- 자산 빌더 `sim_gazebo/build_world_assets.py`: 카메라 iris + **버스 빌보드(ultralytics bus.jpg)** + 패치 패널 + `track_demo.world`.
- **헤드리스 카메라 렌더 핵심 교훈:** gzserver는 `LIBGL_ALWAYS_SOFTWARE=1`+`DISPLAY=:0`로 소프트웨어 GL(llvmpipe)일 때
  카메라 센서를 렌더함(하드웨어 WSLg GL론 프레임 0). 또 **ArduPilotPlugin lockstep이라 SITL이 붙어 월드를 스텝해야**
  카메라가 프레임을 냄.
- **검증:** 드론 카메라 프레임에 YOLO → **bus conf 0.86 + person ×4** (`outputs/sim/fpv_latest.jpg`). 실기 지각 파이프라인이 라이브 시뮬에서 작동.

## ▶ L5 (닫힌 루프) — 코드 완성, **라이브 실행은 환경 블로커로 보류**
- `sim_gazebo/l5_track_attack_defend.py`: 이륙→YOLO box→d_box→pymavlink 속도 setpoint(standoff)→patch 이동(gz)→끌림→충돌→앵커 방어 경보. **MAV=tcp:5760.**
- **정확한 블로커 (실측):**
  1. gazebo FDM은 **-I0(포트 9002/9003)**에 고정 → 작동하는 MAVLink는 **기본 5760**뿐.
  2. **-I1 등 인스턴스 변경은 FDM 포트도 옮겨 gazebo 물리가 멈춤**(프레임 동결 확인).
  3. **이 ArduPilot 빌드는 `-A "--serial0 tcp:<포트>"` 오버라이드를 적용 안 함** — arducopter가 MAVLink 소켓을 아예 안 엶(ss로 확인). 그래서 포트를 못 옮김.
  4. **RESS 실험(e0_harvest -n 4)이 5760-5793을 지속 점유** → 기본 5760을 못 씀.
- **결론:** L5 라이브는 **5760이 빌 때만** 실행 가능. 즉 ① RESS 미실행 시, 또는 ② 별도 WSL/머신에서.
  그때는 `sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --no-mavproxy --no-rebuild` (기본) + L5 그대로 실행.
- **동일 메커니즘은 numerical L5로 이미 검증**(`src/sim/defense_study.py`, 20-seed, 충돌100%/탐지100%). L4(Gazebo 지각)까지는 라이브 검증 완료.
- RESS는 제1저자 지시로 **건드리지 않음**(진행 중인 별개 실험).

## 데모 재기동 (5760이 비었을 때 L5 라이브)
```bash
# 터미널 A (WSL): gazebo (카메라 렌더에 소프트웨어 GL 필수)
cd ~/ardupilot_gazebo
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$HOME/gst_cam/build:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
source /usr/share/gazebo/setup.sh
gzserver --verbose worlds/track_demo.world      # GUI면 gazebo
# 터미널 B (WSL): SITL (5760이 비어 있어야 함)
cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --no-mavproxy --no-rebuild
# 터미널 C (WSL): 닫힌 루프
python3 /mnt/c/Users/default.DESKTOP-5PUUIEJ/Desktop/adversarial_uav_demo/sim_gazebo/l5_track_attack_defend.py
```

## ✅ 밤새 완료 (전부 돌아가고 파일로 있음)
1. 환경 셋업: Windows venv + PyTorch **cu128**(RTX 5080/sm_120 검증) + ultralytics·ART·SB3·
   pybullet·pymavlink·opencv. WSL2(ArduPilot 4.6·Gazebo11·SITL·pymavlink) 실측.
2. 지각-강건성: YOLO detector + adversarial patch → `outputs/optimized_bus.png`
   (버스 신뢰도 0.88→0.04). + occlusion baseline 비교(최적화 patch ≪ 큰 랜덤 patch 교훈).
3. **닫힌 루프 distance-pulling 재현**(NDSS'26 FlyTrap): `outputs/distance_pulling.png`
   — clean 8m 유지 vs attack 24s에 충돌, 방어가 16.6s 전 경보.
4. **방어 정량화**(20-seed): `outputs/defense_study.png` + `outputs/defense_study.md`
   - 모든 공격강도에서 충돌 100% **but 방어 탐지 100%**, lead 82.8s~6.8s
   - 충돌하려면 box/anchor mismatch ≥ 15 필수 → 어떤 실용 tau 로도 탐지(기하-속이기의 근본 한계)
   - tau=0.15: **FAR 0% / TPR 100%**
5. 문서: `README.md`(설계), `docs/BUILD_RUNBOOK.md`(전체 스택 명령), 이 파일.

> 정직한 한계: 4는 **이상화된 numerical 모델**(앵커가 신뢰 가능 가정). 실제론 잡음·
> 멀티센서 동시공격(앵커도 속이기)이 진짜 위협 → 그게 향후 연구이자 랩 강점(합의/과결정).

## ⛔ 막힌 것 (아침에 당신이 풀 것)
- **A. sudo 비밀번호** — WSL apt 설치(gstreamer 등)를 Claude가 못 함.
  풀기: WSL에서 `sudo visudo` → `<you> ALL=(ALL) NOPASSWD:ALL` (편의) 또는 필요한 apt 를 직접 실행.
- **B. 외부코드 빌드/실행 차단** — auto-mode가 `ardupilot_gazebo` 빌드·Gazebo 실행을 거부.
  풀기: Claude Code 설정에 PowerShell/WSL 실행 **permission 규칙 추가**, 또는 L2~L3를 직접 실행.

## ▶ 아침 할 일 (순서대로, BUILD_RUNBOOK.md 참조)
1. 위 A·B 권한 해제.
2. L2: `cd ~/ardupilot_gazebo/build && cmake .. && make -j4` (플러그인 빌드).
3. L3: Gazebo + `sim_vehicle.py -f gazebo-iris` → **iris 가 Gazebo 창에 뜨고 takeoff 되는지** 확인(1차 마일스톤).
4. L4: 카메라 스트림→Python(여기서 opencv-gstreamer 분기 같이 결정).
5. L5: 추종 제어+patch+방어를 실제 루프로. (동역학은 `kinematic_tracker.py`로 이미 검증됨)

## 바로 다시 돌려보려면
```
cd C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo
.\.venv\Scripts\python.exe run_tracking_scenario.py --config configs\tracking_scenario.yaml
.\.venv\Scripts\python.exe src\sim\defense_study.py
```

## 프로젝트 위치
`C:\Users\default.DESKTOP-5PUUIEJ\Desktop\adversarial_uav_demo\` (RESS 논문 repo와 분리, git 아님)
