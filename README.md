# adversarial_uav_demo — 적대 패치로 자율 추종 드론 속이기 (시뮬 + 실기 데모)

**비전 기반 자율 드론**이 물체를 일정 거리로 추종하다가, 비행 중 **adversarial patch**를 만나면
**거리를 오판해 일정 거리를 유지하지 못하고 끌려드는** 공격(NDSS'26 FlyTrap 계열)과, 이를 잡는
**독립 depth 앵커 일관성 방어**를 시뮬로 검증한다. 목표: **모캡 실험실에서 전 과정 자율 비행 시연.**

> **역할 분담:** 시뮬(이 저장소 전체) = 제1저자. 젯슨 이식/실기 = 석사생(§7 핸드오프 참고).

---

## 1. 한눈에 보는 결과

| 단계 | 내용 | 상태 | 근거 |
|---|---|---|---|
| **공격 primitive** | 패치가 YOLO 박스를 **축소**(탐지는 유지) → 물체가 멀어 보임 | ✅ | `outputs/scale_patch.png` |
| **거리 강건** | 패치 하나가 **여러 거리에서** 축소 유지 (EOT) | ✅ | `outputs/eot_patch.png` |
| **라이브 렌더 전이** | **실제 Gazebo 카메라 영상**에서 박스 222→102px (0.46×) | ✅ | `outputs/sim/gazebo_live_result.png` |
| **자율 시나리오** | 이륙→추종→패치에 속아 전진→**안전선 자동정지**(무충돌) | ✅ | `outputs/autonomous_mission.gif` |
| **폐루프 + 방어** | 끌려듦을 독립 앵커가 즉시 포착(τ의 9배 마진) | ✅ | `outputs/closed_loop_patch.png`, `outputs/defense_study.png` |
| **행동 관측(실측 r)** | 패치 보면 **박지도 착륙도 안 함** → **r·d0(≈표준거리의 46%)에 park** = 안전거리 침해 | ✅ | `outputs/behavior_probe.png`, `outputs/scenario1_demo.png` |
| **Gazebo 3D 데모** | 실제 iris 메시 드론이 3m 큐브에서 **안전선(1.5m) 넘어 0.69m까지** 끌려듦 | ✅ | `outputs/sim/scenario1_gazebo.gif` / `.mp4` |
| 라이브 **비행 모션(SITL 물리)** | arducopter-gazebo lockstep 불안정 → 물리비행은 시뮬 제약(§6) | ⚠️ | — |

**핵심 한 줄:** occlusion(가림)은 드론을 멈추게 할 뿐이지만, **학습된 scale 패치는 "보이되 멀게"**
만들어 추종 드론을 **끌어당긴다** — 이것이 라이브 Gazebo 렌더까지 증명됐다.

**▶ 확정 시나리오 ① — 안전거리 침해 + 방어 포착** (실측 폐루프로 결정, `src/sim/behavior_probe.py`):
패치를 본 드론은 **충돌/착륙이 아니라** 더 가까운 평형거리 **`r·d0`(측정 r≈0.46)에 수렴해 park**한다
(d0=8m→3.3m, **59% 침해**). 독립 depth 앵커가 **패치 등장 즉시**(t=패치시점) mismatch로 포착.
실제 충돌(②)은 `r·d0`가 충돌 반경 안일 때(작은 d0 또는 더 강한 패치 r≤0.35)만 발생 — 데모는 ①로 간다.

---

## 2. 공격 → 방어 체인 (파일 + 그림)

```
occlusion(suppression)        정지 패치           EOT 多거리          렌더-강건 EOT         자율 미션
 박스 소멸=멈춤만    →   박스 0.5× 축소   →   전거리 0.5× 유지  →  라이브 Gazebo 222→102  →  이륙·추종·속음·정지
 (distance-pull 불가)    (digital)           (digital)           (실카메라 렌더)         + 방어 포착
```

- **`src/attack/scale_patch.py`** — 박스-높이 목표 회귀로 "탐지 유지 + 박스 축소"(정지 이미지).
- **`src/attack/scale_patch_eot.py`** — 여러 스케일(거리) EOT로 패치 하나가 전 거리에서 작동.
- **`src/attack/scale_patch_eot2.py`** — **렌더-강건 EOT**: 블러(텍스처필터)·조명·원근왜곡(시야각)·
  해상도 다운샘플을 학습 루프에 차분 삽입 → **실제 Gazebo 렌더로 전이**(이게 핵심 돌파).
- **`src/attack/closed_loop_patch.py`** — 측정된 축소율로 추종 드론 폐루프(끌려듦) + 방어.
- **`src/sim/defense_study.py`** — 공격강도 스윕 20-seed, 방어 ROC (기존 수치 검증).
- **`src/mission/follow_mission.py`** — **전 과정 자율 미션 로직**(이륙→탐색→추종→안전정지).
  *이 제어 로직이 그대로 젯슨 실기 코드가 된다.*

---

## 3. 저장소 구조

```
src/
  perception/detector.py        YOLO 래퍼 (카메라→탐지)
  attack/
    scale_patch.py              정지 scale 패치 (박스 축소)
    scale_patch_eot.py          EOT 多거리 패치
    scale_patch_eot2.py         ★ 렌더-강건 EOT (라이브 전이용)
    closed_loop_patch.py        측정 기반 폐루프(끌려듦)
    optimized_patch.py / patch.py   (구) suppression·occlusion 패치
  mission/follow_mission.py     ★ 자율 추종 미션 로직 (= 젯슨 이식 대상)
  sim/defense_study.py          방어 20-seed 수치 검증
sim_gazebo/                     라이브 Gazebo (WSL)
  build_*.py / run_*.sh         월드 생성 + 오케스트레이션
  build_scenario1_single.py     ★ 현실적 3m 큐브 장면(실제 iris 메시, 패치 타깃)
  run_scenario1_single.sh       Gazebo GUI로 보기 / run_scenario1_record.sh  녹화
  make_video.py / make_gif.py   녹화 프레임 -> mp4/GIF
  ab_measure.py / measure_single.py   라이브 박스 측정
  fly_mission.py                ★ 라이브 자율 비행 (= 젯슨 이식 대상)
scripts/                        ★ 실험 러너 (재현용, §4)
  run_scale_patch.py / run_eot_patch.py / run_eot_robust.py
  run_closed_loop.py / run_autonomous_mission.py
outputs/                        그림·GIF (결과물)
  autonomous_mission.gif        ★ 자율 시나리오 데모(kinematic)
  sim/scenario1_gazebo.gif/.mp4 ★ Gazebo 3D 데모(실제 iris 메시, 3m 큐브, 안전선 침해)
  sim/gazebo_live_result.png    ★ 라이브 박스 축소 증거
docs/
  SCALE_PATCH_FINDINGS.md       ★ 전체 실험 상세 기록 (정독 추천)
  HANDOFF.md / BUILD_RUNBOOK.md 환경·기존 이력
mount_design/                   젯슨+카메라 3D 프린트 마운트 (별도, README 내부)
```

---

## 4. 재현 방법

**환경:** Windows venv (`.venv`, PyTorch cu128, RTX 5080) — 디지털 실험·패치 학습.
WSL2 Ubuntu-22.04 (ArduPilot SITL + Gazebo Classic) — 라이브 Gazebo. 설치: `requirements-win-cu128.txt`.

실험 러너는 **`scripts/`** 에 있다 (Windows venv에서 실행):
```bash
.venv/Scripts/python.exe scripts/run_scale_patch.py        # 정지 scale 패치 (박스 축소) -> outputs/scale_patch.png
.venv/Scripts/python.exe scripts/run_eot_patch.py          # EOT 多거리 패치 -> outputs/eot_patch.png, eot_rows.json
.venv/Scripts/python.exe scripts/run_eot_robust.py         # ★ 렌더-강건 EOT -> sim_gazebo/bus_patched.png (Gazebo용)
.venv/Scripts/python.exe scripts/run_closed_loop.py        # 폐루프(끌려듦)+방어 -> outputs/closed_loop_patch.png
.venv/Scripts/python.exe scripts/run_autonomous_mission.py # ★ 자율 시나리오 GIF -> outputs/autonomous_mission.gif
.venv/Scripts/python.exe src/sim/defense_study.py          # 방어 20-seed 수치

# (라이브 Gazebo, WSL) — bus_patched.png 는 run_eot_robust.py 가 먼저 생성해야 함
wsl.exe -d Ubuntu-22.04 bash -lc 'bash sim_gazebo/run_both.sh'       # 단일 빌보드 clean vs patched -> outputs/sim/gazebo_live_result.png
wsl.exe -d Ubuntu-22.04 bash -lc 'bash sim_gazebo/run_fly.sh clean'  # 자율 비행 (clean|patched)
```

> 라이브 Gazebo는 **headless**(gzserver만, GUI 없음). 카메라가 `/tmp/fpv_frames`에 프레임 저장 →
> YOLO가 읽음. 포트 5760이 비어야 함(RESS 등 다른 SITL 미실행).

---

## 5. 대표 결과 수치

- **정지 패치:** bus 박스 306→160px (0.52×), conf 0.87→0.99 (탐지 더 강해지며 축소).
- **EOT 多거리:** 스케일 0.4~0.8 전 구간에서 박스 ratio 0.41~0.66 유지.
- **라이브 Gazebo (단일 빌보드, 겹침·각도 혼선 제거):**
  깨끗한 버스 222px(conf 0.87) vs **EOT-패치 버스 102px(conf 0.76)** → **0.46×, 겉보기 ~2.2배.**
- **폐루프:** 드론 8m→3.3m 끌려듦(standoff 59% 침해), 방어 mismatch 0→1.4(τ=0.15의 9배) 즉시 포착.
- **행동 관측(실측 r, src/sim/behavior_probe.py):** 패치를 보면 드론은 **박지도 착륙도 않고** 안정 평형
  **`최종거리 = r·d0`** 에 수렴해 park한다. d0=8m→**3.7m**, d0=3m→1.4m, d0=1.5m(3m큐브)→**0.69m**.
  탐지 손실(멈춤)은 다가갈수록 박스가 커져 **안 일어남**(원거리 획득 단계 제외). 충돌은 `r·d0`<충돌반경일 때만.
- **자율 미션:** 패치 뜨고 즉시 속임 감지 → 전진 → 안전선 1.5m에서 자동정지, **진짜 최소거리 1.49m(무접촉).**

---

## 6. 정직한 한계

1. **라이브 Gazebo 물리비행(SITL) 불안정.** `gazebo-iris` lockstep SITL이 **run마다 들쭉날쭉**
   (이륙/수평이동이 됐다 안 됐다; GUI를 붙이면 소프트웨어 GL이 느려 lockstep 타이밍이 깨져 모델이
   아예 안 움직임). **코드·공격 문제 아님** — **실기(실제 FC+GPS/EKF)는 이 문제 없음.** 그래서
   **Gazebo 3D 데모(`outputs/sim/scenario1_gazebo.gif`)는 SITL 물리 대신 native actor로
   실측 궤적(behavior_probe의 r≈0.46)을 재생**한다 — 드론은 **실제 iris 메시**, 타깃은 **실제
   패치 텍스처**(라이브 YOLO 222→102px로 검증된 bus_patched.png), 거리·궤적은 **실측값**.
   즉 "보이는 것"은 전부 실측 기반이고, 스크립트된 건 드론의 위치 시간열뿐(물리 적분이 아님).
   전체 자율 로직은 `src/mission/follow_mission.py`(=젯슨 코드), 재생성: `sim_gazebo/run_scenario1_single.sh`(GUI 보기) · `run_scenario1_record.sh`(녹화).
2. **물리 인쇄 패치 미완.** 패치는 디지털/Gazebo-렌더 강건까지만. **실제 프린트+실카메라+조명**
   강건(NPS 손실, 실카메라 EOT)은 다음 작업.
3. **폐루프 "끌려듦→방어"는 측정 기반 수치/kinematic.** 라이브 비행 폐루프는 1번 때문에 미완.

---

## 7. 실기 이관 — 석사생 핸드오프

> **▶ 단계별 전체 설치 순서는 [`docs/JETSON_SETUP.md`](docs/JETSON_SETUP.md).**
> (SD카드 JetPack 플래시 → Jetson용 PyTorch/YOLO → D435i → `fly_mission.py` 포팅 → 3m 큐브 시연.)
> 아래는 요약.

### 7.1 완전 자율 미션 — 사람은 킬스위치만 (이륙→스스로 탐색→추종→속음→정지)

전 과정 자율이다. 상태기계(`src/mission/follow_mission.py` = 젯슨 로직, 검증됨):

```
TAKEOFF ── 이륙 1.0m 호버
   │
SEARCH  ── 제자리 yaw 회전(스캔)으로 물체를 찾는다 (좁은 큐브에선 돌아다니지 않고 제자리 회전이 안전)
   │  └ YOLO가 타깃 클래스를 FOV 안·탐지거리(≤3m)에서 보면 →
FOLLOW  ── 박스를 화면 중앙에 두도록 yaw 정렬 + 박스 높이로 거리추정해 표준거리 d0 유지
   │  └ (패치에 속아 "멀다" 오판 → 전진 → 안전거리 침해) →
SAFETY_HALT ─ 모캡 geofence가 d_safe에서 자동정지 (무접촉). 동시에 "속임" 플래그.
```

- **지각은 단안**: 거리=박스 높이, 방위=박스의 화면상 좌우 오프셋. yaw·전진 둘 다 YOLO 박스 하나로 제어.
- **검증 결과**(`scripts/run_autonomous_mission.py`, 탑다운 데모 `outputs/autonomous_mission.gif` + 프로파일 `outputs/autonomous_profile.png`):

  | 타깃 | 자율 결과 | 최종 거리 |
  |---|---|---|
  | 깨끗 | 탐색→획득→추종, **d0 유지** | **1.50 m** |
  | 패치 | 탐색→획득→추종→**속아서 끌려듦→자동정지** | **0.70 m** (d_safe, deceived 플래그) |

- 실기 FC(ArduPilot/PX4)는 GUIDED에서 **속도+yaw_rate 명령이 안정적**이라(여기 SITL lockstep 문제 없음) 이 상태기계가 그대로 돈다. `sim_gazebo/fly_mission.py`가 **라이브 MAVLink 자율 버전**(yaw 스캔 탐색 + 박스 중심 정렬 + D0 유지 + 안전정지).

### 7.2 젯슨에 올릴 것 / 바꿀 것
- **`sim_gazebo/fly_mission.py`** — 카메라→YOLO(박스 높이+좌우 오프셋)→(vx, yaw_rate) 명령→FC(MAVLink). **딱 두 군데만** 교체(`docs/JETSON_SETUP.md`): ① 카메라 `/tmp/fpv_frames` → **D435i 실시간**, ② MAVLink `tcp:5760`(SITL) → **KOA FC**(serial/udp). 상태기계·제어 수식은 **동일**.
- **`src/mission/follow_mission.py`** — 같은 자율 로직의 2D 참고 구현(분석·시각화용).
- YOLO `yolov8n.pt` (Orin Nano ~15~30fps, 추종 루프 0.25s라 충분).
- 안전거리: 모캡 좌표로 실제 위치 추적 → 자동 geofence(시뮬보다 **더 정확**).

### 7.3 실험 물체 + adv patch 만들기

**어떤 물체?** YOLO(COCO)가 **재학습 없이** 아는 클래스여야 하고 패치가 **그 물체용**으로 학습돼 있어야 한다.
→ **가장 현실적·통제 가능: COCO 물체 사진을 폼보드에 인쇄해 스탠드에 세우기**(시뮬의 빌보드 그대로).
- 추천: **bus**(우리가 이미 패치 학습해둠) 또는 car/chair/tv. 입체 물체(진짜 의자·모니터)는 패치를 곡면에 붙여야 해 더 어렵다 → **평판 보드 권장**.
- **정상 보드** = 깨끗한 사진, **패치 보드** = 같은 사진에 adv patch 합성 인쇄. 가리개로 덮었다 열어 "패치 등장" 연출 가능(단 자율 시엔 패치가 처음부터 붙어 있어도 됨 — 드론이 획득 후 끌려듦).

> **⚠️ 물리 패치는 재학습 필요(석사생 작업).** 현재 `bus_patched.png`는 **디지털/Gazebo-렌더까지만 강건**
> (라이브 YOLO 222→102px 검증). **실물 인쇄→실카메라→실조명**에서도 먹히려면 **물리 EOT 재학습**
> (NPS 손실, 프린터 색영역·무광, 실사진 변환 루프)이 필요하다. 그냥 인쇄하면 전이가 약할 수 있으니
> **먼저 전이 테스트 → 안 되면 재학습**. (이게 §6-2의 "다음 작업".)

### 7.4 3m 큐브 레시피 + 숫자 (Gazebo 데모와 동일)
```
타깃 보드: x=2.5m, 높이 1.0m, 크기 ~0.6~0.7m   |   드론 이륙: x≈0.2m
정상(깨끗):  표준거리 d0 = 1.5m 유지        ← 초록 안전선
패치:        r≈0.46(실측) → r·d0 = 0.69m 로 끌려듦 → 안전선 54% 침해
안전정지:    geofence d_safe = 0.7m (보드와 0.7m, 무접촉)
```
변수는 `sim_gazebo/build_scenario1_single.py`(데모)·`fly_mission.py`(실기) 상단의 `TGT_X/D0/D_SAFE`로 조정.

### 7.5 안전 (필수) + 권장 순서
- **안전**: 모캡 geofence(큐브 벽·바닥) 자동정지 + 저속(≤0.5m/s) + 킬스위치. 탐색은 **제자리 yaw 회전**이라 안전. 충돌 데모를 굳이 할 땐 폼/그물.
- **순서**: ① 젯슨에 `fly_mission.py` 포팅 → **깨끗한 보드로 자율 탐색·추종만**(패치 없이) 먼저. ② 물리 패치 재학습·인쇄·전이 테스트. ③ 모캡서 저속·geofence로 **"이륙→탐색→추종→패치에 속음→자동정지"** 전 과정 자율 시연.

---

## 부록: 하드웨어 마운트
젯슨 Orin Nano + D435i를 X500 V2에 올리는 3D 프린트 마운트는 **`mount_design/README.md`** 참고.
무게중심(CG): 젯슨+카메라로 CG가 위·앞으로 쏠림 → **배터리 위치로 상쇄 + 손가락 밸런스 테스트 + 저속 호버 확인.**
