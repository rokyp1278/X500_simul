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
| 라이브 **비행 모션** | SITL 수평제어 불안정 → 비행 모션은 시뮬 제약(§6) | ⚠️ | — |

**핵심 한 줄:** occlusion(가림)은 드론을 멈추게 할 뿐이지만, **학습된 scale 패치는 "보이되 멀게"**
만들어 추종 드론을 **끌어당긴다** — 이것이 라이브 Gazebo 렌더까지 증명됐다.

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
  ab_measure.py / measure_single.py   라이브 박스 측정
  fly_mission.py                ★ 라이브 자율 비행 (= 젯슨 이식 대상)
scripts/                        ★ 실험 러너 (재현용, §4)
  run_scale_patch.py / run_eot_patch.py / run_eot_robust.py
  run_closed_loop.py / run_autonomous_mission.py
outputs/                        그림·GIF (결과물)
  autonomous_mission.gif        ★ 자율 시나리오 데모
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
- **폐루프:** 드론 10m→3.3m 끌려듦(standoff 59% 침해), 방어 mismatch 0→1.4(τ=0.15의 9배) 즉시 포착.
- **자율 미션:** 패치 뜨고 즉시 속임 감지 → 전진 → 안전선 1.5m에서 자동정지, **진짜 최소거리 1.49m(무접촉).**

---

## 6. 정직한 한계

1. **라이브 Gazebo 비행 모션 불안정.** `gazebo-iris` lockstep SITL이 **수평 GUIDED 속도제어를
   run마다 들쭉날쭉**(속도명령 받고도 드론이 제자리인 경우 많음; LOCAL_POSITION_NED·
   GLOBAL_POSITION_INT 수평값이 MAVLink로 거의 안 옴). **코드·공격 문제 아님** — 한 run은 실제
   접근(bh 162→271)했고, **실기(실제 FC+GPS/EKF)는 이 문제 없음.** 그래서 **전체 자율 시나리오는
   kinematic 시뮬(`src/mission/follow_mission.py`, `outputs/autonomous_mission.gif`)로 확증.**
2. **물리 인쇄 패치 미완.** 패치는 디지털/Gazebo-렌더 강건까지만. **실제 프린트+실카메라+조명**
   강건(NPS 손실, 실카메라 EOT)은 다음 작업.
3. **폐루프 "끌려듦→방어"는 측정 기반 수치/kinematic.** 라이브 비행 폐루프는 1번 때문에 미완.

---

## 7. 실기 이관 — 석사생 핸드오프

> **▶ 단계별 전체 설치 순서는 [`docs/JETSON_SETUP.md`](docs/JETSON_SETUP.md).**
> (SD카드 JetPack 플래시 → Jetson용 PyTorch/YOLO → D435i → `fly_mission.py` 포팅 → 3m 큐브 시연.)
> 아래는 요약.

### 젯슨에 올릴 것
- **`sim_gazebo/fly_mission.py`** — 카메라→YOLO→박스높이→거리추정→속도명령(MAVLink→FC).
  시뮬의 "Gazebo 카메라" 부분만 **실제 D435i 프레임**으로 바꾸면 끝. 제어 수식 동일.
- **`src/mission/follow_mission.py`** — 상태기계(이륙→탐색→추종→안전정지) 참고 로직.
- YOLO는 `yolov8n.pt` (Orin Nano에서 ~15~30fps, 추종 루프 0.3s라 충분).

### 바꿀 것 (코드 아님, 설정/하드웨어)
- 카메라: Gazebo → **D435i**
- FC: SITL → **KOA FC** (둘 다 MAVLink `tcp/serial`, 연결 문자열만 변경)
- 안전 거리: 모캡 좌표로 실제 위치 추적 → 자동 geofence (시뮬보다 **더 정확**).

### 3m 정육면체 모캡 실험 레시피
```
타깃판: 벽 앞 x=2.5m, 높이 1.5m   |   드론 이륙: x=0.3m, 높이 1.5m
추종 유지 d0 = 1.5m (드론 x=1.0m 호버)
패치 등장 → 드론 "3m?" 오판 → 전진
안전선 d_safe = 0.7m → 드론 x=1.8m 자동정지 (판과 0.7m, 무충돌)
```
- **타깃**: YOLO가 바로 아는 COCO 클래스 물체 추천(의자/모니터/사람판 등) — 따로 학습 불필요.
- **패치**: 타깃에 붙일 **실물 프린트 적대 패치**(실카메라용 재학습 필요).
- **안전**: 모캡 geofence 자동정지 + 킬스위치 + (충돌 데모 시) 폼/그물. 전 과정 **자율**(사람은 킬스위치만).

### 권장 순서
1. 젯슨에 `fly_mission.py` 포팅 → 깨끗한 타깃으로 **추종만** 먼저(패치 없이).
2. 실물 패치 학습·프린트.
3. 모캡서 저속·geofence로 "추종 → 패치 → 끌려듦 → 자동정지" 시연.

---

## 부록: 하드웨어 마운트
젯슨 Orin Nano + D435i를 X500 V2에 올리는 3D 프린트 마운트는 **`mount_design/README.md`** 참고.
무게중심(CG): 젯슨+카메라로 CG가 위·앞으로 쏠림 → **배터리 위치로 상쇄 + 손가락 밸런스 테스트 + 저속 호버 확인.**
