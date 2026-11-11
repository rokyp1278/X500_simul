# Jetson 실기 배포 현황 & 인수인계 (JETSON_STATUS)

> 작성일: 2026-11-10
> 대상: 젯슨/실기 배포를 이어받는 담당자 및 협업자
> 역할 분담: 시뮬 = 제1저자 / 젯슨·실기 배포 = 석사생. 이 문서는 **실기 쪽 현황**입니다.
> (설치 가이드는 `docs/JETSON_SETUP.md` 참고. 단, 아래 §9의 정정 사항 먼저 확인)

---

## 0. 한눈에 (TL;DR)

- 젯슨 Orin Nano에 공격 스택(PyTorch + YOLO + RealSense) **배포 완료**, 카메라·적대패치 공격 **실기 재현 성공**.
- FC(KOA FC H743)와 **MAVLink 통신 성공**. **그러나 펌웨어가 PX4 v1.16.0** — 반면 코드(`fly_mission.py`)는 **ArduPilot 전제**. ⭐ **이게 현재 가장 중요한 결정 사항.**
- **현재 블로커**
  1. 젯슨 비행 전원(배터리→젯슨) 파워 케이블 주문 중 → 실비행 대기
  2. 코드를 PX4(OFFBOARD)로 맞추는 작업 필요 (또는 FC 재플래싱 — 아래 §6 권고 참고)

---

## 1. 하드웨어 구성

| 항목 | 내용 |
|---|---|
| 기체 | X500 V2 |
| 컴패니언 컴퓨터 | Jetson Orin Nano Developer Kit |
| 카메라 | Intel RealSense D435i (USB3, 컬러+깊이) |
| FC | KOA FC H743 (STM32H743, Pixhawk급) + M10 GPS |
| 전원 경로 | 배터리 → (ESC→모터 직결) / (전원모듈→FC 5V) / (DC-DC→젯슨) |

- 비행 중 **FC ↔ 젯슨은 데이터 링크(MAVLink)만**. 전원은 각자 배터리에서 따로 받음.
- 젯슨 전원 = DC 잭(파워선). **USB-C 포트는 플래싱/복구용(device 모드)** 이라 주변장치 호스트 역할 못 함 → FC 등은 **USB-A**에 연결.
- KOA FC의 POWER 포트: 전원모듈 없이도 **배터리 전압 모니터링(최대 25V)** 가능. **전류** 측정만 외장 전원모듈 필요(우리 용도엔 전압 페일세이프로 충분).

---

## 2. 젯슨 환경 (고정 버전)

| | |
|---|---|
| JetPack | 6.2.1 |
| L4T | R36.4.x (R36.4.4 → 자동업데이트로 R36.4.7) |
| CUDA | 12.6 |
| Python | 3.10.12 |
| OS | Ubuntu 22.04.5 (jammy) |
| 사용자/호스트 | `das` / `das-desktop` |
| 저장소 | 256GB SD (약 200GB 여유) |

**패키지 고정** (`requirements-jetson.txt`):

| 패키지 | 버전 | 비고 |
|---|---|---|
| torch | 2.8.0 | 반드시 **젯슨 휠 인덱스**에서 (PyPI는 CPU/x86) |
| torchvision | 0.23.0 | 〃 |
| numpy | 1.26.4 | **반드시 <2** (젯슨 torch 휠이 numpy 1.x로 컴파일, 2.x는 크래시) |
| opencv-python | 4.11.0.86 | 5.x 회피(비행 HW 안정성) |
| ultralytics | 8.4.208 | 저장소 요구(>=8.3) 충족 |
| pymavlink | 2.4.50 | FC MAVLink |
| pyrealsense2 | 2.59.1.11053 | D435i (cp310-aarch64 휠, 소스빌드 불필요) |
| **pyserial** | **3.5** | FC 시리얼 통신에 필요 — ⚠️ requirements에 **추가 예정** |

> torch 휠 인덱스: `https://pypi.jetson-ai-lab.io/jp6/cu126` (완전 미러. `.dev` 도메인은 410 폐기 → `.io` 사용)

---

## 3. 설치/재현 순서 (+ 함정)

1. **SD 플래시** — JetPack **6.2.1 SD 이미지** (7.x는 SD 이미지 없음+ML 지원 부실, 쓰지 말 것). GNOME Disks "Restore Disk Image" 또는 balenaEtcher.
2. **부팅/초기설정** — DisplayPort(HDMI 아님). 전원 버튼 없음(파워 꽂으면 자동 부팅). WiFi는 **all-users 자동연결**로 설정(헤드리스 부팅용), SSH 켜기.
3. **pip** — user 영역 설치(`get-pip.py`, `~/.local`)로 sudo 회피. 호출은 `python3 -m pip ...` (`pip` 명령은 PATH에 없음).
4. **torch/torchvision** — `python3 -m pip install torch==2.8.0 torchvision==0.23.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126`
5. **나머지** — `requirements-jetson.txt` 설치. numpy는 1.26.4로 고정 유지.
6. **카메라** — D435i는 **USB3(5000M)** 로 잡혀야 함. USB2(480M)로 떨어지면 커넥터 재삽입(접촉 문제).
7. **검증** — `torch.cuda.is_available()==True`, YOLO로 bus 탐지(약 25fps), D435i 컬러+깊이 스트리밍.

---

## 4. 젯슨 접속

```bash
ssh das@<젯슨IP>        # SSH 키 등록돼 비번 없이 접속
```

- ⚠️ **IP는 DHCP라 재부팅 때마다 바뀜** (예: .7 → .12). 고정 IP/DHCP 예약 설정 권장.
- IP 찾기: 같은 서브넷에서 핑 스윕 후 `ssh das@...`로 hostname 확인.
  - **주의**: mDNS `das-desktop.local`이 엉뚱한 IP로 해석되는 현상 있었음(.13 오해석) → 믿지 말 것.
- **sudo는 비번 필요**(비대화형 불가) → sudo 작업은 담당자가 직접(젯슨에 접속한 세션에서 비번 입력).

---

## 5. FC 연결 & 통신 (벤치 테스트)

**연결**: FC의 **USB-C** → **USB-C↔USB-A 데이터 케이블** → 젯슨 **USB-A**.

- ⚠️ **충전전용 케이블 주의**: FC 불은 켜지는데(전원 O) 젯슨이 인식 못 하면(데이터 X) 충전전용임. **데이터 되는 케이블** 필수(폰↔PC 파일 전송되는 케이블).
- FC가 USB 전원으로 켜짐 → **배터리 뽑고 프롭 제거 후** 테스트(안전).

**소프트웨어 준비** (한 번만):
```bash
python3 -m pip install pyserial==3.5         # sudo 불필요
sudo usermod -aG dialout das                 # /dev/ttyACM0 권한, 비번 필요(직접). 새 세션부터 적용
```

**인식 확인**: `ls /dev/ttyACM0`, `lsusb | grep -i koa` → `1b8c:0036 KoaFC`

**통신 테스트 결과(2026-11-10)**: MAVLink 하트비트 수신 성공.
- autopilot = **PX4**, 기체 = QUADROTOR, armed = False
- **펌웨어 = PX4 v1.16.0**

---

## 6. ⭐ 핵심 이슈: FC는 PX4, 코드는 ArduPilot

**사실**
- 코드는 전부 ArduPilot 전제: `set_mode_apm("GUIDED")`([fly_mission.py:75]), SITL 포트 5760(ArduPilot 기본).
- 실물 FC 펌웨어 = **PX4 v1.16.0**.
- PX4 ↔ ArduPilot은 자율 제어 모드가 다름(PX4 = **OFFBOARD**, ArduPilot = GUIDED). **현 코드로는 이 FC에서 자율비행 불가.**

**중요 맥락**: 이 드론은 **지금 PX4 상태 그대로 조종기 수동비행 검증 완료**(캘리브레이션·튜닝·하드웨어 OK).

**권고 (확정은 협업자 합의)**: **PX4 유지 + 코드를 OFFBOARD로 수정**.
- 재플래싱(ArduPilot) 시 기존 캘리/튜닝 전부 소실 → 재설정·재검증·재튜닝 필요 + 위험. 잘 나는 설정을 버리는 셈.
- 코드 수정 범위는 작음: FC 제어 레이어만.
  - `set_mode_apm("GUIDED")` → PX4 **OFFBOARD** 전환
  - PX4 규칙: OFFBOARD 진입 전·중 **셋포인트 >2Hz 스트리밍** 필요(끊기면 자동 이탈). 현 루프 4Hz(DT=0.25)라 조건 충족. 순서: 셋포인트 스트리밍 시작 → OFFBOARD → arm → 이륙.
  - 시동/이륙 명령 PX4 규약으로 조정.
- **불변**: 상태기계(SEARCH→FOLLOW→SAFETY_HALT), 속도명령 메시지, **그리고 연구 핵심(YOLO·거리추정·적대패치·속임수감지)은 펌웨어 무관 → 수정 없음.**

---

## 7. 블로커 & 다음 할 일

- [ ] **FC 펌웨어 방향 확정** (협업자 합의: PX4 코드화 vs ArduPilot 재플래싱)
- [ ] 확정 시 `fly_mission.py` FC 제어부 수정 (OFFBOARD화)
- [ ] 젯슨 비행 전원(배터리→젯슨 DC-DC) 케이블 수령 → 장착
- [ ] 위치 소스 결정: 실외=GPS / 실내=mocap 또는 옵티컬플로우 (코드 지오펜스는 mocap 전제)
- [ ] 데이터 케이블 또는 TELEM 배선 상시 확보 (FC↔젯슨 비행 링크)
- [ ] 수동비행 재확인 → 자율 드라이런(FC 없이 두뇌만) → 실비행

---

## 8. 테스트 기록 (실기)

- **지각 테스트(2026-11-09)**: D435i→YOLO→거리추정 실기 작동. person 탐지, 약 9fps.
- **적대패치(2026-11-09)**: `src/attack/scale_patch.py`로 우리 YOLO 대상 패치 생성 → bus 박스 **306→153px(×0.50)** 탐지 유지(conf 0.91) = 약 2배 멀어 보임(README의 306→160px와 일치). 공격+방어(깊이 교차검증) 둘 다 확인.
  - ⚠️ `single_patched.png`는 주석 박힌 결과 figure라 거기 YOLO 돌리면 안 됨 — 패치를 새로 생성할 것.
- **FC 통신(2026-11-10)**: 위 §5 참고.

---

## 9. 파일 & 정정 필요

- `requirements-jetson.txt` — 젯슨 고정 버전(이번에 추가). **pyserial==3.5 추가 필요.**
- `sim_gazebo/fly_mission.py` — 젯슨 포팅 대상. 두 군데 교체: ①카메라 Gazebo→D435i ②MAVLink `tcp:127.0.0.1:5760`→실FC(`/dev/ttyACM0`). + §6의 PX4 OFFBOARD화.
- `docs/JETSON_SETUP.md` — ⚠️ **"JetPack 7.2.1"로 적혀 있으나 실제는 6.2.1이어야 함**(7.x는 SD 이미지 없음). 정정 필요.
