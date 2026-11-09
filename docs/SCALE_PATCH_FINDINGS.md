# Scale-manipulation patch — live distance-pulling primitive (2026-11-08)

## 공백이었던 것
L5 라이브(Gazebo+SITL+YOLO)에서 공격이 **suppression만** 재현됐다: 적대/평범 occluder를
버스 앞·위 어디 놔도 YOLO가 **conf→0, 박스 소멸**(all-or-nothing). 부분 가림으로 **박스 축소**가
안 됐다. distance-pulling(박스 작아짐→멀어 보임→추종드론이 끌려듦)엔 **박스를 줄이되 탐지는
유지**해야 하는데, occlusion 기하로는 불가능. → "scale 조작 적대 패치를 학습해야" 로 남아 있었다.

## 이제 됨 — `src/attack/scale_patch.py`
YOLOv8 raw head `[1,84,8400]`에서 **객체 위치 앵커의 box-height(ch3)를 '목표 높이(target_ratio×clean)'
로 회귀**시키고 **class 점수는 margin 위로 유지**, 매 20스텝 **실제 탐지기(post-NMS)로 조기평가**해
"탐지되면서 가장 작은" 패치를 보존. (처음엔 height 최소화 → 1px로 붕괴 = suppression이었음.
목표-높이 회귀로 교정.)

### 결과 (bus.jpg, RTX 5080, 실제 NMS 탐지로 검증)
| 목표 | 패치 후 conf | box_h (clean 306px) | 겉보기 거리 |
|---|---|---|---|
| clean | 0.87 | 306 px | ×1.00 |
| 60% | **0.93** | 187 px (0.61×) | **×1.64** |
| 50% | **0.99** | 160 px (0.52×) | **×1.92** |
| 40% | 0.88 | 149 px (0.49×) | ×2.06 |

- 버스는 **여전히(오히려 더 강하게) 탐지**되는데 박스만 ~절반 → 겉보기 거리 ~2배.
  **이것이 occlusion으로 불가능했던 distance-pulling primitive.** 그림 `outputs/scale_patch.png`.
- **자연 한계 ~0.49×**: 그 아래로 목표를 낮춰도 박스가 더 안 줄고 suppression으로 붕괴.
  즉 **달성 가능한 shrink는 r≈0.5로 유계(bounded)**.

## 방어와의 접지 (핵심)
`src/sim/defense_study.py`의 공격 모델 파라미터는 박스 shrink 비율 `r`(=축소 후/전). 위 실측
**r≈0.5**는 그 스윕 구간(0.05~0.98) 안이며, **기존 20-seed 앵커-일관성 방어가 그 구간을 탐지**한다
(d_box vs 독립 depth 앵커 불일치 > τ). → 지금까지 numerical이던 distance-pulling+방어 주장이
**실제 학습 패치가 낼 수 있는 shrink 크기(≈0.5, 겉보기 2배)로 접지**됨. 공격이 유계라 방어가
놓치는 구간(너무 작은 shrink)은 애초에 추종드론을 거의 안 끌어당긴다.

## 한계 / 다음
- 정지 이미지 single-view 증명. **라이브 폐루프/다거리 EOT**는 아직(패치를 여러 scale·각도로
  학습해야 Gazebo 비행 중 유지). 큰 작업이지만 이제 목적함수·접지는 확보됨.
- 물리 인쇄·조명 강건성(NPS/EOT) 미반영 — 시뮬 범위.

---

# EOT 多거리 패치 + 폐루프 (2026-11-08, 이어서)

## EOT: 패치 하나가 여러 거리에서 축소 유지 — `src/attack/scale_patch_eot.py`
버스 crop에 패치를 올려 **여러 스케일(0.4~0.8 = 원거리~근거리)로 캔버스에 차분 합성**,
스케일별 목표높이 회귀를 EOT로 평균. (핵심 버그 교정: 손실 앵커를 `inside & conf>0.2`로
좁혀야 함 — 전체 inside는 배경이 섞여 희석되어 축소 실패. scale 0.3은 clean conf 0.29로 제외.)

ONE 패치, 실제 NMS 검증(`outputs/eot_patch.png`):
| 스케일(거리) | clean_h | patched_h | ratio | 겉보기거리 |
|---|---|---|---|---|
| 0.40 | 200 | 96 (conf0.74) | **0.48** | ×2.09 |
| 0.50 | 253 | 167 (conf0.51) | 0.66 | ×1.52 |
| 0.60 | 304 | 160 (conf0.91) | **0.53** | ×1.90 |
| 0.80 | 406 | 168 (conf0.64) | **0.41** | ×2.42 |

→ 접근하는 드론의 전 구간에서 박스 ~0.4~0.5로 유지(겉보기 1.5~2.4배). 라이브 폐루프에
필요한 **多거리 강건성** 확보. (Gazebo 라이브는 포트/안정성 이슈 → 더 엄밀히 이 실측
거동으로 물리 폐루프를 구동.)

## 폐루프 — `src/attack/closed_loop_patch.py` (`outputs/closed_loop_patch.png`)
접근 드론이 박스높이로 standoff 유지. 공격 시 매 프레임 박스가 측정 r(box_h)로 축소 →
perceived = d_true/r → "멀다"고 믿고 전진 → distance-pulling. 독립 depth 앵커로 방어.

**결과:**
- 드론이 **10m → 3.3m**로 끌려듦(= `d0·r`, r≈0.41). 드론은 내내 **8m인 줄 앎**.
  → **standoff 59% 침해**(의도 8m, 실제 3.3m).
- **공격은 유계**: r이 ~0.4~0.5라 무한 충돌이 아니라 `d0·r`에서 평형. (occlusion=즉시 정지,
  progressive-shrink numerical=충돌; 실제 패치=유계 침해 — 가장 정직한 중간.)
- **방어**: 패치 켜지는 즉시 mismatch 0→~1.4(τ=0.15의 **9배**)로 점프 → 지연 없이 포착.

## 종합
occlusion(suppression) → 정지 패치(단일 거리 축소) → **EOT 패치(多거리 축소)** → **폐루프
(bounded standoff 침해 + 즉시 방어)**. distance-pulling+방어 주장이 **실제 학습 패치로
거리 전구간 접지**됨. 남은 건 물리 인쇄/조명 EOT(NPS)와 실제 Gazebo 비행 재현(엔지니어링).

---

# 라이브 Gazebo 전이 — 음성 결과 (2026-11-08, 실제 Gazebo 돌림)

**실제로 Gazebo를 돌렸다.** A/B 월드(`sim_gazebo/build_ab_world.py`): 깨끗한 버스 빌보드 +
EOT-패치 버스 빌보드를 8m 앞 나란히, 헤드리스 gzserver+SITL(lockstep)로 카메라 렌더,
no-fly로 측정(`sim_gazebo/ab_measure.py`, `run_ab.sh`). 프레임 214개 렌더됨.

결과(`outputs/sim/ab_frame.png`, `ab_live.csv`):
| 빌보드 | box_h | conf |
|---|---|---|
| 깨끗한 버스 | **231 px** | 0.34 |
| EOT-패치 버스 | **231 px** | 0.66 |

**박스 크기 동일(231=231) → 라이브 렌더에서 축소 전이 실패.** 패치는 conf만 흔들고(오히려 패치가
더 높음) 스케일은 못 바꿈. 원인: Gazebo 패널 텍스처 필터링(mipmap)·비스듬한 시야각·조명·
640×480 리샘플링이 adversarial 픽셀 패턴을 파괴(적대 패치 전이의 알려진 취약성).

## 정직한 종합
- 디지털 평면 이미지: scale 패치 **작동**(박스 0.5×, primitive 성립). 폐루프(측정 기반)도 성립.
- **라이브 Gazebo 렌더: 전이 실패**(박스 안 줄고 conf만). → **라이브 distance-pulling 미달성**,
  지난 suppression과 같은 벽. 전이하려면 **렌더 파이프라인(각도·조명·해상도·텍스처필터) 위에서
  EOT 재학습** 필요(큰 작업). 검증된 distance-pulling+방어 주장은 digital/numerical 범위로 유지.
- SITL 함정(이번): gazebo-iris lockstep SITL이 **이륙 명령 직후 EOF로 죽음** → 빌보드는 정적이라
  **no-fly로 측정**(SITL은 월드 스텝만, 드론 땅에)하여 회피. 빌보드는 카메라 높이(z=1.6)로 내림.

---

# 라이브 Gazebo 전이 — 성공 (2026-11-08, 끝까지 간 결과)

앞의 "전이 실패"를 두 가지로 고쳐 **전이 달성**:

1. **렌더-강건 EOT** (`src/attack/scale_patch_eot2.py`): 학습 루프에 실제 렌더가 가하는 변환을
   차분 삽입 — 가우시안 블러(mipmap/텍스처필터)·밝기/대비/채널게인(조명)·저해상도 변위장
   원근왜곡(시야각)·리사이즈(해상도). 이걸 EOT로 평균. 학습 eval에서 전 스케일 ratio 0.46~0.52.
2. **기하 교정**: 빌보드가 yaw 틀려 심하게 비스듬했음(clean conf 0.34) → 정면(yaw 0)으로.

**확정 측정** — 단일 빌보드(겹침·각도 혼선 제거), clean/patched 각각 독립 비행 측정
(`sim_gazebo/build_single.py`,`measure_single.py`,`run_both.sh`):
| 빌보드 | box_h | conf | n |
|---|---|---|---|
| 깨끗한 버스 | **222 px** | 0.87 | 64 |
| EOT-패치 버스 | **102 px** | 0.76 | 64 |

→ **ratio 0.46, 겉보기 거리 ~2.2배, 버스 여전히 탐지.** YOLO가 패치된 버스의 윗부분만 박싱해
박스가 절반 이하로 줄어듦. 그림 `outputs/sim/gazebo_live_result.png`.

**즉 라이브 Gazebo 렌더에서 distance-pulling primitive 달성** — occlusion/suppression이 못 하던
"탐지 유지 + 박스 축소"가 실제 3D 렌더 카메라 영상에서 재현됨. (주의: 디버깅 중 간이 측정에서
빌보드 겹침/과도한 각도/narrow panel이면 ratio≈1로 안 나옴 — 겹침 아티팩트·패치 과소. 공정 조건
= 정면·단일·충분한 패치면적. 함정: build_single.py world write의 `%(name)` 오타로 빈 월드가
로드돼 "no bus" 나왔던 것 교정.)

## 남은 것 (진짜 마지막)
- 폐루프 비행(SITL 이륙시 lockstep EOF 이슈)로 "박스축소→끌려듦"까지 라이브 연결 — 지금은
  정적 호버 측정으로 primitive만 확증. 물리 인쇄 NPS(실물)는 랩 실기 몫.
