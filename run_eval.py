"""clean vs patched 평가 러너 (N-seed).

기록 지표:
  perception: confidence, FN율, time-to-first-detection
  decision:   상태 전이, 판단 지연
  trajectory: 경로 길이, nominal 대비 편차, 최종상태(loiter/착륙/abort/이탈), 임무시간
사용: python run_eval.py --config configs/mission.yaml [--patched]
TODO(Phase 1~3 구현 후 채움).
"""
if __name__ == "__main__":
    raise SystemExit("TODO: Phase 1 이후 구현")
