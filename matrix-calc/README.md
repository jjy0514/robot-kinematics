# matrix-calc — DH 정기구학 4x4 행렬 계산

`dh_simple.py` 맨 위의 DH 표(Standard DH: a, alpha, d, theta)를 고친 뒤 실행하면
0A1 … 5A6 과 최종 0A6 행렬을 터미널에 출력합니다. Python 3 표준 라이브러리만 사용합니다.

```bash
python3 dh_simple.py
```

- theta 열에는 관절각 θ = q + offset 을 계산해서 넣습니다.
- 변환식: ⁱ⁻¹Aᵢ = Rz(θ)·Tz(d)·Tx(a)·Rx(α)
