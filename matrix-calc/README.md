# matrix-calc — 4x4 행렬 곱 / 6-DOF 정기구학 계산기

Python 3 표준 라이브러리만 사용합니다 (numpy 불필요). 웹 앱과 같은 DH 규약·단위(mm, deg)를 씁니다.

```
mat4.py       라이브러리: matmul, chain(누적곱), dh_standard/dh_modified, forward, 출력 포맷
fk.py         DH 표 + q → A_i, T_0^i, T_0^end 단계별 출력
multiply.py   직접 적은 4x4 행렬들을 순서대로 곱하기 (손계산 검산)
examples/     tx200.json (웹 앱 JSON 형식), matrices.txt
```

## fk.py — 정기구학

```bash
python3 fk.py --q 0 -20 110 0 0 0                       # 기본: TX200
python3 fk.py --preset PUMA560 --q 0 45 180 0 45 0      # TX200 / PUMA560 / UR5
python3 fk.py --robot examples/tx200.json --q 30 -20 110 0 45 0
python3 fk.py --q 30 -20 110 0 45 0 --explain 2         # T0_2 = T0_1·A2 의 원소별 전개
python3 fk.py --q 30 -20 110 0 45 0 --brief             # 최종 결과만
```

웹 앱 **DH 파라미터 → JSON 복사** 결과를 파일로 저장하면 `--robot` 으로 그대로 읽습니다 (Modified DH, 툴 변환 포함).

## multiply.py — 행렬 직접 입력해서 곱하기

```bash
python3 multiply.py examples/matrices.txt              # A1 · A2
python3 multiply.py examples/matrices.txt --explain    # 마지막 곱의 16개 원소 전개
```

입력 형식: 행렬 하나 = 4줄, 행렬 사이 빈 줄, `# 이름` 으로 이름 지정.
원소에는 `cos(30)`, `-sin(90)`, `250*cos(30)`, `sqrt(2)/2` 같은 식을 쓸 수 있습니다 (각도는 deg).

## 코드에서 쓰기

```python
import mat4
A1 = mat4.dh_standard(a=250, alpha_deg=-90, d=642, theta_deg=30)
A2 = mat4.dh_standard(950, 0, 0, -110)
T02 = mat4.matmul(A1, A2)
print(mat4.format_matrix(T02, "T0_2"))
```
