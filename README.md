# 로봇 기구학 (Robot Kinematics) 연습

6축 산업용 로봇의 **정기구학(FK)** 과 **역기구학(IK)** 을 웹 브라우저에서 연습하기 위한 도구입니다.
Three.js 로 로봇을 도식화해서 렌더링하고, 각 관절 좌표계의 x / y / z 축을 표시합니다.

## 실행

ES 모듈을 쓰기 때문에 `file://` 로 직접 열면 동작하지 않습니다. 로컬 서버로 띄워 주세요.

```bash
cd robot-kinematics
python3 -m http.server 8000
# 브라우저에서 http://localhost:8000
```

## 기능

| 탭 | 내용 |
|---|---|
| **DH 파라미터** | a, α, d, θ offset, 관절 한계를 표에서 직접 수정 (즉시 반영). 툴(플랜지) 변환 입력. JSON 복사/붙여넣기 |
| **정기구학** | 관절각 슬라이더, 말단 자세(위치 + 오일러각), T<sub>0</sub><sup>end</sup> 행렬, 단계별 A<sub>i</sub> · T<sub>0</sub><sup>i</sup> 행렬 |
| **역기구학** | 목표 자세 입력 또는 3D 기즈모 드래그 → 수치 IK (Damped Least Squares), 드래그 중 실시간 IK |
| **연습문제** | ① 랜덤 관절각 → 말단 위치 계산 (FK 결과는 가려짐) ② 랜덤 목표 자세 → 관절각 계산 후 채점 |
| **표시** | 좌표계별 표시/숨김과 **축 길이 배율**(원점이 겹치는 좌표계는 자동으로 1×·1.5×·2× 로 분리), 축 이름표, 관절축(z) 연장선, d/a 구간 색 구분, 링크 투명도 |

### 왼쪽 "풀이 과정" 패널

화면 오른쪽 위 `◀ 풀이` 버튼으로 열고 닫을 수 있습니다. 로봇 자세나 목표를 바꾸면 실시간으로 다시 계산됩니다.

| 모드 | 보여주는 과정 |
|---|---|
| **정기구학** | DH 표에 q 대입 → A<sub>i</sub> 일반식 → 링크별 cos/sin 값과 A<sub>i</sub>, 누적곱 T<sub>0</sub><sup>i</sup> → 말단 행렬 → 오일러각 추출식 (atan2 에 숫자 대입) |
| **IK 해석해** | 구면 손목 분리법(Pieper): 적용 조건 확인 → 툴 제거 → 손목 중심 p<sub>w</sub> → θ₁ (어깨 오프셋 고려) → θ₂·θ₃ (등가 링크 + 코사인 법칙) → R<sub>3</sub><sup>6</sup> 에서 θ₄·θ₅·θ₆ (ZYZ 꼴) → q = θ − offset → 8개 해 표, 클릭해서 적용 |
| **IK 수치해** | DLS 갱신식 → 첫 반복의 오차 e₀, 자코비안 J(q₀), Δq₀ → 전체 반복 기록 (오차 감소, 재시작) → 결과 |

해석해는 TX200, PUMA 560 처럼 구면 손목 + 평행한 관절 2·3 구조에서 동작합니다. UR5 처럼 조건을 만족하지 않으면 그 이유를 보여줍니다.
연습문제를 푸는 동안에는 풀이가 가려집니다.

- **DH 규약**: Standard (Spong/Corke) ↔ Modified (Craig) 전환. 전환 시 말단 자세가 동일하도록 파라미터를 자동 변환합니다.
- **자세 표현**: RPY (Rz·Ry·Rx), Stäubli XYZ (Rx·Ry·Rz), ZYZ 오일러각
- **프리셋**: Stäubli TX200(근사), PUMA 560, UR5

### DH 변환식

- Standard: `A_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)` — 관절 i 는 z<sub>i-1</sub> 축을 따라 회전
- Modified: `A_i = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)` — 관절 i 는 z<sub>i</sub> 축을 따라 회전
- θ<sub>i</sub> = q<sub>i</sub> + offset, 길이 mm, 각도 deg

### Stäubli TX200 (근사, Standard DH)

| i | a | α | d | θ offset |
|---|---|---|---|---|
| 1 | 250 | -90 | 642 | 0 |
| 2 | 950 | 0 | 0 | -90 |
| 3 | 0 | 90 | 0 | 90 |
| 4 | 0 | -90 | 800 | 0 |
| 5 | 0 | 90 | 0 | 0 |
| 6 | 0 | 0 | 194 | 0 |

q = 0 이면 팔이 수직으로 선 자세(Stäubli 영점과 동일한 형태)입니다. 공개된 치수를 기반으로 한 학습용 근사값입니다.

## 구조

```
index.html          UI 레이아웃
css/style.css
js/kinematics.js    DH 변환, FK, 자코비안, 수치/해석 IK, 오일러각 변환 (DOM 의존성 없음 → Node 에서 테스트 가능)
js/main.js          Three.js 렌더링과 UI
js/solution.js      왼쪽 풀이 과정 패널 (KaTeX 수식)
```

`kinematics.js` 의 함수는 Node 에서도 바로 쓸 수 있어서, 직접 짠 코드의 결과와 비교하기 좋습니다.

```js
import * as K from './js/kinematics.js';
const { robot } = K.clonePreset('TX200');
const fk = K.forward(robot, [0, -20, 110, 0, 0, 0]);
console.log(fk.Tend);
```

## 앞으로 해볼 것

- 자코비안 / 특이점(singularity) 시각화
