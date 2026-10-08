"""4x4 동차변환행렬 계산 라이브러리 (Python 표준 라이브러리만 사용)

행렬은 중첩 리스트 [[r00, r01, r02, px], ..., [0, 0, 0, 1]] 로 표현한다.
길이 단위는 자유 (웹 앱과 맞추려면 mm), 각도 입력은 deg.
"""
import math

DEG = math.pi / 180.0


# ---------------------------------------------------------------- 기본 연산
def identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def matmul(A, B):
    """C = A·B  (C[i][j] = Σ_k A[i][k]·B[k][j])"""
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def chain(mats):
    """누적 곱 [M1, M1·M2, M1·M2·M3, ...] 을 돌려준다 (정기구학의 T_0^1, T_0^2, ...)."""
    out = []
    T = identity()
    for M in mats:
        T = matmul(T, M)
        out.append(T)
    return out


def explain_element(A, B, i, j, digits=4):
    """C[i][j] 가 어떻게 계산되는지 전개한 문자열 (i, j 는 0-based)."""
    terms = [f"({A[i][k]:.{digits}f})({B[k][j]:.{digits}f})" for k in range(4)]
    value = sum(A[i][k] * B[k][j] for k in range(4))
    return f"C[{i + 1}][{j + 1}] = " + " + ".join(terms) + f" = {value:.{digits}f}"


# ---------------------------------------------------------------- 기본 변환
def rot_x(deg):
    c, s = math.cos(deg * DEG), math.sin(deg * DEG)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def rot_y(deg):
    c, s = math.cos(deg * DEG), math.sin(deg * DEG)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def rot_z(deg):
    c, s = math.cos(deg * DEG), math.sin(deg * DEG)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def trans(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


# ---------------------------------------------------------------- DH 변환
def dh_standard(a, alpha_deg, d, theta_deg):
    """Standard DH: A_i = Rz(θ_i)·Tz(d_i)·Tx(a_i)·Rx(α_i)"""
    ct, st = math.cos(theta_deg * DEG), math.sin(theta_deg * DEG)
    ca, sa = math.cos(alpha_deg * DEG), math.sin(alpha_deg * DEG)
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def dh_modified(a_prev, alpha_prev_deg, d, theta_deg):
    """Modified DH (Craig): A_i = Rx(α_{i-1})·Tx(a_{i-1})·Rz(θ_i)·Tz(d_i)"""
    ct, st = math.cos(theta_deg * DEG), math.sin(theta_deg * DEG)
    ca, sa = math.cos(alpha_prev_deg * DEG), math.sin(alpha_prev_deg * DEG)
    return [
        [ct, -st, 0.0, a_prev],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def tool_matrix(tool):
    """tool = {x, y, z, rx, ry, rz}  회전은 RPY: Rz(rz)·Ry(ry)·Rx(rx)  (웹 앱과 동일)"""
    if not tool:
        return identity()
    T = trans(tool.get("x", 0), tool.get("y", 0), tool.get("z", 0))
    R = matmul(matmul(rot_z(tool.get("rz", 0)), rot_y(tool.get("ry", 0))), rot_x(tool.get("rx", 0)))
    return matmul(T, R)


def forward(robot, q_deg):
    """정기구학. robot = {convention, rows: [{a, alpha, d, offset}], tool}
    반환: (A 리스트, 누적곱 T 리스트, T_end)"""
    dh = dh_modified if robot.get("convention") == "modified" else dh_standard
    A = [dh(r["a"], r["alpha"], r["d"], q + r.get("offset", 0)) for r, q in zip(robot["rows"], q_deg)]
    T = chain(A)
    T_end = matmul(T[-1], tool_matrix(robot.get("tool")))
    return A, T, T_end


# ---------------------------------------------------------------- 자세 추출
def rpy_zyx(T):
    """R = Rz(rz)·Ry(ry)·Rx(rx) 의 (rx, ry, rz) [deg]"""
    r = T
    cy = math.hypot(r[0][0], r[1][0])
    ry = math.atan2(-r[2][0], cy)
    if cy < 1e-9:  # 짐벌락: rz = 0
        rx = math.atan2(r[0][1], r[1][1]) if ry > 0 else math.atan2(-r[0][1], r[1][1])
        return rx / DEG, ry / DEG, 0.0
    return math.atan2(r[2][1], r[2][2]) / DEG, ry / DEG, math.atan2(r[1][0], r[0][0]) / DEG


# ---------------------------------------------------------------- 출력
def _num(v, digits):
    s = f"{v:.{digits}f}"
    return s[1:] if s.startswith("-") and float(s) == 0 else s  # -0.0000 → 0.0000


def format_matrix(M, name=None, rot_digits=4, pos_digits=3):
    """회전부는 rot_digits, 위치(4열)는 pos_digits 자리로 정렬해서 출력용 문자열을 만든다."""
    cells = [[_num(v, pos_digits if (j == 3 and i < 3) else rot_digits) if i < 3 else str(int(round(v)))
              for j, v in enumerate(row)] for i, row in enumerate(M)]
    widths = [max(len(cells[i][j]) for i in range(4)) for j in range(4)]
    lines = []
    pad = " " * (len(name) + 3) if name else ""
    for i, row in enumerate(cells):
        l, r = ("⎡", "⎤") if i == 0 else ("⎣", "⎦") if i == 3 else ("⎢", "⎥")
        body = "  ".join(c.rjust(widths[j]) for j, c in enumerate(row))
        prefix = f"{name} = " if (name and i == 1) else pad
        lines.append(f"{prefix}{l} {body} {r}")
    return "\n".join(lines)
