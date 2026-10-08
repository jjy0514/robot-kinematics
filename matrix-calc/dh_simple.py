import math

# ============================================================
# DH 파라미터 (Standard DH)  — 여기만 고쳐서 쓰면 됨
#   a     : 링크 길이 (mm)
#   alpha : 링크 비틀림 (deg)
#   d     : 링크 오프셋 (mm)
#   theta : 관절각 (deg)  ← θ = q + offset 을 계산해서 넣기
# ============================================================
#         a      alpha     d      theta
DH = [
    [  250,   -90,     642,     0  ],   # 1
    [  950,     0,       0,  -110  ],   # 2
    [    0,    90,       0,   200  ],   # 3
    [    0,   -90,     800,     0  ],   # 4
    [    0,    90,       0,     0  ],   # 5
    [    0,     0,     194,     0  ],   # 6
]


def dh_matrix(a, alpha, d, theta):
    """(i-1)A(i) = Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)"""
    t = math.radians(theta)
    al = math.radians(alpha)
    ct, st = math.cos(t), math.sin(t)
    ca, sa = math.cos(al), math.sin(al)
    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1],
    ]


def matmul(A, B):
    """4x4 행렬 곱 C = A · B"""
    C = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                C[i][j] += A[i][k] * B[k][j]
    return C


def print_matrix(name, M):
    print(f"{name} =")
    for row in M:
        print("  [" + "  ".join(f"{round(v, 10) + 0.0:10.4f}" for v in row) + " ]")  # -0.0000 방지
    print()


T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]  # 단위행렬

for i, (a, alpha, d, theta) in enumerate(DH):
    A = dh_matrix(a, alpha, d, theta)
    print_matrix(f"{i}A{i + 1}", A)
    T = matmul(T, A)

print("=" * 52)
print_matrix(f"0A{len(DH)}", T)
