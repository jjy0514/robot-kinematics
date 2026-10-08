#!/usr/bin/env python3
"""6-DOF 정기구학: DH 표 + 관절각 q → A_i, T_0^i, T_0^end 를 단계별로 출력

사용 예:
  python3 fk.py --q 0 -20 110 0 0 0                    # 기본 TX200
  python3 fk.py --preset PUMA560 --q 0 45 180 0 45 0
  python3 fk.py --robot examples/tx200.json --q 10 20 30 40 50 60
  python3 fk.py --q 0 -20 110 0 0 0 --explain 6        # T_0^6 = T_0^5·A_6 의 원소 전개
  python3 fk.py --q 0 -20 110 0 0 0 --brief            # 최종 결과만

--robot 에는 웹 앱(DH 파라미터 탭)의 "JSON 복사" 결과를 그대로 저장해서 쓰면 된다.
"""
import argparse
import json
import sys

import mat4

# 웹 앱 js/kinematics.js 의 프리셋과 동일 (Standard DH, mm / deg)
PRESETS = {
    "TX200": [(250, -90, 642, 0), (950, 0, 0, -90), (0, 90, 0, 90), (0, -90, 800, 0), (0, 90, 0, 0), (0, 0, 194, 0)],
    "PUMA560": [(0, 90, 0, 0), (431.8, 0, 0, 0), (20.32, -90, 150.05, 0), (0, 90, 431.8, 0), (0, -90, 0, 0), (0, 0, 0, 0)],
    "UR5": [(0, 90, 89.159, 0), (-425, 0, 0, 0), (-392.25, 0, 0, 0), (0, 90, 109.15, 0), (0, -90, 94.65, 0), (0, 0, 82.3, 0)],
}


def load_robot(args):
    if args.robot:
        with open(args.robot, encoding="utf-8") as f:
            robot = json.load(f)
    else:
        rows = [dict(a=a, alpha=al, d=d, offset=off) for a, al, d, off in PRESETS[args.preset]]
        robot = {"convention": "standard", "rows": rows}
    robot.setdefault("convention", "standard")
    return robot


def main():
    p = argparse.ArgumentParser(description="DH 기반 6-DOF 정기구학 (4x4 행렬 곱 단계별 출력)")
    p.add_argument("--q", type=float, nargs="+", required=True, help="관절각 q1..qn (deg)")
    p.add_argument("--preset", choices=PRESETS, default="TX200", help="기본 DH 프리셋 (기본: TX200)")
    p.add_argument("--robot", help="DH 설정 JSON 파일 (웹 앱 'JSON 복사' 형식)")
    p.add_argument("--explain", type=int, metavar="i", help="T_0^i = T_0^(i-1)·A_i 의 16개 원소 계산을 전개")
    p.add_argument("--brief", action="store_true", help="중간 단계 없이 최종 결과만")
    p.add_argument("--digits", type=int, default=4, help="회전부 소수 자릿수 (기본 4)")
    args = p.parse_args()

    robot = load_robot(args)
    rows = robot["rows"]
    if len(args.q) != len(rows):
        sys.exit(f"관절각 개수({len(args.q)})가 DH 행 수({len(rows)})와 다릅니다.")

    A, T, T_end = mat4.forward(robot, args.q)
    std = robot["convention"] != "modified"
    fmt = lambda M, name: mat4.format_matrix(M, name, rot_digits=args.digits)

    print(f"규약: {'Standard DH  A_i = Rz(θ)·Tz(d)·Tx(a)·Rx(α)' if std else 'Modified DH  A_i = Rx(α_(i-1))·Tx(a_(i-1))·Rz(θ)·Tz(d)'}")
    print(f"q = {args.q} (deg)\n")

    print(f"{'i':>2} {'a':>9} {'α':>7} {'d':>9} {'offset':>7}   θ = q + offset")
    for i, (r, q) in enumerate(zip(rows, args.q), 1):
        off = r.get("offset", 0)
        print(f"{i:>2} {r['a']:>9g} {r['alpha']:>7g} {r['d']:>9g} {off:>7g}   {q:g} {'-' if off < 0 else '+'} {abs(off):g} = {q + off:g}°")
    print()

    if not args.brief:
        for i in range(len(rows)):
            k = i + 1
            print(f"── 관절 {k} " + "─" * 50)
            print(fmt(A[i], f"A{k}"))
            print()
            print(fmt(T[i], f"T0_{k}") if k > 1 else f"T0_1 = A1")
            if k > 1:
                print(f"       (= T0_{k - 1} · A{k})")
            print()

    if args.explain:
        k = args.explain
        if not 1 <= k <= len(rows):
            sys.exit(f"--explain 는 1~{len(rows)} 사이여야 합니다.")
        left = T[k - 2] if k > 1 else mat4.identity()
        print(f"── T0_{k} = {'T0_' + str(k - 1) if k > 1 else 'I'} · A{k} 원소별 전개 " + "─" * 30)
        for r in range(4):
            for c in range(4):
                print(mat4.explain_element(left, A[k - 1], r, c, digits=args.digits))
        print()

    if any(abs(robot.get("tool", {}).get(key, 0)) > 1e-12 for key in ("x", "y", "z", "rx", "ry", "rz")):
        print(fmt(T_end, "T0_tool"))
        print("       (= T0_n · T_tool)\n")

    print("══ 결과 " + "═" * 52)
    print(fmt(T_end, "T0_end"))
    x, y, z = T_end[0][3], T_end[1][3], T_end[2][3]
    rx, ry, rz = mat4.rpy_zyx(T_end)
    print(f"\n위치  p = ({x:.3f}, {y:.3f}, {z:.3f})")
    print(f"자세  RPY (Rz·Ry·Rx)  Rx = {rx:.3f}°, Ry = {ry:.3f}°, Rz = {rz:.3f}°")


if __name__ == "__main__":
    main()
