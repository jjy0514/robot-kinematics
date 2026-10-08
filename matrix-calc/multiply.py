#!/usr/bin/env python3
"""4x4 행렬 여러 개를 순서대로 곱한다: M1·M2·…·Mn  (손계산 검산용)

입력 파일 형식 (examples/matrices.txt 참고):
  - 행렬 하나 = 숫자 4줄 (공백 또는 쉼표 구분)
  - 행렬 사이는 빈 줄
  - '#' 으로 시작하는 줄은 다음 행렬의 이름 (예: "# A1"), 없으면 M1, M2 ...
  - 숫자 대신 cos(30), sin(-90), sqrt(2)/2, pi/4 같은 식도 쓸 수 있음 (각도는 deg)

사용 예:
  python3 multiply.py examples/matrices.txt
  python3 multiply.py examples/matrices.txt --explain        # 마지막 곱의 원소별 전개
  python3 multiply.py examples/matrices.txt --explain-all    # 모든 단계의 원소별 전개
  cat my.txt | python3 multiply.py -                         # 표준입력
"""
import argparse
import math
import sys

import mat4

# 행렬 원소에 쓸 수 있는 함수 (삼각함수는 deg 입력)
SAFE = {
    "cos": lambda d: math.cos(math.radians(d)),
    "sin": lambda d: math.sin(math.radians(d)),
    "tan": lambda d: math.tan(math.radians(d)),
    "sqrt": math.sqrt,
    "pi": math.pi,
}


def parse_value(tok):
    try:
        return float(tok)
    except ValueError:
        try:
            return float(eval(tok, {"__builtins__": {}}, SAFE))  # 위 SAFE 이름만 허용
        except Exception as e:
            raise ValueError(f"숫자로 읽을 수 없음: {tok!r} ({e})")


def split_row(line):
    # 괄호 안 쉼표는 구분자로 보지 않음:  cos(30), -sin(30)  /  1 0 0 250
    toks, depth, cur = [], 0, ""
    for ch in line:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if depth == 0 and (ch.isspace() or ch == ","):
            if cur:
                toks.append(cur)
            cur = ""
        else:
            cur += ch
    if cur:
        toks.append(cur)
    return toks


def read_matrices(text):
    mats, names, rows, name = [], [], [], None
    for ln, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if line.startswith("#"):
            name = line.lstrip("#").strip() or None
            continue
        if not line:
            if rows:
                sys.exit(f"{ln}행: 행렬은 4줄이어야 합니다 (현재 {len(rows)}줄).")
            continue
        vals = [parse_value(t) for t in split_row(line)]
        if len(vals) != 4:
            sys.exit(f"{ln}행: 숫자가 4개여야 합니다 → {raw!r}")
        rows.append(vals)
        if len(rows) == 4:
            mats.append(rows)
            names.append(name or f"M{len(mats)}")
            rows, name = [], None
    if rows:
        sys.exit(f"마지막 행렬이 4줄이 아닙니다 ({len(rows)}줄).")
    return mats, names


def main():
    p = argparse.ArgumentParser(description="4x4 행렬 연쇄 곱 (단계별 출력)")
    p.add_argument("file", help="행렬 파일 경로, '-' 이면 표준입력")
    p.add_argument("--explain", action="store_true", help="마지막 곱의 16개 원소 계산을 전개")
    p.add_argument("--explain-all", action="store_true", help="모든 곱셈 단계의 원소 계산을 전개")
    p.add_argument("--digits", type=int, default=4, help="소수 자릿수 (기본 4)")
    args = p.parse_args()

    text = sys.stdin.read() if args.file == "-" else open(args.file, encoding="utf-8").read()
    mats, names = read_matrices(text)
    if not mats:
        sys.exit("행렬이 없습니다.")
    fmt = lambda M, n: mat4.format_matrix(M, n, rot_digits=args.digits, pos_digits=args.digits)

    print(f"입력 행렬 {len(mats)}개: {' · '.join(names)}\n")
    for M, n in zip(mats, names):
        print(fmt(M, n))
        print()

    acc, acc_name = mats[0], names[0]
    for k in range(1, len(mats)):
        new = mat4.matmul(acc, mats[k])
        new_name = f"{acc_name}·{names[k]}" if k == 1 else f"(…)·{names[k]}"
        label = " · ".join(names[: k + 1])
        print(f"── {label} " + "─" * max(4, 50 - len(label)))
        if args.explain_all or (args.explain and k == len(mats) - 1):
            for r in range(4):
                for c in range(4):
                    print("  " + mat4.explain_element(acc, mats[k], r, c, digits=args.digits))
            print()
        print(fmt(new, "R"))
        print()
        acc, acc_name = new, new_name

    print("══ 결과 " + "═" * 52)
    print(fmt(acc, " · ".join(names)))


if __name__ == "__main__":
    main()
