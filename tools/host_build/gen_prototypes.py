#!/usr/bin/env python3
"""
Turn the Arduino sketch into a plain C++ file the way the Arduino IDE does:
insert a prototype for every top-level function just before the first
function definition. Building that file on a PC catches the class of error
the IDE produces ("'X' does not name a type") without needing the Teensy
toolchain.

Usage: python3 gen_prototypes.py <sketch.ino> <output.cpp>
"""
import re
import sys

FUNC_DEF = re.compile(r'^(static\s+)?[A-Za-z_][\w:<>\s\*&]*\s[\*&]?\w+\([^;]*\)\s*\{\s*$')
NOT_FUNC = re.compile(r'^(if|for|while|switch|struct|enum|namespace|class)\b')


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    src = open(sys.argv[1], encoding="utf-8").read().split("\n")
    protos, first = [], None
    for i, line in enumerate(src):
        if FUNC_DEF.match(line) and not NOT_FUNC.match(line):
            protos.append(line.rstrip(" {") + ";")
            if first is None:
                first = i
    if first is None:
        print("no function definitions found")
        return 1
    out = src[:first] + ["// ---- auto-generated prototypes (Arduino IDE style) ----"] + protos + src[first:]
    open(sys.argv[2], "w", encoding="utf-8").write("\n".join(out))
    print(f"{len(protos)} prototypes inserted before line {first + 1}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
