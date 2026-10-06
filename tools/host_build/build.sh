#!/usr/bin/env bash
# Build the Teensy firmware on a PC against mock libraries and fly one
# simulated descent. Needs g++ (C++14) and python3.
#
#   ./build.sh                                  # default SIM run
#   ./build.sh "TLM OFF;SIM -300 200 500 -3 2"  # custom commands, ';' separated
set -euo pipefail
cd "$(dirname "$0")"
SKETCH=../../firmware/Parafoil_Guided_Landing
python3 gen_prototypes.py "$SKETCH/Parafoil_Guided_Landing.ino" ino_with_protos.cpp
g++ -std=gnu++14 -O1 -Wall -Wno-unused-parameter -I mock -I "$SKETCH" -o host_ino main_host.cpp
./host_ino "${1:-TLM OFF;SIM 400 300 500 2 1}" | grep -v '^GUIDE'
