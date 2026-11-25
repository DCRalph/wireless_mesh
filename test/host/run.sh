#!/bin/sh
# Host tests for Direct and Mesh under ASan/UBSan. Run from anywhere: test/host/run.sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
src="$here/../../src"
out="${TMPDIR:-/tmp}/wireless_mesh_host_test"
c++ -std=c++17 -g -O1 -fsanitize=address,undefined -fno-sanitize-recover=undefined \
  -Wall -Wextra -Wno-unused-parameter -I "$here/shim" -I "$src" \
  "$here/test.cpp" "$src/Direct.cpp" "$src/Mesh.cpp" -o "$out"
"$out"
