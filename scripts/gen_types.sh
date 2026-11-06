#!/usr/bin/env bash
# idl/ から C++ の型のヘッダを src/component/generated/ に作り直す。juiz (pip の juiz-cli) の `juiz idl` を使う。
# 使い方: scripts/gen_types.sh [juiz のパス (既定 juiz)]
set -euo pipefail
cd "$(dirname "$0")/.."
juiz="${1:-juiz}"
out=src/component/generated
rm -rf "$out"
mkdir -p "$out"
"$juiz" idl -g cpp -o "$out" idl/localization_amcl.jidl
ls "$out"
