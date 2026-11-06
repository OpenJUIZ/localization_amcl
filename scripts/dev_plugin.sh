#!/usr/bin/env bash
# プラグインをビルドして app/plugins/<target>/ に置く(libjuiz_sdk も一緒に)。
# 使い方: scripts/dev_plugin.sh [ビルドディレクトリ (既定 build)]
set -euo pipefail
cd "$(dirname "$0")/.."

build="${1:-build}"
sdk="$(scripts/fetch_juiz_sdk.sh)"
target="$(basename "$sdk" | sed -E 's/^juiz_sdk-[^-]+-//')"
cmake -S . -B "$build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$sdk" > /dev/null
cmake --build "$build" --config Release --parallel
mkdir -p "app/plugins/$target"
cp "$build"/src/component/*localization_amcl.* "$build"/src/component/*juiz_sdk.* "app/plugins/$target/" 2>/dev/null || true
ls "app/plugins/$target"
