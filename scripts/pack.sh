#!/bin/sh
# app/ とプラグインから .juizapp を作る(juiz pack を使う)
# 使い方: scripts/pack.sh [<プラグインの置き場>] [<出力先>]
#   <プラグインの置き場> は <target>/<ライブラリ> の形のディレクトリ(既定 app/plugins)。
#   各ターゲットに liblocalization_amcl と libjuiz_sdk が要る(scripts/dev_plugin.sh が置く形)
#   <出力先> の既定は dist/
set -eu
cd "$(dirname "$0")/.."
plugins=$(cd "${1:-app/plugins}" && pwd)
out=${2:-dist}
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
# app/ から開発用のプラグイン・テスト用の conf を除いて写し、ライセンスを足す
(cd app && tar cf - --exclude=./plugins --exclude='./.*.conf' --exclude=./smoke_test.conf .) | (cd "$stage" && tar xf -)
mkdir -p "$stage/plugins"
cp -R "$plugins"/. "$stage/plugins/"
cp LICENSE-MIT LICENSE-APACHE "$stage/"
# amcl (LGPL) を静的リンクしているので、そのライセンスも入れる。stb_image は public domain / MIT
cp third_party/amcl/LICENSE "$stage/LICENSE-amcl"
mkdir -p "$out"
juiz pack "$stage" -o "$out"
