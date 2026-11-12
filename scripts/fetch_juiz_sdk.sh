#!/usr/bin/env bash
# juiz の Releases から juiz_sdk のアーカイブを取ってきて展開し、そのディレクトリを出力する。
# 使い方: scripts/fetch_juiz_sdk.sh [版 (既定 0.2.7)] [ターゲット (既定はこの機械)] [置き場所 (既定 .juiz_sdk)]
#   cmake -B build -DCMAKE_PREFIX_PATH="$(scripts/fetch_juiz_sdk.sh)"
set -euo pipefail

version="${1:-0.2.7}"
target="${2:-}"
dest="${3:-.juiz_sdk}"

if [ -z "$target" ]; then
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64) target=x86_64-unknown-linux-gnu ;;
    Linux-aarch64 | Linux-arm64) target=aarch64-unknown-linux-gnu ;;
    Darwin-x86_64) target=x86_64-apple-darwin ;;
    Darwin-arm64) target=aarch64-apple-darwin ;;
    MINGW* | MSYS* | CYGWIN*) target=x86_64-pc-windows-msvc ;;
    *) echo "unknown platform: $(uname -s)-$(uname -m)" >&2; exit 1 ;;
  esac
fi

name="juiz_sdk-${version}-${target}"
if [ "$target" = x86_64-pc-windows-msvc ]; then ext=zip; else ext=tar.gz; fi
mkdir -p "$dest"
if [ ! -d "$dest/$name" ]; then
  url="https://github.com/OpenJUIZ/juiz/releases/download/v${version}/${name}.${ext}"
  curl -fsSL -o "$dest/$name.$ext" "$url"
  curl -fsSL -o "$dest/$name.$ext.sha256" "$url.sha256"
  # .sha256 は「<ハッシュ>  <ファイル名>」の形
  (cd "$dest" && if command -v sha256sum > /dev/null; then sha256sum -c "$name.$ext.sha256"; else shasum -a 256 -c "$name.$ext.sha256"; fi) >&2
  if [ "$ext" = zip ]; then unzip -q "$dest/$name.$ext" -d "$dest"; else tar xzf "$dest/$name.$ext" -C "$dest"; fi
  rm "$dest/$name.$ext" "$dest/$name.$ext.sha256"
fi
(cd "$dest/$name" && pwd)
