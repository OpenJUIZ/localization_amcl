# localization_amcl

[AMCL](https://github.com/OpenJUIZ/amcl)(ROS の amcl から ROS への依存を外したライブラリ)を使って、移動ロボットのオドメトリ・LiDAR のスキャン・地図から自己位置を推定する [JUIZ](https://github.com/OpenJUIZ/juiz) のアプリです。`.juizapp` として配布し、[mobile_sim](https://github.com/OpenJUIZ/mobile_sim) と組み合わせて使います。地図は [mapper_gmapping](https://github.com/OpenJUIZ/mapper_gmapping) で作ったもの(ROS map_server の YAML + 画像)も使えます。

> 状態: 作っているところです。計画は [docs/plan.md](docs/plan.md)、進み具合は [Issues](https://github.com/OpenJUIZ/localization_amcl/issues) を見てください。

## 構成

| パス | 中身 |
| --- | --- |
| `third_party/amcl/` | OpenJUIZ/amcl(git submodule)。静的ライブラリにしてプラグインに入れる |
| `third_party/stb/` | 地図の画像を読む [stb_image](https://github.com/nothings/stb)(public domain / MIT) |
| `src/core/` | AMCL を包む部分(JUIZ に依存しない)。地図の読み込み、粒子フィルタの更新、推定姿勢と map -> odom |
| `src/component/` | JUIZ のコンポーネント(C++ のプラグイン) |
| `tests/` | `src/core` のテスト(ctest) |
| `app/` | `juiz` で起動する conf |
| `scripts/` | juiz_sdk の取得、開発用のビルド、テスト |

## 開発

CMake 3.16 以降と C/C++17 のコンパイラが要ります。

```sh
git clone --recursive https://github.com/OpenJUIZ/localization_amcl
cd localization_amcl
sdk="$(scripts/fetch_juiz_sdk.sh 0.2.4)"   # juiz の Releases から juiz_sdk を取って .juiz_sdk/ に展開する
cmake -B build -DCMAKE_PREFIX_PATH="$sdk"
cmake --build build
ctest --test-dir build
```

- juiz_sdk の版は、使う juiz の版と完全に一致させます(既定は 0.2.4)。
- juiz_sdk が無くても `-DLOCALIZATION_AMCL_BUILD_COMPONENT=OFF` で `src/core` とテストだけをビルドできます。
- `scripts/dev_plugin.sh` はプラグインをビルドして `app/plugins/<target>/` に置きます。`python3 scripts/smoke_test.py` で juiz から読み込めるかを確かめます(`pip install juiz-cli==0.2.4`)。

## ライセンス

localization_amcl は MIT OR Apache-2.0 です(`LICENSE-MIT`、`LICENSE-APACHE`)。

組み込んでいる OpenJUIZ/amcl は **LGPL**(ソースの見出しは LGPL-2.1 以降)です。配布するプラグインは amcl を静的にリンクしているので、amcl を差し替えたい人は、このリポジトリのソースと上の手順で作り直せます。stb_image は public domain / MIT です。
