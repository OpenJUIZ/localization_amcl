# localization_amcl

[AMCL](https://github.com/OpenJUIZ/amcl)(ROS の amcl から ROS への依存を外したライブラリ)を使って、移動ロボットのオドメトリ・LiDAR のスキャン・地図から自己位置を推定する [JUIZ](https://github.com/OpenJUIZ/juiz) のアプリです。`.juizapp` として配布し、[mobile_sim](https://github.com/OpenJUIZ/mobile_sim) と組み合わせて使います。地図は [mapper_gmapping](https://github.com/OpenJUIZ/mapper_gmapping) で作ったもの(ROS map_server の YAML + 画像)も使えます。

> 状態: 0.1 系。計画は [docs/plan.md](docs/plan.md)、進み具合は [Issues](https://github.com/OpenJUIZ/localization_amcl/issues) を見てください。

## 使い方

[Releases](https://github.com/OpenJUIZ/localization_amcl/releases) から `localization_amcl-<version>.juizapp` を、[mobile_sim の Releases](https://github.com/OpenJUIZ/mobile_sim/releases) から 0.3 系の `mobile_sim-<version>.juizapp` を取ってきます。

```sh
pip install juiz-cli==0.2.5
juiz run mobile_sim-<version>.juizapp          # 先に動かしておく(ipc://mobile_sim、http://localhost:8000/)
juiz run localization_amcl-<version>.juizapp   # 別の端末で。ブラウザで http://localhost:8002/ を開く
```

- localization_amcl は `juizapp.yaml` の `depends`(`mobile_sim >=0.3.1, <0.4`)を見て、動いている mobile_sim の版を確かめてからつなぎます。
- 入力はトピック `odom`(`Pose2DStamped`)、`scan`(`LaserScan`)、`lidar_mount`(`PoseStamped`)、`initialpose`(`Pose2DStamped`)、出力はトピック `amcl_pose`(`Pose2DStamped`、frame `map`)です。Transformations には map から見た `amcl0` の位置と、`map -> amcl_odom0`(ROS の map -> odom)を書き、mobile_sim の `odom0` につなぎます。
- 地図は既定で mobile_sim の既定の地図の写し(`assets/maps/default.yaml`)です。環境変数 `AMCL_MAP` に ROS map_server の YAML を渡すか、画面の「ファイルを読む」「mobile_sim の地図を使う」で差し替えられます。[mapper_gmapping](https://github.com/OpenJUIZ/mapper_gmapping) で保存した地図もそのまま読めます。
- 画面: 地図・粒子・推定姿勢(2σ の楕円)・スキャン・真の姿勢を描きます。「初期姿勢を指定」を押してから地図の上でドラッグすると初期姿勢を与え直し(RViz の 2D Pose Estimate と同じ)、「大域的な推定」で粒子を地図全体に撒きます。キーかパッドで mobile_sim のロボットを動かせます。
- パラメータは ROS の amcl と同じ名前です(`app.conf` の `amcl0` の項目)。

対応するターゲット: Linux(x86_64、aarch64)、macOS(Apple シリコン、Intel)、Windows(x86_64)。

### Docker

mobile_sim のイメージと一緒に動かします。

```sh
git clone --recursive https://github.com/OpenJUIZ/localization_amcl.git && cd localization_amcl
docker compose -f docker/compose.yml up
# ブラウザで http://localhost:8002/ (自己位置推定) と http://localhost:8000/ (シミュレータ) を開く
```

- イメージは `ghcr.io/openjuiz/localization_amcl`(linux/amd64 と linux/arm64)です。`:latest` はリリースの版、`:edge` は main の最新です。手元でビルドするときは `up --build` にします。
- 自分の地図は `docker/maps/` に置き、`AMCL_MAP=/maps/<名前>.yaml` を渡すか、画面で `/maps/<名前>.yaml` を読みます。
- 別の場所の mobile_sim につなぐときは、環境変数 `MOBILE_SIM_URL`(既定 `http://mobile_sim:8000`)を変えます。mobile_sim は、localization_amcl が接続してきたアドレスの 8002 番にトピックを送るので、mobile_sim からその番号に届くようにしてください。

## 構成

| パス | 中身 |
| --- | --- |
| `third_party/amcl/` | OpenJUIZ/amcl(git submodule)。静的ライブラリにしてプラグインに入れる |
| `third_party/stb/` | 地図の画像を読む [stb_image](https://github.com/nothings/stb)(public domain / MIT) |
| `src/core/` | AMCL を包む部分(JUIZ に依存しない)。地図の読み込み、粒子フィルタの更新、推定姿勢と map -> odom |
| `src/component/` | JUIZ のコンポーネント(C++ のプラグイン) |
| `tests/` | `src/core` のテスト(ctest) |
| `app/` | `juiz` で起動する conf |
| `scripts/` | juiz_sdk の取得、開発用のビルド、結合テスト、パッケージ |
| `docker/` | Dockerfile と compose.yml |

## 開発

CMake 3.16 以降と C/C++17 のコンパイラが要ります。

```sh
git clone --recursive https://github.com/OpenJUIZ/localization_amcl
cd localization_amcl
sdk="$(scripts/fetch_juiz_sdk.sh 0.2.5)"   # juiz の Releases から juiz_sdk を取って .juiz_sdk/ に展開する
cmake -B build -DCMAKE_PREFIX_PATH="$sdk"
cmake --build build
ctest --test-dir build
```

- juiz_sdk の版は、使う juiz の版と完全に一致させます(既定は 0.2.5)。
- juiz_sdk が無くても `-DLOCALIZATION_AMCL_BUILD_COMPONENT=OFF` で `src/core` とテストだけをビルドできます。
- `scripts/dev_plugin.sh` はプラグインをビルドして `app/plugins/<target>/` に置きます。`python3 scripts/smoke_test.py` で juiz から読み込めるかを確かめます(`pip install juiz-cli==0.2.5`)。
- `python3 scripts/integration_test.py --mobile-sim <mobile_sim のリポジトリ>` で mobile_sim と一緒に動かして推定の精度を確かめます(`--ui <png>` でブラウザ画面のテストも。Playwright が要る)。
- `scripts/pack.sh` で `dist/localization_amcl-<version>.juizapp` を作ります。リリースは GitHub の画面で `v<version>` の Release を作ると、`release.yml` が 5 ターゲットの `.juizapp` を添付します。

## ライセンス

localization_amcl は MIT OR Apache-2.0 です(`LICENSE-MIT`、`LICENSE-APACHE`)。

組み込んでいる OpenJUIZ/amcl は **LGPL**(ソースの見出しは LGPL-2.1 以降)です。配布するプラグインは amcl を静的にリンクしているので、amcl を差し替えたい人は、このリポジトリのソースと上の手順で作り直せます。stb_image は public domain / MIT です。
