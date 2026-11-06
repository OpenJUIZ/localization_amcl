# localization_amcl 計画書(AMCL による自己位置推定の juizapp)

2026-11-06 作成

## 目的と要件

[OpenJUIZ/amcl](https://github.com/OpenJUIZ/amcl)(ROS の navigation/amcl から ROS への依存を外したライブラリ)を JUIZ のコンポーネントで包み、**オドメトリ・LiDAR のスキャン・既知の地図**から移動ロボットの自己位置を推定するモジュールを `.juizapp` として配布する。ROS の `amcl` ノードを参考にし、まずは [mobile_sim](https://github.com/OpenJUIZ/mobile_sim) と組み合わせて動かす。地図は [mapper_gmapping](https://github.com/OpenJUIZ/mapper_gmapping) で作って保存したものも使えるようにする。

| # | 要件 | 方針 |
| --- | --- | --- |
| 1 | OpenJUIZ/amcl で自己位置推定をする | C++ のコンポーネント 1 つに、粒子フィルタ(`pf_t`)・オドメトリモデル(`AMCLOdom`)・レーザーモデル(`AMCLLaser`)を持つコンテナを置く |
| 2 | 入力はオドメトリ・LiDAR・地図 | トピック `odom`(`Pose2DStamped`)・`scan`(`LaserScan`)・`lidar_mount`(`PoseStamped`)を購読。地図は ROS map_server の YAML + 画像を conf で指定するか、プロセスで渡す |
| 3 | 出力は推定した姿勢 | ROS の `amcl_pose` と同じく推定姿勢と共分散を返し、トピック `amcl_pose` に配信。Transformations には map から見た姿勢(ContainerPose)と map -> odom の Transform を書く |
| 4 | juizapp として配布する | mapper_gmapping と同じ形(`plugins/<target>/` に C++ のプラグインと libjuiz_sdk、5 ターゲット、Releases に添付、ghcr.io の Docker イメージ) |
| 5 | 様子を見られる | ブラウザ画面に地図・粒子・推定姿勢を描き、地図上のクリックで初期姿勢を与えられる(RViz の 2D Pose Estimate 相当) |

## 調べたこと

### OpenJUIZ/amcl

- 中身は ROS navigation の amcl パッケージのライブラリ部分(`pf`、`map`、`sensors`)そのもので、ROS のノード(`amcl_node.cpp`)は入っていない。`LikelihoodFieldModelProb`(beam skip)や `pf_set_selective_resampling` があるので、比較的新しい(melodic 以降)版から取られている。
- **今の Linux(g++ 13)ではビルドできない**。`portable_utils.hpp` が `HAVE_DRAND48` を macOS でしか定義せず、glibc の `drand48` と static の宣言がぶつかる。CMake で `drand48` の有無を調べて定義する直しが要る。
- CMake は `include_directories(${CMAKE_SOURCE_DIR}/include)` など最上位のプロジェクトを前提にしていて、submodule として `add_subdirectory` すると include が見つからない。テスト(中身の無い `test_misc`)も常に作る。`target_include_directories`・`amcl::amcl` の別名・`-fPIC`・テストの切り替えを足す。
- 乱数は C の `drand48` の大域状態。コンテナを 2 つ動かすと乱数を共有するが、結果が変わるだけで壊れはしない(`srand48` で種を決められる)。スレッド安全ではないので、コンテナの中で 1 つの mutex で守る。
- ライセンスは LGPL(ソースの見出しは LGPL-2.1 以降、LICENSE ファイルは LGPL-3.0)。→ [決めてほしいこと](#決めてほしいこととリスク)
- 上記の直しは OpenJUIZ/amcl への PR にする(openslam_gmapping のときと同じ進め方)。

### ROS amcl ノードの動き(参考にする点)

- 地図を受けたら `map_t` を作り、`map_update_cspace(max_occ_dist)` で尤度場(障害物までの距離)を前もって計算する。空きセルの一覧を作り、大域的な自己位置推定(粒子を空きセル全体に撒く)に使う。
- オドメトリで `update_min_d`(0.2 m)か `update_min_a`(π/6)動いたときだけ、動作モデル → レーザーモデル → `resample_interval` 回に 1 回リサンプリング、と進める。動いていないときは何もしない(粒子が縮みすぎないように)。
- レーザーは `max_beams`(30)本に間引き、角度はロボットから見た向きに直して `AMCLLaserData` に入れる。当たらないビームは `range_max` として渡す。
- 推定姿勢は重みが最大のクラスタの平均と共分散(`pf_get_cluster_stats`)。map -> odom は「推定姿勢 × オドメトリの逆」で求め、`transform_tolerance` だけ先の時刻で配信する。
- パラメータ: `min_particles`(100)、`max_particles`(5000)、`kld_err`(0.01)、`kld_z`(0.99)、`odom_model_type`(diff)、`odom_alpha1`〜`5`(0.2)、`laser_model_type`(likelihood_field)、`laser_z_hit`(0.95)、`laser_z_rand`(0.05)、`laser_sigma_hit`(0.2)、`laser_likelihood_max_dist`(2.0)、`recovery_alpha_slow`/`fast`(0 = 使わない)、`initial_pose_x`/`y`/`a` と `initial_cov_xx`/`yy`/`aa`(0.25、0.25、(π/12)²)など。
- サービス `global_localization`(粒子を地図全体に撒き直す)と、トピック `initialpose`(RViz から初期姿勢を与える)がある。

### mobile_sim・mapper_gmapping の側

- mobile_sim(0.3.3)はトピック `odom`(`Pose2DStamped`、ノイズあり)、`scan`(`LaserScan`、当たらないビームは `range_max + 1`)、`lidar_mount`(`PoseStamped`)を 20 Hz で配信している。**mobile_sim の変更は要らない**。
- mobile_sim は地図を `get_map`(L8 の画像)と `get_map_info`(mapconf と同じ項目)で返し、地図のファイルは ROS map_server の YAML + PNG(`app/assets/maps/default.yaml`)。mapper_gmapping の `save_map` も同じ形式を書く。
- mapper_gmapping の構成(CMake の submodule + 静的リンク、`src/core` と `src/component`、`scripts/`、CI・release・Docker の各 workflow)をそのまま写せる。Transformations の使い方(`map` の Anchor、ContainerPose、`set_transform` で map -> odom、System をまたぐ Transform で mobile_sim の `odom0` につなぐ)も同じにする。

## 設計

### 全体の形

```
juiz run mobile_sim                         (kind: base, ipc://mobile_sim, http :8000)
  main_ec 20 Hz: topic "odom" / "scan" / "lidar_mount"
        │ ipc
        ▼
juiz run localization_amcl-x.y.z.juizapp    (kind: app, depends: mobile_sim, http :8002)
  amcl_set_odometry   ← subscribe "odom"
  amcl_add_scan       ← subscribe "scan"(最新のオドメトリと組にしてフィルタを進める)→ publish "amcl_pose"
  amcl_set_laser_pose ← subscribe "lidar_mount"
  amcl_set_initial_pose ← subscribe "initialpose"(画面からも呼ぶ)
  amcl_set_map / amcl_load_map / amcl_global_localization / amcl_get_pose / amcl_get_state ...
```

- HTTP は 8002(mobile_sim の 8000、mapper の 8001 とぶつけない)。
- mapper_gmapping と同じく、スキャンが来たときに手元の最新のオドメトリと組にする。

### 地図の与え方

1. **conf のファイル**(既定): コンテナの引数 `map` に map_server の YAML を書く。画像は PNG と PGM を読む(stb_image を同梱。public domain / MIT)。`negate`・`occupied_thresh`・`free_thresh`・`mode: trinary` は map_server と同じ解釈。相対パスは mobile_sim と同じく `AMCL_MAP` > `JUIZ_APP_DIR` > conf のディレクトリ > カレントディレクトリの順に探す。パッケージには mobile_sim の既定の地図の写しを `assets/maps/default.yaml` として入れる。
2. **プロセスで渡す**: `amcl_set_map(image, info)` に、mobile_sim の `get_map`・`get_map_info` や mapper の `get_map`・`get_map_info` の結果をそのまま渡せる。画面に「mobile_sim の地図を使う」「mapper の地図を使う」ボタンを置く。
3. `amcl_load_map(path)` で、mapper の `save_map` が書いたファイルを読み直せる。

ROS では地図をトピック `/map` で受けるが、juiz の画像の型と情報の dict が分かれているので、プロセスにした。

### リポジトリ構成

mapper_gmapping と同じ。

- `CMakeLists.txt`: 最上位。C++17
- `third_party/amcl/`: git submodule(OpenJUIZ/amcl、コミットを固定)。静的ライブラリ(`-fPIC`)にしてプラグインに入れる
- `third_party/stb/`: `stb_image.h`(1 ファイル)
- `src/core/`: AMCL を包む juiz 非依存の C++ ライブラリ(パラメータ、地図の読み込み、フィルタの更新、推定姿勢と共分散、map -> odom)
- `src/component/`: juiz のコンポーネント
- `tests/`: core の単体テスト(ctest)。合成した部屋の地図と、その中で作ったスキャン・ノイズ入りのオドメトリで、追従(位置追跡)と大域的な自己位置推定を確かめる
- `app/`: `juiz pack` の入力(`juizapp.yaml`、`app.conf`、`web/`、`assets/maps/`)
- `scripts/`、`docker/`、`.github/workflows/`

### コンテナとプロセス

コンテナ `amcl`(インスタンス名 `amcl0`)。引数は ROS amcl と同じ名前と既定値にする(`min_particles`、`max_particles`、`kld_err`、`kld_z`、`update_min_d`、`update_min_a`、`resample_interval`、`recovery_alpha_slow`、`recovery_alpha_fast`、`selective_resampling`、`odom_model_type`、`odom_alpha1`〜`5`、`laser_model_type`、`laser_max_beams`、`laser_min_range`、`laser_max_range`、`laser_z_hit`・`z_short`・`z_max`・`z_rand`、`laser_sigma_hit`、`laser_lambda_short`、`laser_likelihood_max_dist`、`do_beamskip` ほか、`initial_pose_x`/`y`/`a`、`initial_cov_xx`/`yy`/`aa`、`seed`)。Transformations まわりは mapper と同じ `map_frame`・`odom_frame`・`wait_laser_pose`、地図は `map`。

| プロセス | 入力 | 出力 | 使い方 |
| --- | --- | --- | --- |
| `amcl_set_odometry` | `Pose2DStamped` | なし | トピック `odom` を購読 |
| `amcl_add_scan` | `LaserScan` | フィルタを更新したか | トピック `scan` を購読。更新したら推定姿勢を `amcl_pose` に配信 |
| `amcl_set_laser_pose` | `PoseStamped` | なし | トピック `lidar_mount` を購読 |
| `amcl_set_initial_pose` | `Pose2DStamped`(+ 共分散の任意の dict) | なし | トピック `initialpose` を購読。画面のクリックから呼ぶ |
| `amcl_global_localization` | なし | なし | 粒子を地図の空きセル全体に撒き直す |
| `amcl_set_map` | 画像(L8)、info(dict) | なし | mobile_sim や mapper の地図を渡す |
| `amcl_load_map` | path | なし | map_server の YAML を読む |
| `amcl_get_pose` | なし | `Pose2DStamped`(map) | readonly |
| `amcl_get_map` / `amcl_get_map_info` | なし | 画像 / dict | readonly。画面用 |
| `amcl_get_state` | なし | 推定姿勢、共分散、粒子(間引く)、粒子数、更新回数、オドメトリ、map -> odom など | readonly。画面が数 Hz で取りに来る |

- 共分散を入れる標準の型が juiz_std に無いので、`amcl_pose` は `Pose2DStamped`(mapper の `slam_pose` と同じ)にして、共分散は `get_state` で返す。標準の型が足されたら切り替える。
- Transformations: `map`(static_anchor)から見た `amcl0` の ContainerPose を毎スキャン書き、`map -> amcl_odom0` の Transform を `set_transform` で書く。`amcl_odom0` から mobile_sim の `odom0` へ System をまたぐ Transform を張ると、mobile_sim の中で `map -> amcl_odom0 -> odom0 -> robot0 -> lidar0` がつながる(mapper と同じ形。mapper と同時には動かさない前提。同時に動かす conf は別に用意する)。
- 新しくコードを書く。ROS の amcl_node.cpp は写さず、手順とパラメータの名前・既定値を合わせる。

### ブラウザ画面

localization_amcl の HTTP ブローカー(8002)で `web/` を配信する。地図、粒子、推定姿勢と誤差楕円、スキャンを Canvas に描く。地図上のドラッグで初期姿勢(位置と向き)を与え、「大域的な自己位置推定」ボタンで撒き直す。mobile_sim の真の姿勢も重ねて誤差を出し、mobile_sim の操作(mapper の画面と同じ)も入れる。

### juizapp

- `kind: app`、`depends: { mobile_sim: ">=0.3.3, <0.4" }`、`juiz: "=0.2.4"`(mapper と同じく確かめた版に固定)。
- ターゲットは 5 つ(Linux x86_64・arm64、macOS Intel・Apple Silicon、Windows x86_64)。amcl は C と C++ の混在で、`drand48` が無い Windows 向けの代わりの実装はライブラリに既にある。

## 実装の進め方

各段階を 1 つの PR にし、GitHub の issue で管理する。

0. **L0 amcl ライブラリの直し**(OpenJUIZ/amcl への PR): `drand48` の判定、submodule として使える CMake、テストの切り替え、Windows(MSVC)と macOS のビルドを CI で確かめる。
1. **M0 雛形**: CMake、submodule、ライセンス、juiz_sdk 0.2.4 で空のプラグインを作る CI(Linux)。
2. **M1 自己位置推定の本体**(`src/core`): 地図の読み込み(YAML + PNG/PGM、画像から)、フィルタの初期化・更新・推定姿勢・map -> odom。合成した部屋で、(a) 初期姿勢を与えた追従で誤差 0.1 m 以下、(b) 初期姿勢なしの大域的推定で収束、を ctest で確かめる。
3. **M2 juiz コンポーネント**: コンテナとプロセス、`app.conf`、Transformations。結合テストは mobile_sim を動かして決まった経路を走らせ、推定姿勢と真値(mobile_sim の `get_pose`)の誤差を見る。オドメトリのノイズを大きめにして、オドメトリより良くなることを確かめる。
4. **M3 ブラウザ画面**: 地図・粒子・姿勢の表示、初期姿勢の指定、大域的推定、地図の切り替え。Playwright で確かめる。
5. **M4 juizapp 化と配布**: 5 ターゲットのビルド、`juiz pack`、`juiz run` での結合テスト、Releases への添付(GitHub の画面で Release を作る運用)。
6. **M5 Docker**: mobile_sim と一緒に動かす compose.yml と、ghcr.io への公開。

その後に、mapper で作った地図を mobile_sim に読ませて AMCL で走る一連の流れ(地図作り → 自己位置推定)のチュートリアル、経路計画・ナビゲーションとのつなぎ方を考える。

## 決めてほしいこととリスク

次の点は推奨で進め、違う場合だけ指摘してほしい。

| 決めること | 推奨 | 理由 |
| --- | --- | --- |
| amcl ライブラリの直し | OpenJUIZ/amcl に PR を出し、submodule はそのコミットに固定 | openslam_gmapping と同じ進め方。ライブラリを単体でも使えるようになる |
| amcl の取り込み方 | submodule + 静的リンク | 配布物がプラグインと libjuiz_sdk の 2 ファイルで済む |
| localization_amcl のライセンス | mobile_sim・mapper と同じ MIT OR Apache-2.0。amcl の LGPL の全文をパッケージに入れ、README と juizapp に amcl が LGPL であることと、ソースの場所を書く | 下のリスクを参照 |
| パッケージの種類 | `kind: app`、`depends: mobile_sim` | mapper と同じ。実機の基礎を使うときは、同じトピック名で配信すれば conf の subsystems で差し替えられる |
| 地図の与え方 | conf のファイル(既定は mobile_sim の既定の地図の写し)+ `amcl_set_map` | 起動してすぐ動き、mapper で作った地図にも差し替えられる |
| HTTP のポート | 8002 | mobile_sim 8000、mapper 8001 と並べる |
| 実装言語 | C++ だけ | mapper と同じ。rustc の一致の制約が無い |

既知のリスク:

- **ライセンス(LGPL)**: amcl は LGPL なので、静的リンクしたプラグインを配るときは、利用者が amcl を差し替えて作り直せるようにする必要がある(LGPL-2.1 §6、LGPL-3.0 §4)。localization_amcl のソースを全部公開し、ビルド手順を README に書くことでこれを満たせる見込み。より安全にするなら amcl だけ共有ライブラリにして `plugins/<target>/` に同梱する(配布物が 3 ファイルになる)。どちらにするかは M4 の前に確かめる。
- **Windows・macOS のビルド**: Player 由来の古い C のコード。MSVC で直しが要るかもしれない。直しは L0 の PR に入れる。
- **大域的な自己位置推定の収束**: 対称な部屋では収束しないことがある(AMCL の性質)。テストの部屋は非対称にする。
- **処理時間**: `max_particles` 5000 で 30 本のビームなら ROS でも数 ms 程度。20 Hz のスキャンでも更新は動いたときだけなので詰まらない見込み。M2 で測る。
