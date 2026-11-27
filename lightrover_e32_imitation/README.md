# lightrover_e32_imitation — ライトローバーE32 模倣学習サンプル

人がマウスで運転したときの **LiDARの見え方** と **操作** を記録し、
「この見え方なら、こう動く」を小さなニューラルネットに学習させて、
同じように走らせるサンプルです（Behavior Cloning）。

```
デモ収集 (人が運転)  →  学習 (PCで数分)  →  AI走行  →  人が修正 → 再学習 …
```

マップは使いません。学習するのはルートではなく「運転のしかた」なので、
**コースの配置を変えても走れる** ことがデモの見どころです。

## 仕組み

```
 人の操作 (mouse_teleop) ─> /il/human_cmd ─┐
                                            ├─ il_cmd_mux ─> /il/cmd ─> twist_limiter ─> /rover_twist ─> 実機 (または il_sim)
 il_policy (学習した方策) ─> /il/policy_cmd ┘   (人が優先)

 /scan ──┬─> il_recorder : 人が操作している瞬間の (スキャン, 操作) を ~/il_data に保存
         └─> il_policy   : スキャン → 特徴量(36方向の最小距離) → MLP → 速度指令
```

| ファイル | 役割 |
|---|---|
| `features.py` | スキャン→特徴量の変換（記録・学習・推論で共通） |
| `recorder.py` | デモ記録ノード |
| `cmd_mux.py` | 人とAIの指令の切り替え（人が操作すると即座に人が優先） |
| `policy_node.py` | 学習済みモデルでの走行ノード（numpyのみで推論） |
| `train.py` | 学習スクリプト（PyTorch、ROS不要） |
| `sim_node.py` | シミュレータノード `il_sim`（実機の代わりに `/scan` を出し `/rover_twist` で動く） |
| `courses.py`, `courses/*.yaml` | シミュレータのコース定義と LiDAR のレイキャスト |
| `sim2d.py` | ROS なしでパイプラインを試すオフライン用の簡易シミュレータ |

特徴量は「ロボット正面を0°として360°を36分割し、各方向の一番近い物体までの距離」です。
E32の `laser_frame` は `base_link` に対して -90° 回っているので（URDFより）、その分を補正しています。

## 準備

このパッケージは `lightrover_e32_ros2` リポジトリに含まれています
（`~/ros2_ws/src/lightrover_e32_ros2/lightrover_e32_imitation`）。

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select lightrover_e32_imitation
source install/setup.bash
```

記録・走行に使うパッケージ（mouse_teleop、teleop_twist_keyboard、numpy など）は
`lightrover_e32_ros2` のセットアップで導入済みのため、**追加の apt インストールは不要** です。

### 学習用の環境（PyTorch）

学習（`train.py`）にだけ PyTorch が必要です。Ubuntu 24.04 ではシステムの Python に
pip で直接インストールできない（`externally-managed-environment` エラーになる）ため、
学習用の仮想環境（venv）を作ります。`train.py` は ROS に依存しないので venv だけで動きます。

```bash
sudo apt install python3-venv          # 未導入の場合のみ
python3 -m venv ~/il_venv
~/il_venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
~/il_venv/bin/pip install numpy matplotlib
```

`--index-url ...cpu` を付けると CPU 版になります（付けないと数GBの CUDA 版が入ります。このサンプルは CPU で十分です）。
推論ノード（`il_policy`）は numpy だけで動くので、ロボットを動かす側に PyTorch は不要です。
学習を別の PC で行う場合も、同じ手順で venv を作ってデータフォルダをコピーすれば学習できます。

## デモの手順

### 0. ロボットを起動

公式ドキュメントの手順どおり、micro-ROS agent と `robot.launch.py`（`/scan` が出る状態）を起動します。
実機の代わりにシミュレータを使う場合は、後述の「[シミュレータで試す](#シミュレータで試すsim-to-real)」を見てください。
**teleop 系の launch は起動しないでください**（このサンプルが自分で起動します）。

### 1. コースを作る

机の上に箱や板で周回コースを作ります。

- **通路幅は 40cm 以上** にしてください。E32のLiDARは 12cm より近い物を測れず
  「何もない」と出力するため、壁に寄りすぎると壁が見えなくなります。
- 机の端から落ちないよう、コースの外周は壁で囲んでください。

### 2. デモを収集（5分程度）

```bash
ros2 launch lightrover_e32_imitation collect.launch.py
```

マウス操作ウィンドウでロボットを運転し、コースを **5〜10周** します。
記録されるのは「マウスで操作している瞬間」だけなので、止まって休んでも大丈夫です。
データは `~/il_data/demo_日時.npz` に15秒ごとに保存されます。

ログに `前=0.45m 左=0.20m 右=0.18m` のような表示が2秒ごとに出ます。
**最初に手でロボットの前・左・右に物を置き、該当する値が小さくなることを確認してください**（向きの確認）。

コツ:
- 一定のペースで、なるべく同じ走り方をする（毎回違う走り方をすると平均化されてしまう）
- 両回り（時計回り・反時計回り）走ると汎用性が上がる
- わざと壁に寄ってから戻る動きも少し入れると、ずれたときに戻れるようになる

### 3. 学習

「準備」で作った venv の Python で、`train.py` を直接実行します。

```bash
IL=~/ros2_ws/src/lightrover_e32_ros2/lightrover_e32_imitation/lightrover_e32_imitation
~/il_venv/bin/python $IL/train.py --data ~/il_data --out ~/il_models/model.npz --mirror --plot ~/il_models/val.png
```

CPUで1〜2分です。最後に次のような評価が出ます。

```
検証データでの平均誤差   直進 0.004 m/s   旋回 0.075 rad/s
(参考: 常に平均値を出すだけの場合  直進 0.004 m/s   旋回 0.310 rad/s)
曲がる場面で左右を当てた割合: 98%
```

「平均値を出すだけ」より明確に小さければ学習できています。`val.png` で人とAIの操作の重なり具合を確認できます。

主なオプション:

| オプション | 既定値 | 説明 |
|---|---|---|
| `--mirror` | off | 左右反転したデータを足して2倍にする |
| `--max-range` | 1.0 | これより遠い物は同一視[m]。卓上コースなら 0.5〜1.0 |
| `--sectors` | 36 | 360°の分割数 |
| `--hidden` | 64 64 | 隠れ層のサイズ |
| `--intervention-weight` | 2.0 | 手順5で人が修正したサンプルの重み |

### 4. AIで走らせる

```bash
ros2 launch lightrover_e32_imitation run_policy.launch.py
# 別ターミナルで走行開始
ros2 param set /il_policy enabled true
```

WSL などで `ros2 param set` がノードを見つけられない場合は `--no-daemon` を付けてください
（`ros2 param set --no-daemon /il_policy enabled true`）。
`run_policy.launch.py enabled:=true` で起動すると、起動直後から走り出します。

停止は `enabled false`。走行中にマウスを操作すると、その間は人の操作が優先されます。
`ros2 param set /il_policy speed_scale 0.7` で速度を落とせます。

### 5. うまくいかない場所を修正して再学習（DAgger）

AI走行中に危なくなったらマウスで割り込んで修正します。
`run_policy.launch.py` でも記録は有効なので、**割り込んだ区間だけ** が `~/il_data` に追加されます。
もう一度手順3で学習すると、苦手な場面が改善されます。

「1回目は失敗 → 失敗した場面だけ人が教える → 成功」という流れは、
フィジカルAIのデータ収集ループそのものなので、デモのストーリーとしてもおすすめです。

## シミュレータで試す（Sim-to-Real）

実機の代わりにシミュレータ（`il_sim`）を起動すると、同じ手順を PC だけで行えます。
`il_sim` は実機と**同じトピック・同じ形式**で `/scan` を出し、`/rover_twist` を受け取って動くので、
`collect.launch.py` と `run_policy.launch.py` はそのまま使えます。
シミュレータで集めたデータで学習したモデルを、そのまま実機で走らせることもできます。

```
実機         : micro-ROS agent + robot.launch.py ─┐
                                                   ├─ /scan, /rover_twist ─ collect / run_policy (共通)
シミュレータ : sim.launch.py (il_sim)  ────────────┘
```

> シミュレータと実機を同時に起動しないでください（同じトピック名なので指令やスキャンが混ざります）。
> 同じネットワークで他の人もROS 2を使っている場合は、`ROS_DOMAIN_ID` を分けてください。

### 起動

```bash
ros2 launch lightrover_e32_imitation sim.launch.py                    # 四角いコース
ros2 launch lightrover_e32_imitation sim.launch.py course:=l_shape    # L字コース
```

RViz にコース（茶色の壁）、LiDAR の点（赤）、ロボット、走行軌跡（青線）が表示されます。緑の矢印がスタート位置です。

| コース | 内容 |
|---|---|
| `square` | 四角いドーナツ形（外側1.3m四方、通路幅0.4m）。曲がり角はすべて同じ向き |
| `square_wide` | 通路幅0.6mの四角いコース。学習していない通路幅で走れるかの確認用 |
| `l_shape` | L字形。反時計回りに走ると左折5回と右折1回が入る |
| `obstacles` | 通路幅0.6mのコースに、箱を左右交互に置いたもの |

自作のコースは `courses/` の YAML をコピーして壁の座標を書き換え、`course:=~/my_course.yaml` のようにパスで指定できます。

起動中の操作:

```bash
ros2 param set /il_sim course l_shape                 # コースを切り替える (スタート位置に戻る)
ros2 service call /il_sim/reset std_srvs/srv/Empty    # スタート位置に戻す・統計をリセット
```

### デモ収集 → 学習 → AI 走行

シミュレータを起動したまま、別のターミナルで実機と同じ手順を行います。
実機のデータと混ざらないよう、保存先を `~/il_data_sim` に分けておきます。

```bash
# デモ収集: マウスでシミュレータ上のロボットを運転する
ros2 launch lightrover_e32_imitation collect.launch.py data_dir:=~/il_data_sim

# 学習
IL=~/ros2_ws/src/lightrover_e32_ros2/lightrover_e32_imitation/lightrover_e32_imitation
~/il_venv/bin/python $IL/train.py --data ~/il_data_sim --out ~/il_models/model_sim.npz --mirror

# AI 走行 (割り込んで修正した分は ~/il_data_sim に追加される)
ros2 launch lightrover_e32_imitation run_policy.launch.py data_dir:=~/il_data_sim \
    model_path:=~/il_models/model_sim.npz enabled:=true
```

シミュレータ側のターミナルには、周回するたびに `1 周 (52 秒, 衝突 0 回)` のように表示され、
10秒ごとに周回数・衝突回数・走行距離が出ます。モデルの良し悪しを数字で比べられます。

### 実機で走らせる（Sim-to-Real）

シミュレータを止め、実機を起動してから（手順0）、シミュレータで学習したモデルを指定します。

```bash
ros2 launch lightrover_e32_imitation run_policy.launch.py model_path:=~/il_models/model_sim.npz
ros2 param set /il_policy speed_scale 0.7     # 最初は遅めで
ros2 param set /il_policy enabled true
```

学習しているのはルートではなく「LiDARの見え方に対する運転のしかた」なので、
実機のコースがシミュレータのコースと同じ形である必要はありません（通路幅は近いほうが有利です）。

実機でうまくいかない場面では、マウスで割り込んで修正します。修正分は `~/il_data` に記録されるので、
シミュレータのデータと合わせて学習し直します。

```bash
~/il_venv/bin/python $IL/train.py --data ~/il_data_sim ~/il_data --out ~/il_models/model_mix.npz --mirror
```

「シミュレータで大量に学び、実機の少しのデータで仕上げる」という、フィジカルAIの実務と同じ流れです。

### 現実らしさの設定

シミュレータは、実機との差になりやすい要素を設定で変えられます。
既定（`realism:=real`）は実機に近づけた値、`realism:=ideal` はノイズも遅れもない理想的な設定です。

```bash
ros2 launch lightrover_e32_imitation sim.launch.py realism:=ideal
```

| パラメータ | real | ideal | 内容 |
|---|---|---|---|
| `range_noise` | 0.01 | 0 | LiDAR の距離ノイズ（標準偏差[m]） |
| `dropout` | 0.02 | 0 | LiDAR の計測抜けの割合 |
| `range_min` | 0.12 | 0 | これより近い物は測れない[m]（E32 の LiDAR は 0.12） |
| `cmd_delay` | 0.10 | 0 | 指令が届いて動き出すまでの遅れ[s] |
| `linear_gain` | 1.0 | 1.0 | 指令に対する実際の速度の比（電池が減ると 1 より小さくなるイメージ） |
| `angular_gain` | 1.0 | 1.0 | 指令に対する実際の旋回の比（床の滑りなど） |
| `motion_noise` | 0.05 | 0 | 動きのばらつき（指令に対する相対値） |

real の値は目安です。実機の様子を見て `config/sim_real.yaml` を調整してください。
起動中にも変更できます（小数で指定してください。`0` ではなく `0.0`）。

```bash
ros2 param set /il_sim cmd_delay 0.3
ros2 param set /il_sim linear_gain 0.7
```

## ROS なしで試す（オフライン）

ROS も実機も不要で、venv だけで動きます。

```bash
IL=~/ros2_ws/src/lightrover_e32_ros2/lightrover_e32_imitation/lightrover_e32_imitation
~/il_venv/bin/python $IL/sim2d.py record --out ~/il_data_sim/demo_sim.npz
~/il_venv/bin/python $IL/train.py --data ~/il_data_sim --out ~/il_models/sim_model.npz --mirror
~/il_venv/bin/python $IL/sim2d.py eval --model ~/il_models/sim_model.npz --plot ~/il_models/sim_run.png
```

四角いドーナツ形のコースを「お手本コントローラ」が走ったデータで学習し、学習した方策で周回させます。
物理は単純化しているので、ここで動いても実機で動くとは限りません（その差を体験する教材にもなります）。

## 教材として扱えるテーマ

シミュレータを使うと、実機では時間のかかる比較を手軽に行えます。

- **Sim-to-Real**: シミュレータだけで学習したモデルを実機で走らせ、どこで失敗するかを観察する。実機の修正データを少し足すとどう変わるか
- **現実らしさの影響**: `realism:=ideal` で集めたデータのモデルを、`realism:=real` や実機で走らせると？ `cmd_delay` や `linear_gain` を変えるとどうなるか
- **汎化**: `square` だけで学習したモデルを `l_shape` や `obstacles` で走らせる。動作確認では、同じ手順でも集めたデータによって、走り切れるモデルと L字の角で止まってしまうモデルの両方がありました。どんなデータを足せば安定するかを考える題材になります

- **データの量と質**: 周回数を 2 / 5 / 10 周と変えて比較する
- **前処理**: `--sectors` や `--max-range` を変えて挙動の違いを見る（生データで保存しているので再収集不要）
- **データ拡張**: `--mirror` の有無で、片回りしか教えていないのに逆回りできるか試す
- **行動の多峰性**: コース中央に障害物を置き、左右半々で避けるデモを取ると、平均をとって正面衝突しやすくなる。なぜか、どう解決するか（デモを揃える、ACTや拡散ポリシーなど）を考える
- **DAgger**: 修正データの有無で成功率を比較する

## 既知の制限

- シミュレータの動きは「指令した速度で進む」だけの簡略版です（壁にぶつかるとその場で止まり、壁から離れる方向と旋回だけできます）。慣性やタイヤの滑りは `motion_noise` などで大まかに表しているだけです。
- シミュレータの LiDAR は壁までの距離にノイズを足しただけで、黒い物や光沢のある物が測りにくいといった実機の LiDAR の癖は再現していません。

- LiDARは 12cm 未満を測れず、その方向は「遠い」と同じ扱いになります。通路は広めに作ってください。
- 前方 ±30° に 15cm 未満の物があると前進しない安全装置を入れていますが、上の理由で完全ではありません。卓上で運用し、必ず人が見ていてください。
- 行動は「前進速度・旋回速度」の2つで、後退するデモは想定していません。

## ライセンス

[Apache License 2.0](LICENSE) で公開しています。
