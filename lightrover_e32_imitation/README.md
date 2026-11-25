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
                                            ├─ il_cmd_mux ─> /il/cmd ─> twist_limiter ─> /rover_twist ─> 実機
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
| `sim2d.py` | 実機なしでパイプラインを試す簡易2Dシミュレータ |

特徴量は「ロボット正面を0°として360°を36分割し、各方向の一番近い物体までの距離」です。
E32の `laser_frame` は `base_link` に対して -90° 回っているので（URDFより）、その分を補正しています。

## 準備

```bash
cd ~/ros2_ws/src
# lightrover_e32_ros2 は導入済みとします
cp -r <このフォルダ> lightrover_e32_imitation
cd ~/ros2_ws && colcon build --symlink-install --packages-select lightrover_e32_imitation
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
IL=~/ros2_ws/src/lightrover_e32_imitation/lightrover_e32_imitation
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

## 実機なしで試す

ROS も実機も不要で、venv だけで動きます。

```bash
IL=~/ros2_ws/src/lightrover_e32_imitation/lightrover_e32_imitation
~/il_venv/bin/python $IL/sim2d.py record --out ~/il_data_sim/demo_sim.npz
~/il_venv/bin/python $IL/train.py --data ~/il_data_sim --out ~/il_models/sim_model.npz --mirror
~/il_venv/bin/python $IL/sim2d.py eval --model ~/il_models/sim_model.npz --plot ~/il_models/sim_run.png
```

四角いドーナツ形のコースを「お手本コントローラ」が走ったデータで学習し、学習した方策で周回させます。
物理は単純化しているので、ここで動いても実機で動くとは限りません（その差を体験する教材にもなります）。

## 教材として扱えるテーマ

- **データの量と質**: 周回数を 2 / 5 / 10 周と変えて比較する
- **前処理**: `--sectors` や `--max-range` を変えて挙動の違いを見る（生データで保存しているので再収集不要）
- **データ拡張**: `--mirror` の有無で、片回りしか教えていないのに逆回りできるか試す
- **行動の多峰性**: コース中央に障害物を置き、左右半々で避けるデモを取ると、平均をとって正面衝突しやすくなる。なぜか、どう解決するか（デモを揃える、ACTや拡散ポリシーなど）を考える
- **DAgger**: 修正データの有無で成功率を比較する

## 既知の制限

- LiDARは 12cm 未満を測れず、その方向は「遠い」と同じ扱いになります。通路は広めに作ってください。
- 前方 ±30° に 15cm 未満の物があると前進しない安全装置を入れていますが、上の理由で完全ではありません。卓上で運用し、必ず人が見ていてください。
- 行動は「前進速度・旋回速度」の2つで、後退するデモは想定していません。

## ライセンス

[Apache License 2.0](LICENSE) で公開しています。
