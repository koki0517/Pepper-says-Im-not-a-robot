# Pepper-says-Im-not-a-robot

3輪オムニホイールの台車に載った Pepper を、SpaceMouse（3Dconnexion）で操作するための ROS 2 パッケージです。

- 台車の全方向移動（前後・左右・旋回）
- 首を砲塔のように動かし、機体が旋回しても頭が同じ方向を向き続ける（IMU なし。車輪の回転量から機体の向きを計算）
- 左右ボタンで、頭が向き続ける方向を変更
- 腕を SpaceMouse の傾きに追従させる

## 動作環境

| 項目 | 内容 |
|---|---|
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble |
| 入力 | 3Dconnexion SpaceMouse（`spacenavd` + `ros-humble-spacenav`） |
| アクチュエータ | Dynamixel（Protocol 1.0、1 Mbps、`/dev/ttyUSB0`） |

### ハードウェア構成

| ID | サーボ | 役割 | モード |
|---|---|---|---|
| 1 | MX-106 | 後ろのホイール（180°） | ホイールモード |
| 2 | MX-106 | 左前のホイール（+60°） | ホイールモード |
| 3 | MX-106 | 右前のホイール（-60°） | ホイールモード |
| 4 | RX-28 | 首（360°以上回転できる） | ホイールモード |
| 5 | RX-28 | 腕 | 関節モード |

角度は Pepper のローカル座標で、前を 0°、上から見て反時計回り（左）を正としています。

- ホイール直径: 10 cm
- 機体の中心からホイールまで: 12 cm
- 座標系: 前が X+、左が Y+、上が Z+（右手系）

## セットアップ

```bash
# spacenav ドライバ（spacenavd は事前にインストールしておく）
sudo apt install ros-humble-spacenav

# ワークスペースに clone（DynamixelSDK はサブモジュール）
cd ~/ros2_ws/src
git clone --recursive https://github.com/koki0517/Pepper-says-Im-not-a-robot.git
# すでに clone している場合
git submodule update --init

# ビルド
cd ~/ros2_ws
colcon build --packages-select dynamixel_sdk pepper_omni
source install/setup.bash
```

`/dev/ttyUSB0` を sudo なしで使うには、ユーザーが `dialout` グループに入っている必要があります。

## 使い方

```bash
ros2 launch pepper_omni teleop.launch.py
```

spacenav ドライバ、joy の変換、Dynamixel ドライバがまとめて起動します。

### SpaceMouse の操作

| 操作 | joy の値 | 動き |
|---|---|---|
| 前後に押す | `axes[2]` | 前進 / 後退（最大 0.3 m/s） |
| 左右に押す | `axes[1]` | 左 / 右へ移動（最大 0.3 m/s） |
| ひねる | `axes[3]` | 旋回（最大 1.0 rad/s） |
| 前後に傾ける | `axes[4]` | 腕：中立で 108°、前に倒すと最大 158°、後ろに倒すと最大 58° |
| 左ボタン | `buttons[0]` | 頭が向く方向を左へ回す（押している間 90°/s） |
| 右ボタン | `buttons[1]` | 頭が向く方向を右へ回す（押している間 90°/s） |

腕は傾きに合わせて角度が決まり、手を離すと 108° に戻ります。頭が向く方向は、起動時に頭が向いていた方向から始まります。

### 起動時と終了時の動き

- **起動時**: 腕は最初に joy の値を受け取ったとき、すぐに 108° へ動きます。周りに物がないか確認してから起動してください。
- **終了時（Ctrl+C）**: ホイールと首は止まってトルクが切れます。腕は落ちないように、トルクを入れたまま位置を保ちます。

### 個別に起動する

```bash
ros2 launch pepper_omni spacenav.launch.py      # SpaceMouse → /cmd_vel など
ros2 launch pepper_omni omni_driver.launch.py   # Dynamixel ドライバのみ

# ドライバだけ起動して、指令を直接送る
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

## ノードとトピック

```
spacenav_node ──/spacenav/joy──▶ joy_to_cmd_vel ──/cmd_vel────────▶ omni_driver ──▶ Dynamixel
                                                ──/head_turn_rate──▶
                                                ──/arm_goal_deg────▶
```

| トピック | 型 | 内容 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 台車の速度 [m/s, rad/s] |
| `/head_turn_rate` | `std_msgs/Float64` | 頭が向く方向を回す速さ [rad/s]（反時計回りが正） |
| `/arm_goal_deg` | `std_msgs/Float64` | 腕の目標角度（サーボの角度 [deg]） |
| `/base_yaw` | `std_msgs/Float64` | 車輪の回転量から計算した機体の向き [rad]（起動時が 0） |
| `/head_angle` | `std_msgs/Float64` | 機体から見た頭の角度 [rad] |

### joy_to_cmd_vel

`/spacenav/joy` の各軸に、デッドゾーン（0.1）と最大速度を掛けて `/cmd_vel` を出します。joy が 0.5 秒届かなくなると、速度 0 を出します。

### omni_driver

50 Hz で次の処理を繰り返します。ポートは1つのプロセスしか開けないので、ホイール・首・腕をこのノード1つで扱います。

1. **ホイール**: `/cmd_vel` を各ホイールの回転数に変換し、レジスタの値（単位 0.114 rpm）として書き込みます。どれかのホイールが上限（400 ≒ 45.6 rpm）を超えるときは、3輪を同じ比率で縮めるので、進む方向は変わりません。`/cmd_vel` が 0.5 秒届かなくなると止まります。
2. **機体の向きの計算**: 3輪が 120° おきに付いているので、3輪の接地点の移動量を合計すると、並進の成分は打ち消し合って旋回だけが残ります。
   `Δyaw = Σ(ホイールの移動量) / (3 × 機体の半径)`
3. **首**: 目標は「頭が向く方向 − 機体の向き」です。機体の旋回を打ち消す分（フィードフォワード）と、目標とのずれに比例した補正（P 制御）を足して速度指令にします。RX-28 は 0〜300° の範囲しか位置を読めないので、残りの 60° では指令した速度から角度を推定します。
4. **腕**: `/arm_goal_deg` を 58〜158° に制限し、目標位置として書き込みます。

### 首（RX-28）の速度特性

RX-28 のホイールモードの速度指令は、回転数ではなく出力の割合です。実測（12.4 V）すると、次の関係になりました。

```
角速度 [deg/s] ≈ 0.335 × (|指令値| − 48)
```

指令値が約 48 以下ではほとんど動かない（静止摩擦）ので、指令値に 48 を上乗せしています（`neck_offset_value`）。電源電圧や機構が変わったら測り直してください。

## 設定

設定は `pepper_omni/config/` の YAML ファイルにあります。インストール先にコピーして使われるので、変更した後は `colcon build` し直してください。

| ファイル | 主な設定 |
|---|---|
| `joy_to_cmd_vel.yaml` | 軸とボタンの割り当て、符号、最大速度、デッドゾーン、首を回す速さ、腕の中心角度とゲイン |
| `omni_driver.yaml` | ポート、ID、ホイールの角度と回転方向、寸法、速度の上限、首の制御の調整値、腕の可動範囲 |

## 既知の問題

- 走行中に ID1 の読み取りが時々失敗します（止まっているときは失敗しません）。読めなかった分は次に読めたときにまとめて計算するので、機体の向きの計算には影響しません。配線かノイズの問題と考えられます。
- 機体の向きは車輪の回転量だけから計算しているので、車輪が滑ると、頭の向きが少しずつずれていきます。
