
## パッケージ一覧

- **command_dyposition** – サーボ位置を指令するサービスの例
- **dynamixel_sdk** – ROBOTIS製 Dynamixel SDK の ROS 2 ラッパー
- **dynamixel_sdk_custom_interfaces** – SDK用カスタムインタフェースの定義
- **dynamixel_sdk_examples** – SDKを利用したサンプルコード
- **exmple_dynamixel_cpp** – Dynamixel XM540-W270-R を指定した値(コード内の数値に一度だけ)に位置制御するノード
- **command_dyposition**-serviceパッケージでクライエントが指定した値に Dynamixel XM540-W270-Rを位置制御するノード　自由に何度も設定可能
- **joyserv** – コントローラーの×ボタンが入力を検知するノード
- **joysub** – ジョイスティック入力をサブスクライブするノード
- **roboclaw_test**-roboclawモータードライバでmaxonモーターを指定した速度で回転させるノード
- **roboclaw_joy**-左ジョイスティックの値でmaxonモーターの回転速度を変更するノード

---

## 動作環境

- OS：Ubuntu 24.04
- ROS2ディストリビューション：Jazzy

---

## インストール手順

```bash
# ROS2のセットアップ
source /opt/ros/jazzy/setup.bash

# このリポジトリをクローン
git clone https://github.com/yamayuki87/ros2_wsVB.git

# 依存パッケージインストール（必要な場合）
rosdep install --from-paths src --ignore-src -r -y
