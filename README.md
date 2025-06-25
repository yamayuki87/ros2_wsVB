# プロジェクト名

## 概要
このリポジトリは〇〇〇を目的としたROS2パッケージ群です。

---

## パッケージ一覧

- **command_dyposition** – サーボ位置を指令するサービスの例
- **dynamixel_sdk** – ROBOTIS製 Dynamixel SDK の ROS 2 ラッパー
- **dynamixel_sdk_custom_interfaces** – SDK用カスタムインタフェースの定義
- **dynamixel_sdk_examples** – SDKを利用したサンプルコード
- **exmple_dynamixel_cpp** – Dynamixel XM540-W270-R を制御するノード
- **joyserv** – ジョイスティック入力をトピック配信するノード
- **joysub** – パッケージは未整備のディレクトリ

---

## 動作環境

- OS：Ubuntu 22.04
- ROS2ディストリビューション：Humble / Iron / Jazzy（選択）

---

## インストール手順

```bash
# ROS2のセットアップ
source /opt/ros/jazzy/setup.bash

# このリポジトリをクローン
git clone https://github.com/your_account/your_repo.git
cd your_repo

# 依存パッケージインストール（必要な場合）
rosdep install --from-paths src --ignore-src -r -y
