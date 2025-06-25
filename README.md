
+
+## パッケージ一覧
+- **command_dyposition** – サーボ位置を指令するサービスの例

+- **dynamixel_sdk** – ROBOTIS 製 Dynamixel SDK の ROS 2 ラッパー

+- **dynamixel_sdk_custom_interfaces** – SDK 用カスタムインタフェースの定義
+- **dynamixel_sdk_examples** – SDK を利用したサンプルコード

+- **exmple_dynamixel_cpp** – Dynamixel XM540-W270-R を制御するノード

+- **joyserv** – ジョイスティック入力をトピック配信するノード

+- **joysub** – パッケージは未整備のディレクトリ
+```bash
+```bash
+```bash
+## ビルド方法
+1. ROS 2 をあらかじめセットアップします。
+   source /opt/ros/jazzy/setup.bash
+2. 本ワークスペースのルートで `colcon` を実行します。
+   colcon build --symlink-install
+3. ビルド後はワークスペースを読み込みます。
+   ```bash
+   source install/setup.bash
+   ```
+
+## ライセンス
+ライセンスは各パッケージの `LICENSE` ファイルを参照してください。

