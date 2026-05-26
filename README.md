# 関東春ロボコン2025 手動/自動ハイブリッド 自律移動ロボット制御システム 「haru25」

## 概要
関東春ロボコン2025の競技テーマ「クマさんの蜂蜜狂宴」に出場した自律移動ロボットの制御システムです。
複雑なスラローム（雑木林ゾーン）や狭い一本道（小道ゾーン）を抜け、フィールド上のオブジェクトを正確に回収・運搬するための高度なナビゲーション機能を持っています。

本プロジェクトの最大の特徴は、**ROS 2（上位PC）とTeensyマイコン（下位ハードウェア）による高度な分散制御アーキテクチャ**です。自己位置推定や経路計画などの高負荷な演算をROS 2で処理し、厳密なリアルタイム性が要求されるモーターのPID制御やセンサ読み取りをTeensyにオフロードすることで、安定した制御システムを構築しました。

## システム構成・技術スタック
* **High-Level Control (ROS 2 PC):**
  * Framework: ROS 2
  * Navigation: EKF (拡張カルマンフィルタ), ICP (反復最近接点アルゴリズム), Pure Pursuit
  * Simulation: Gazebo, Rviz2
* **Low-Level Control (Microcontroller):**
  * MCU: Teensy 4.x (Arduino/C++環境)
  * Actuators: DJI Robomaster モーター (C610/C620等), 空圧シリンダ
  * Sensors: LD19 LiDAR, BNO055 IMU, QuadEncoder
* **Communication Protocols:**
  * PC-MCU間: `PacketSerial` (COBSエンコードによる200Hz高速・高信頼性シリアル通信)
  * MCU-Motor間: CAN通信 (`FlexCAN_T4`を用いた1kHz制御)

## 実装した主要機能
- **上位/下位の役割を明確化したハイブリッド制御**  
  ROS 2の `navigation.cpp` ノードにて、目標速度・角速度を計算。そのデータを `PacketSerial` を用いて200Hz周期でTeensyへ送信し、Teensy側では受信データを目標値としてCANバス経由で4輪のRobomasterモーターを1kHzの高速ループで制御（`can_test.ino`）しています。

- **EKFとICPによる堅牢なセンサーフュージョン自己位置推定**  
  エンコーダの回転情報とIMUからの角速度をEKF（拡張カルマンフィルタ）で融合し、滑らかなベース位置を推定。さらに、LD19 LiDARからの2DスキャンデータをICPアルゴリズムを用いて環境マップと照合させることで、タイヤの空転による累積誤差を完全に補正する自作のローカライゼーションパイプラインを構築しました。

- **Sim2Realを前提とした開発環境の構築**  
  実機のハードウェアと通信する `hardware.launch.py` と、シミュレータ上で動作確認を行う `gazebo.launch.py` を分離して用意し、ソフトウェアのアルゴリズム検証とハードウェアのデバッグを並行して進められるモダンな開発ライフサイクルを整備しています。
