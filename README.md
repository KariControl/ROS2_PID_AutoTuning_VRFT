# はじめに
本ソフトウェアはROS 2のrosbagファイルで保存した入出力データを用いてPIDゲインを調整する自動調整ソフトである。PIDゲイン調整用のパッケージ、サンプルのプラント・制御器のパッケージ、実行用スクリプトから構成されている。

参考文献：
Campi, M. C., Lecchini, A., & Savaresi, S. M. (2002). Virtual reference feedback tuning: a direct method for the design of feedback controllers. Automatica, 38(8), 1337-1346.

作成者のqiita記事：
https://qiita.com/KariControl/items/bfbfb5b9ac7492df938f

# 環境
OS:ubuntu 22.04
ROS version:ROS 2 humble
言語：C++、Python、shell scripts

# ノード構成
・controller_msgs
PIDゲイン調整用入出力信号のためのタイムスタンプ付きカスタムメッセージノード。制御入力inputと出力応答outputの変数を内包。

・time_sync
時刻同期用のノード。rosbagでプレイされた入力信号と出力信号を受信。受信した入出力を時刻同期させた上でcontroller_msgsのメッセージとして出力。

・pid_tuner
PIDゲイン調整用ノード。time_syncから受信した入出力データにてPIDゲインの自動調整を実行。調整後のゲインをターミナル上に表示。

・vehicle_sim(option)
例題用の制御対象ノード。PIDゲイン調整機能とは関係ないため，例題を実行しない場合は不要。

・velocity_control(option)
例題用の制御器ノード。PIDゲイン調整機能とは関係ないため，例題を実行しない場合は不要。

<img src="image/node_config.png">

# 例題の実行手順
1.ビルド
mainブランチをクローンして，下記のコマンドを実行する。

source build_setup.sh

2.初期入出力データの測定(rosbagの準備)
下記のコマンドを実行して例題用の入出力データ(rosbag)を生成しておく。

source run_velocity_control_sim.sh

なお，本例題ではlaunchファイル(control_run.py)にて例題の制御器におけるPIDゲインを設定している。rosbagデータの入出力応答を変更する場合，launchファイル上のパラメータを変更して実行する必要がある。

3.PIDゲイン調整
シェルスクリプト(source run_PID_tuning.sh)内のrosbag名を該当ファイル名に変更する。launchファイル(pid_tuner.py)にて，参照モデルの時定数とデータ数(読み込むデータの最大データ点数)を設定する。下記のコマンドを実行してrosbagデータからPIDゲインをオフライン計算する。

source run_PID_tuning.sh

正常に実行ができた場合，下記のようにターミナル上にてPIDゲインの調整結果が表示される。

<img src="image/result.jpg">

4.ゲイン調整結果の確認
launchファイル(control_run.py)のPIDゲインを変更し，下記のコマンドを実行して制御器調整後の制御応答を確認する。

source run_velocity_control_sim.sh