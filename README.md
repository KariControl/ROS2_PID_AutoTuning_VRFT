# はじめに
本ソフトウェアはROS 2のrosbagファイルで保存した入出力データを用いてPIDゲインを調整する自動調整ソフトである。PIDゲイン調整用のパッケージ、サンプルのプラント・制御器のパッケージ、実行用スクリプトから構成されている。

参考文献：
Campi, M. C., Lecchini, A., & Savaresi, S. M. (2002). Virtual reference feedback tuning: a direct method for the design of feedback controllers. Automatica, 38(8), 1337-1346.

本ソフトにおけるqiita記事：
後ほど更新。

# 環境
OS:ubuntu 22.04
ROS version:ROS 2 humble
言語：C++、Python、shell scripts

# パッケージ構成
本ソフトにおける各パッケージの概要を本節で述べる。

### controller_msgs
PIDゲイン調整用入出力信号のためのタイムスタンプ付きカスタムメッセージ。制御入力inputと出力応答outputの変数を内包。

### time_sync
時刻同期用ノードのパッケージ。rosbagでプレイされた入力信号と出力信号を受信。受信した入出力を時刻同期させた上でcontroller_msgsのメッセージとして出力。

### pid_tuner
PIDゲイン調整用ノードのパッケージ。time_syncから受信した入出力データにてPIDゲインの自動調整を実行。調整後のゲインをターミナル上に表示。

### vehicle_sim(option)
例題用の制御対象ノードのパッケージ。PIDゲイン調整機能とは関係ないため，例題を実行しない場合は不要。

### velocity_control(option)
例題用の制御器ノードのパッケージ。PIDゲイン調整機能とは関係ないため，例題を実行しない場合は不要。

# ノード構成
本ソフトのノード構成を下記のグラフに示す。例題の関係上，制御対象の出力データのトピックが/vehicle_velocity，入力データのトピックが/accelとなっている。/plant_infoが時刻同期済みの入出力信号におけるカスタムメッセージでのトピックとである。

![node](https://github.com/KariControl/ROS2_PID_AutoTuning_VRFT/assets/118587431/cd5b62f3-b0b1-43fc-96a1-97fe1a7ddc26)

# 例題の実行手順
1.ビルド
mainブランチをクローンして，下記のコマンドを実行する。

```
source build_setup.sh
```

2.初期入出力データの測定(rosbagの準備)
下記のコマンドを実行して例題用の入出力データ(rosbag)を生成しておく。

```
source run_velocity_control_sim.sh
```

なお，本例題ではlaunchファイル(control_run.py)にて例題の制御器におけるPIDゲインを設定している。rosbagデータの入出力応答を変更する場合，launchファイル上のパラメータを変更して実行する必要がある。

3.PIDゲイン調整
シェルスクリプト(source run_PID_tuning.sh)内のrosbag名を該当ファイル名に変更する。launchファイル(pid_tuner.py)にて，参照モデル(time_const)の時定数とデータ数(読み込むデータの最大データ点数max_data_points)を設定する。下記のコマンドを実行してrosbagデータからPIDゲインをオフライン計算する。

```
source run_PID_tuning.sh
```

正常に実行ができた場合，下記のようにターミナル上にてPIDゲインの調整結果が表示される。

![result](https://github.com/KariControl/ROS2_PID_AutoTuning_VRFT/assets/118587431/ecedffd0-b6bf-43ec-a4e8-67f9b9e70476)

4.ゲイン調整結果の確認
launchファイル(control_run.py)のPIDゲインを変更し，下記のコマンドを実行して制御器調整後の制御応答を確認する。

```
source run_velocity_control_sim.sh
```

# 例題におけるゲイン調整結果例
$`K_{P}=1.4`$，$`K_{I}=0.0`$，$`K_{D}=0.0`$の条件下にて初期入出力応答データを測定すると，下記のような制御応答を得られる。

![initial](https://github.com/KariControl/ROS2_PID_AutoTuning_VRFT/assets/118587431/6c28fdab-cd2b-4040-97ab-78a99d73f2da)
         
本条件下で得られたrosbagデータを用いてゲイン調整を実施することにより，ゲイン調整結果を$`K_{P}=0.766527`$，$`K_{I}=0.0`$，$`K_{D}=0.728840`$と得られる。ただし，最大データ点数は150であり，参照モデルの時定数を1.5sと設定している。調整後ゲインを適用することにより，下記のような制御応答を得られる。

![tuning](https://github.com/KariControl/ROS2_PID_AutoTuning_VRFT/assets/118587431/6937cf45-48da-43f7-8ee2-963d763a00a5)

# 注意事項
### 自前のrosbag(例題以外への適用)使用時の注意
time_syncノードでは，入出力信号をgeometory_msgs/TwistStamped型とgeometory_msgs/AccelStamped型でサブスクライブする仕様となっている。自前のrosbagを用意して例題以外の制御対象に適用する場合，time_syncノードのサブスクライブ処理と時刻同期処理の型を修正する必要がある。また，topic名に関しても自前のrosbagに合わせて変更が必要となる。

### 初期入出力データ(PIDゲイン調整に用いるrosbagデータ)
PIDゲイン調整のアルゴリズム(VRFT)の都合上，制御器調整結果が初期実験データの応答波形に依存する。望ましい制御器調整結果とならない場合，初期入出力応答の再取得やpid_tunerの最大データ点数(max_data_points)変更が必要となる。
