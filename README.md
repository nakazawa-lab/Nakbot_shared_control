# 入力融合による遠隔操作支援プログラム

## 目的と概要

- NakBotの遠隔操作支援を行う
- 遠隔操縦入力を加えた際に、LRFで測定した障害物に衝突しないよう自然に入力を修正する。
- DWA(Dynamic Window Approach)を人間からの入力を考慮したものにする



## パッケージ構成

- my_robo_descriptionパッケージ
  - NakBotを表現し、シミュレータgazebo上で表示させるためのパッケージ
  - ロボットのモデル、gazeboのモデル、rvizの設定ファイルなどが含まれる
- my_robo_simulationパッケージ
  - NakBotを実際にシミュレータで動かし、手法を実装させるパッケージ
  - ジョイスティックからの入力を管理するプログラム、提案手法のプログラムが含まれる

```
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-joystick-drivers
```



- JetSASパッケージ
  - 中澤研のサンプルプログラムをもとに、JetSASシステムとシミュレーションで開発したプログラムの橋渡しを行うパッケージ
  - 実機センサ情報の取得、SH2Aボートとの通信を行う



## プログラムの実行手順

- シミュレータの開始

``` 
roslaunch my_robo_description my_robo_description.launch [必須optionでempty:=Trueまたはsparse_house:=Trueまたはhouse:=True] [optionでX:=x座標 Y:=y座標 yaw*=ラジアン角度]
```

- 実行(パソコンにPS3コントローラを接続して行う)

```
roslaunch my_robo_simulation my_robo_drive.launch
```

- NakBotでの実行
  - rootログインしたシェルで行う(センサ情報の取得ができなくなる)
  - 別ターミナルでroscoreを開いておいて、以下を実行

```
bash ~/catkin_ws/src/NakBot_shared_control/JetSAS/scripts/set_parameter.sh
roslaunch JetSAS JetSAS_node
```





## 重要ファイル

### nakbot_shared_control

- メタパッケージ

### my_robo_descriptionパッケージ

- my_robo.xacro
  - ロボットのモデリングを行ったファイル
- controller.yaml
  - gazebo内で動く差動二輪駆動モータのシミュレーションの設定ファイル

### my_robo_simulationパッケージ

- src, includeフォルダ内
  - 提案手法のプログラム

### JetSASパッケージ

- set_patameter.sh
  - 実機の最高速度や加速度をroscoreで立ち上がるマスタに登録する
  - これをやらないと思うような速度で動かない
- ros_node.cpp, ros_node.h
  - 研究室のサンプルプログラムに付け加えたファイル。
