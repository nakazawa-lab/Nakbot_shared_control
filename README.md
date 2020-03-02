# 北島卒業論文プログラム

## 目的と概要

- NakBotの遠隔操作支援を行う
- 遠隔操縦入力を加えた際に、LRFで測定した障害物に衝突しないよう自然に入力を修正する。
- 論文を参照



## パッケージ構成

- 依存パッケージ
  - ROS標準パッケージであるgmappingメタパッケージを要する
    - 本来必要ないかもしれないがweb上のチュートリアルに従って開発したら依存してしまった
- my_robo_descriptionパッケージ
  - NakBotを表現し、シミュレータ上で表示させるためのパッケージ
  - ロボットのモデル、gazeboのモデル、rvizの設定ファイルなどが含まれる
- my_robo_simulationパッケージ
  - NakBotを実際にシミュレータで動かし、提案手法を実装させるパッケージ
  - ジョイスティックからの入力を管理するプログラム、提案手法のプログラムが含まれる
- JetSASパッケージ
  - 中澤研のサンプルプログラムをもとに、JetSASシステムとシミュレーションで開発したプログラムの橋渡しを行うパッケージ
  - 実機センサ情報の取得、SH2Aボートとの通信を行う

## ビルドまでの手順

- テクノモール時にインストール手順をまとめたので念の為記す

1. ROS公式HPに従ってROS Melodicのインストール

2. 依存パッケージgmapping関連のインストール 

   https://qiita.com/protocol1964/items/1e63aebddd7d5bfd0d1b 

   一番初めの「依存パッケージをインストール」を行う 

3. ```git clone https://github.com/nakazawa-lab/Nakbot_shared_control.git```

4. ビルドする。gazeboのエラーは[これ]([Gazebo [Err\] [REST.cc:205] Error in REST request.](https://www.youtube.com/watch?v=ftDz_EVoatw) )にしたがって直す





## プログラムの実行手順

- 卒論の実験では、シミュレーション実験をmaster, 実機実験をNakBotブランチで行っているので、微妙に挙動が異なる可能性がある

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

### my_robo_descriptionパッケージ

- my_robo.xacro
  - ロボットのモデリングを行ったファイル
- controller.yaml
  - gazebo内で動く差動二輪駆動モータのシミュレーションの設定ファイル

### my_robo_simulationパッケージ

- src, includeフォルダ内
  - 提案手法のプログラム
- scriptsフォルダ内
  - データの解析のために用いたpythonスクリプトたち

### JetSASパッケージ

- set_patameter.sh
  - 実機の最高速度や加速度をroscoreで立ち上がるマスタに登録する
  - これをやらないと思うような速度で動かない
- ros_node.cpp, ros_node.h
  - 研究室のサンプルプログラムに付け加えたファイル。





## 備考

- ミス、動かないなどあればkitajima1206[アットマーク]keio.jpまで