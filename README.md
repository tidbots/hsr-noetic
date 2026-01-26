# HSRをROS1 Noeticで開発する
ROS Noetic Ninjemysをdockerから簡単に使いたい。

- ROS Noetic Ninjemys
- Ubuntu 20.04 LTS (Focal Fossa) GPU
- GPUバージョン

## 事前の準備
お使いのコンピュータにNVIDIA(GPU）関連の設定,DockerとDocker composeをインストールしてください。

下記のホストPC環境での動作を確認しています。

- Ubuntu 22.04.3 LTS
- Docker version 24.0.7, build afdd53b
- Docker Copmose version v2.21.0

### DockerとDocker-composeのインストール
### NVIDIA (GPU)関連の設定

### ロボット（実機）と開発用PCの時刻同期
ロボット（実機）を動かすためには、ロボット（実機）と開発用PCの間で時刻同期が行われている必要があります。

ホストPC（ここでは開発用PCで動作しているUbuntu）にchronyをインストールし、ロボット（実機）と時刻の同時を行います。

コンテナではホストPCのネットワークを使用するので、ホストPCがロボット（実機）と時刻同期してれば、コンテナからも時刻の同期が行えます。

chronyを使った時刻同期の手順は[HSRユーザマニュアルの時刻同期](https://docs.hsr.io/hsrc_user_manual/howto/pc_setup_for_robot.html#id2)に従ってください。

この手順はシミュレータで開発する場合には必要ありません。

## インストール
```
$ cd ~
$ git clone https://github.com/tidbots/hsr_noetic.git
$ cd hsr_noetic
$ docker compose build
```
イメージが hsr:noetic-nvidia という名前で作成されます。
イメージ名を変更したい場合は、compose.ymlを編集してください。

## 設定ファイルの編集
.envファイルをお使いの環境に合わせて編集してください。

## 初めて実行する前の大事な注意
Dockerコンテナで行った、ディレクトリの作成やファイルの編集、パッケージのインストールなどはDockerイメージには反映されず、コンテナが破棄された時点で消えてしまいます。
オブジェクト指向で言うところのクラス（Dockerイメージ）とインスタンス（Dockerコンテナ）の関係です。

そこで、ホストコンピュータのディレクトリをコンテナにマウントし共有します。
このようにすれば、相互にマウントしたディレクトリ内での編集作業が消えることはありません。


### ホストコンピュータとコンテナ間で共有するディレクトリの作成
コンテナを初めて起動するする前に、事前の準備として下記の通り、ホストコンピュータにディレクトリを作成しておいてください。
```
$ cd ~
$ mkdir share
```
ここでは"share"という名前のディレクトリを作成しています。
共有するディレクトリの名前は .envファイルで下記のように指定しています。"share"以外の名前にしたい場合は.envファイルを編集してください。
```
WORKSPACE_DIR=/home/roboworks/share
```

### 事前に共有ディレクトリを作成しなかった場合
事前に共有ディレクトリを作成しなかった場合、Dockerシステムは共有ディレクトリを自動的に作成します。
ただし、自動的に作成された共有ディレクトリはroot権限で作成されるため、書き込みが自由にできないディレクトリになってしまいます。
そのような場合はコンテナを停止した後に、ホストコンピュータで下記のように共有したいディレクトリのユーザとグループを変更してください。
```
$ cd ~
$ ls -al share
合計 8
drwxr-xr-x  2 root   roor     4096 12月 31 23:29 .
drwxr-x--- 24 ユーザ名　グループ名 4096 12月 31 23:29 ..
```

```
$ sudo chown ユーザ名 share/
$ sudo chgrp グループ名 share/
```
共有ディレクトリのユーザ名とグループ名が変更されているか確認します。
```
$ ls -al share
合計 8
drwxr-xr-x  2 ユーザ名 グループ名 4096 12月 31 23:31 .
drwxr-x--- 24 ユーザ名 グループ名 4096 12月 31 23:31 ..
```

## コンテナの実行
下記のコマンドでコンテナを起動します
```
$ cd ~/hsr_noetic
$ docker compose up
```
ホストコンピュータの別の端末から起動中のコンテナに入るには下記のコマンドを実行します。
```
$ cd ~/hsr_noetic
$ docker compose exec hsr_noetic /bin/bash
```

## シミュレータで開発する
### シミュレータの起動
```
$ sim_mode
$ roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```
![](/fig/Screenshot_from_2023-09-24_11-26-28__1_.png)

![](/fig/Screenshot_from_2023-09-24_11-26-44__1_.png)


### 会場ガイドのサンプル
https://docs.hsr.io/hsr_develop_manual/python_interface/hsr_guide_sample.html#id2

サンプルを実行すると、ロボットが会場を一周して案内します。
```
cd ~/roboworks/sample
$ python3 guide_mega.py
```

### 把持のサンプル
https://docs.hsr.io/hsr_develop_manual/python_interface/hsr_pick_sample.html

マーカ認識で把持をするサンプルです。

1. ARマーカ付きの物体を出現させる
[ARマーカ](https://docs.hsr.io/hsr_develop_manual/python_interface/marker_interface.html#spawn-bottle-with-marker-label) を参照

```
roslaunch spawn_bottle_with_marker.launch
```
以下のようにgazebo上にマーカ付きのボトルが出現したと思います。

![](/fig/.png)

2. 把持プログラムの実行
```
python3 grasp_bottle.py
```

### ROSのインターフェースを使う
https://docs.hsr.io/hsr_develop_manual/ros_interface/ros_interface.html を参照

サンプルプログラムは　~/catkin_ros/src/ にあります。

例えば、頭を動かすサンプルは　https://docs.hsr.io/hsr_develop_manual/ros_interface/ros_controller_head.html　にあるように,下記のコマンドを実行して下さい。
```
rosrun hsrb_motion_samples head_message.py
```




## 実機で開発する


### 時間の同期の確認
お使いのPCとHSRの時間が同期されているか確認してください
```
$
```

```
$ ihsrb
```

### 実機への接続の確認
```
$ ping hsrb.local

```
### ROSに関する環境変数が正しく設定されているか確認してください。
```
$ env | grep ROS
ROS_MASTER_URI=http://hsrb.local:11311
ROS_IP=192.168.190.20
```



### サンプルプログラム（トヨタ公式）を試してみる
```
$ rosrun 
```



## HSRで開発を進める
[HSR開発マニュアル(最新)](https://docs.hsr.io/hsr_develop_manual/index.html) を参考にHSRのプログラミングを楽しんでください。

## 高度な使い方
開発環境を変更したい場合は、各HSRのユーザマニュアルを参考にして下さい。
- [HSRBユーザマニュアル](https://docs.hsr.io/hsrb_user_manual/index.html)
- [HSRCユーザマニュアル](https://docs.hsr.io/hsrc_user_manual/index.html)
