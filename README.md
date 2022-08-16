# ROSマッサージベッドシステム

このレポジトリに含まれているものはマッサージベッドを動かすためのROSパッケージです．ロボットアームを動かすためにROSを内部では使用していますが，ユーザーはROSを使用せず外部のPCから命令を送り，センサ情報を取得することが可能です．

このパッケージには下記の３つの機能が含まれています．

- ユーザーからの手先速度司令をもとにロボットアームを動かす（techman_control_bridge）
- ロボットアームの手先位置の情報をユーザーに送る（techman_sensor_bridge）
- 力覚センサの情報をユーザーに送る（leptrino_sensor_bridge）

各プログラムの最後にbridgeとついている理由は，これらがユーザーから情報を受け取って，それをROSのメッセージに変換する，もしくはROSメッセージを平分に直してユーザーに送る役割を担っているからです．



## 1. 使い方

### 1.1 プログラミング言語

ロボットアームへの速度司令の送信と手先位置と力覚センサからのデータ受信は，TCP通信を使って行われます．TCP通信はほぼすべてのプログラミング言語に備わっており，MATLABやC#，C++といった言語からマッサージベッドを動かすことが可能です．



### 1.2 通信環境

最小の実験環境は下記のとおりです．

```
---------------          ---------------
|             |          |             |<------------>Robot Arm (Techman)
|   User PC   |<-------->|   ROS PC    |
|             |          |             |<------------Force Torque Sensor (leptrino)
---------------          ---------------
```

ユーザーPCは司令塔となるPCです．このPCの内部でユーザーが作成したプログラムが実行されます．このPCのOSはWindowsやLinux，MacOSでも問題ありません．必要な条件はTCP通信が可能なことと，Ethernetケーブルが刺さることです．

ROS PCはROSが実行されるPCになります．このPCはユーザーPCからのデータを受け取り，また，ROS内部のメッセージを平文に直して，ユーザーPCに送ります．

ユーザーPCとROS PCの間の通信はEthernetケーブルによって行われます．



### 1.3 IPアドレス

本システムを構成するネットワークのIPアドレスは下記の表のようになっています．ユーザーPCを使用する際はIPアドレスを`192.168.10.3` ~ `192.168.10.255`の範囲内のどれかのIPアドレスを使うようにしてください．

| PC名           | IPアドレス                   |
| -------------- | ---------------------------- |
| ROS PC         | 192.168.10.1                 |
| ロボットアーム | 192.168.10.2                 |
| ユーザーPC     | 192.168.10.3~255までのどれか |



### 1.4 データの送受信

#### a. 速度指令の送信

ロボットアームに手先速度指令を送る場合には，下記のフォーマットで，`192.168.10.1`のポート`50011`にTCP通信にて送信してください．

```
VELC0.01,0.00,0.00,0.00,0.00,0.00:
```

ヘッダーとして`VELC`を，フッターとして`:`を速度指令の前後につけてください．速度指令は`,`で区切ってください．また，速度指令の単位は`m/s`であり，順番はx,y,z,roll,pitch,yawです．



#### b. 手先位置の受信

ロボットアームの手先位置を受信する場合には，`192.168.10.1`のポート`50012`からデータをTCP通信にて受信してください．ポートからは下記のフォーマットでロボットアームの手先位置情報が送られてきます．

```
POSE0.00,0.00,0.00,0.00,0.00,0.00,0.00:
```

ヘッダーとして`POSE`が，フッターとして`:`がついてきます．また，`,`で区切って７つのデータが受信できます．最初の３つのデータは手先座標のｘ，ｙ，ｚで，単位はmです．残りの４つのデータはクォータニオンで順番は`x`,`y`,`z`,`w`です．



#### c. 力覚センサーの受信

力覚センサーから力やトルク情報を受信する場合には，`192.168.10.1`のポート`50013`からデータをTCP通信にて受信してください．ポートからは下記のフォーマットで力覚センサーの値が送られてきます．

```
SNSE0.00,0.00,0.00,0.00,0.00,0.00:
```

ヘッダーとして`SNSE`が，フッターとして`:`がついてきます．また，`,`で区切って6つのデータが受信できます．最初の３つのデータは力で，順番はｘ，ｙ，ｚであり，単位は`N`です．残りの３つのデータはトルクで，順番はｘ，ｙ，ｚ軸周りとなります．単位は`Nm`です．



### 1.5 サンプルプログラム

- ロボットアームの速度指令のサンプルプログラム
  - `src/test_techman_control_bridge.cpp`
  - `src/tool_techman_control_bridge.cpp`
- ロボットアームの手先位置の受信のためのサンプルプログラム
  - `src/test_techman_sensor_bridge.cpp`
- 力覚センサーの値の受信のためのサンプルプログラム
  - `src/test_leptrino_sensor_bridge.cpp`



## 2. ROS PC起動方法

### 2.1 ロボットアームドライバーの起動

```shell
roslaunch tm5-900-moveit-config tm5-900_moveit_planning_execution.launch sim:=false robot_ip:=192.168.10.2
```

上記のコマンドによりロボットアームのROSドライバーが立ち上がります．GUIが立ち上がり，現実のロボットとGUI中のロボットが同じ姿勢をとったら接続ができた証拠です．うまく接続できない場合はロボットアームの制御ボックスを再起動してください．



### 2.2 techman_control_bridgeの起動

```shell
rosrun ros_massage_bed techman_control_bridge
```



### 2.3 techman_sensor_bridgeの起動

```shell
rosrun ros_massage_bed techman_sensor_bridge 
```



### 2.4 力覚センサードライバーの起動

```
roslaunch leptrino_force_torque leptrino.launch
```



### 2.5 leptrino_sensor_bridgeの起動

```shell
rosrun ros_massage_bed leptrino_sensor_bridge
```



## 3. チュートリアル

### 3.1 ロボットを動かす

ここではロボットを動かす練習をします．ロボットの速度制御で動作させるために必要なプログラムを起動し，簡単な命令を送ってみましょう．

まず，下記のコマンドをROS PCで実行してください．

```
roslaunch tm5-900-moveit-config tm5-900_moveit_planning_execution.launch sim:=false robot_ip:=192.168.10.2
```

```
rosrun ros_massage_bed techman_control_bridge
```

```
rosrun ros_massage_bed tool_techman_control_bridge
```

スライダーを左右に動かすと，それに合わせてロボットの手先速度も変化することを確認してみましょう．



### 3.2 手先位置を受信する

ロボットの手先位置を受信してみましょう．

下記のコマンドをROS PCで実行してください．もし，チュートリアル３．１のプログラムがまだ立ち上がっているなら`Cntl+C`で切ってください．ただ，切らなくても致命的な問題は起こりません．ROSは同名のプログラムが立ち上がるのを許さないので，すでに立ち上がっている同名のプログラムが自動的に切れるだけです．

```
roslaunch tm5_900_moveit_config tm5_900_moveit_planning_execution.launch sim:=false robot_ip:=192.168.10.2
```

```
rosrun ros_massage_bed techman_sensor_bridge 
```

```
rosrun ros_massage_bed test_techman_sensor_bridge 
```

`test_techman_sensor_bridge`を起動するとロボットアームの手先位置情報が表示されることを確認してください．



### 3.3 力とトルクを受信する

```
roslaunch leptrino_force_torque leptrino.launch
```

```
rosrun ros_massage_bed leptrino_sensor_bridge
```

```
rosrun ros_massage_bed test_leptrino_sensor_bridge
```

`test_leptrino_sensor_bridge`を起動したあとに，力覚センサーを押したり，ねじったりしてみましょう．画面に表示されている数値が変化することを確認してください．



## 4. 依存関係

#### ROS PCの依存関係

- [Boost ](https://www.boost.org/)
- [Poco](https://pocoproject.org/)
- [imgui](https://github.com/ocornut/imgui)



## 5. 再インストール

前提としてROSはインストール済みとします．

### 5.1 依存関係のインストール

```
sudo apt build-essential
sudo apt install libboost-all-dev
sudo apt install libpoco-dev
sudo apt install python3-catkin-tools
sudo apt install libglfw3-dev
sudo apt install libgoogle-glog-dev
```

### 5.2 ROSパッケージのインストール

```shell
cd ~/catkin_ws/src/
git clone https://github.com/TechmanRobotInc/tmr_ros1.git
git clone --branch stable https://github.com/harumo11/ros_massage_bed.git:stable
```
```
git clone https://github.com/harumo11/leptrino_force_torque.git
cd leptrino_force_torque
sudo cp 99-leptorino.rules /etc/udev/rules.d/
```

```
catkin build (環境によってはcatkin makeを使ってください．)
```
