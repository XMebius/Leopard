## 系统环境

ubuntu22.04-desktop amd64

## 基础环境配置

```bash
sudo apt update
sudo apt install vim git gcc mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev libglib2.0-dev 
sudo apt install cmake build-essential libqt5gamepad5 libqt5gamepad5-dev qtbase5-dev
export CMAKE_PREFIX_PATH="/usr/lib/x86_64-linux-gnu/cmake"

git clone https://github.com/mit-biomimetics/Cheetah-Software.git
cd Cheetah-Software
mkdir build
cd build
```

**将common/CMakeLists.txt中的30行中的master改成main**

### Qt下载

从[下载链接](https://download.qt.io/archive/qt/5.12/5.12.0/)处复制链接并使用wget下载

```bash
wget https://download.qt.io/archive/qt/5.12/5.12.0/qt-opensource-linux-x64-5.12.0.run
```

执行qt的安装程序，安装位置放在家目录下

```bash
chmod +x ./qt...
./qt..
```

### Eigen下载

```bash
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xvf eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir build
cd build
cmake ..
make -j20
sudo make install
```

### LCM下载

```bash
sudo apt install libglib2.0-dev python3-dev openjdk-11-jdk doxygen liblua5.2-dev

wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.zip
unzip v1.5.0zip
cd lcm-1.5.0
mkdir build
cd build
cmake ..
make -j20
sudo make install
```

执行`lcm-spy`命令，检查是否成功

### glibc问题

为了解决新版ubuntu中ioctl函数的问题，需下载glibc库

```bash
wget https://ftp.gnu.org/gnu/glibc/glibc-2.11.tar.gz
tar -xf glibc-2.11.tar.gz
sudo mkdir -p /usr/include/bits
sudo cp ./glibc-2.11/streams/stropts.h /usr/include
sudo cp ./glibc-2.11/bits/stropts.h /usr/include/bits
sudo cp ./glibc-2.11/sysdeps/x86_64/bits/xtitypes.h /usr/include/bits
```

### Cheetah编译

编译之前先更新动态链接库，使得lcm的库能被sim目标链接到

```bash
sudo ldconfig
```

将/robot/src/SimulationBridge.cpp中的79行的错误处理改为

```cpp
} catch (std::exception& e) {
  // 将异常信息转换为 std::string
  std::string errorMsg = e.what();

  // 确保不超过缓冲区大小（留一个字符的空间用于 '\0'）
  size_t copySize = std::min(errorMsg.size(), sizeof(_sharedMemory().robotToSim.errorMessage) - 1);

  // 使用 std::copy 安全复制数据
  std::copy(errorMsg.begin(), errorMsg.begin() + copySize, _sharedMemory().robotToSim.errorMessage);

  // 手动设置字符串终止符
  _sharedMemory().robotToSim.errorMessage[copySize] = '\0';

  // 重新抛出异常
  throw;
}
```

开始编译之前，需要对CMakelists.txt进行修改:

1. 将sim/CMakeLists.txt中的`cmake_policy(SET CMP0071 OLD)`中的OLD改为NEW解除cmake警告
2. 将所有的cmake_minimum_required(VERSION ...)改为cmake_minimum_required(VERSION 3.15)解除警告
3. 将third_party/osqp/CMakeLists.txt中的`add_library (osqpstatic SHARED ${osqp_src} ${osqp_headers} ${linsys_solvers})`
   中的SHARED改为STATIC，防止ninja遇到multiple rules for target的问题
   然后执行下述命令，generator可以根据自己的需求选择make或者ninja

```bash
cd Cheetah-Software
mdkir build
cd build
cmake ..
../scripts/make_types.sh
make -j20
```

测试

```bash
./sim/sim
sudo ./user/MIT_Controller/mit_ctrl m s 
```

## 硬件改动

主要包括IMU和电机的通信方式：

1. Unitree SDK 代替 串口通信
2. CSerialPort 打开配置串口与 IMU 通信

### Unitree SDK

在/robot/include/rt下添加rt_usb.h代替原来的rt_spi.h中的函数与定义，对应的src/rt下添加rt_usb.cpp。同时在thrid_party里添加Unitree的SDK安装目录，并在third_party中的CMakeLists.txt中添加Unitree
SDK的路径。然后在lcm-types下面添加`usb_command_t.lcm` `usb_data_t.lcm` `usb_torque_t.lcm`
文件用于取代spi数据结构，将make_types.sh中的/usr/local/share/java/lcm.jar 路径换为/usr/share/java/lcm.jar

### CSerailPort

将CSerialPort的库下载编译安装到third_party中，在CMakeLists.txt中添加CSerialPort目录。在rt_wheetec.cpp中加入读取数据

### HardwareBridge

在HardwareBridge中添加类内成员，同时对initHardware和run等函数进行修改

## 手柄控制

机器狗支持两种控制方式，gamepad手柄或者是usb手柄，前者是通过qt的gamepad模块读取，后者是通过sbus解包得到。第一种方式在Simulation中被使用，只有在调用Qt界面显示的时候才会使用./sim/updateGamepadCommand函数；第二种方式在sim和hardware中通用，无论是Simulation还是Hardware模式，都会执行run_sbus函数如下所示

```cpp
void HardwareBridge::run_sbus() {
    int port = init_sbus(false);  // Hardware
    if (_port > 0) {
        int x = receive_sbus(_port);
        if (x) {
            sbus_packet_complete();
        }
    }
}
```

其中init_sbus打开串口并调用rt_serial中的配置函数为串口进行配置。如果是仿真则端口为"/dev/ttyUSB0"，否则为ttyS4

使用remote controllet就意味着不使用qt的gamepad库，此时就算在仿真界面也要把use_rc设置为1，否则还是只能从gamepad里读取数据

### gamepad手柄数据读取(Xbox)

使用qt的gamepad包来做数据接收，主要操作步骤如下：

### 数据打包

数据的转化步骤关键在DesiredStateCommand中的convertToStateCommands，其中最关键的是得到leftAnalogStick和rightAnalogStick的值，这两个值在后续控制算法中要用到

## TJU_Controller

在user目录下创建TJU_Controller文件夹，作为我们的工作区，下一步先定义我们的控制器所需要的参数，此处先将MIT中不需要的RPC参数等去掉，经实验没有问题。后续的改动类似，琐碎，不再赘述

## 数据可视化

安装了lcm过后，可以使用lcm-spy来查看数据，但是从输出信息上来看少装了一个库

```bash
sudo apt install libjchart2d-java
```

同时找到lcm.jar的位置，并将make_types.sh中的路径改为其路径，比如`cp /usr/share/java/lcm.jar .`

关于lcm工具，他们各自的工具和功能如下：

| 命令 | 作用 |
| --- | --- |
| lcm-gen|生成lcm消息|
|lcm-spy|查看当前数据|
|lcm-logger|记录log|
|lcm-logplayer,lcm-logplayer-gui|log回放|

在使用lcm-spy之前需要设置CLASSPATH环境变量，将lcm.jar的路径加入到CLASSPATH中，然后执行`lcm-spy`即可查看数据。或者直接执行脚本lauch_lcm_spy.sh

## 仿真

改动太多，略

1. 在DrawList中可以增加我们自己的obj文件，从厂商那里拿
2. 在切换user控制器的时候将config中的default-user-parameters-file.yaml改为对应的用户参数文件



