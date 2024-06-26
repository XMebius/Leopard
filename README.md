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

### MIT Cheetah编译

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

开始编译

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

## 运行TJ Leopard

将config中的default-user-parameters-file.yaml改为对应的用户参数文件"leopard-userParam-default.yaml"

```bash
./sim/sim
sudo ./user/TJ_Leopard/tj_ctrl l s
```

如果是硬件运行

```bash
sudo ./user/TJ_Leopard/tj_ctrl l r f
```
