# Extended Kalman Filter Project

## 1. 原项目地址

本项目源自Udacity的Self Driving Car Engineer课程。

原项目地址为：[udacity/CarND-Extended-Kalman-Filter-Project: Self-Driving Car Nanodegree Program Starter Code for the Extended Kalman Filter Project (github.com)](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

项目模拟了一辆行驶的汽车以及在汽车边上的一辆自行车的场景。并且提供了一份数据集，包含了车上不同传感器探测到的自行车的位置信息。项目要求需要通过“增强卡尔曼滤波器Extended Kalman Filter”进行“感知融合Sensor Fusion”后得到自行车的位置，并通过模拟器Simulator进行可视化的展示。

## 2. 模拟器

为了进行可视化展示，项目提供了一个[模拟器Simulator](https://github.com/udacity/self-driving-car-sim/releases "点击可跳转至下载链接")，可以直接下载到本地安装。

## 3. uWebSockets

如果只把原项目和模拟器下载到本地的话，大概率是编译不了的，原项目的main.cpp文件中的#include `<uWS/uWS.h>大概率是找不到的，`为了能让程序顺利编译，同时为了能让代码生成的结果和模拟器连接，还需要使用uWebSocketIO。但这玩意在windows上很麻烦。可以用vcpkg来下载安装，但我反正没成功，各种坑，多半和开发环境，工具链配置有关，最后选择了用Ubuntu Bash在windows上运行虚拟机来进行本地编译。

### 安装Ubuntu Bash

但这个软件只能用在Linux和Mac OS上，Windows上直接使用比较麻烦。我使用的是Window 10的系统，则需要对开发环境进行特殊的配置，通过安装Linux Bash来获得Linux的环境，从而更好的使用uWebSocketIO，点这里可以查看Win10上安装Ubuntu的[安装步骤](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/ "点击可查看安装步骤")。

### 安装uWebSockets所需依赖库

安装好Ubuntu后，需要通过apt-get安装几个uWebSockets要的依赖库，包括cmake，libssl-dev，openssl之类的（之后编译的时候可能会弹出些错误，缺啥就装啥了，需要注意看log）

### 安装uWebSockets

安装好依赖库后，原项目的文件夹有一个install-ubuntu.sh的文件，直接运行应该是会自动安装好uWebSockets的，但我试了试并没有反应，所以就手动安装了。

回到用户目录下，通过git把uWebSockets的github仓库克隆下来，然后在uWebSockets的仓库的文件夹里面先用 `mkdir build`新建一个build文件夹，然后进到build里，然后用 `cmake.. && make`编译并运行一下，这样uWebSockets就算安装好了。过程会出几次错，一般是少了一些必要的库，用apt-get安装后再试试就行了，可能是g++，可能是libuv1.dev

### 试编译项目

为了检验各种依赖库是不是都装好了，可以再到项目的文件夹里尝试编译一下，可以编译并生成./ExtendedKF文件的话就没问题了。

> 此外本项目还需要使用到由奔驰提供的用于生成并可视化用于感知融合Sensor Fusion的应用，可以生成更多的数据，用于卡尔曼滤波器的学习，项目地址为：[udacity/CarND-Mercedes-SF-Utilities: Tools for Sensor Fusion processing. (github.com)](https://github.com/udacity/CarND-Mercedes-SF-Utilities)

## 4. 项目任务

### 本地开发

配置好环境后就可以愉快的进行本地开发了。虽然再Udacity的网站里可以通过他们提供的workspace特别简单的进行项目，所有环境都是配置好了的，我还是选择在本地开发了。一是因为网络问题，即不太稳定也不想一直连着网在线写代码；二是自己建立仓库再本地开发能更熟悉从搭建环境到编译运行的全过程；三是可以通过自己的仓库记录下整个项目过程，也方便以后回头来复习或者分享给别人。

另外需要注意的是，我个人不是很会在Ubuntu里写代码，于是还是用的VS code编辑器来编辑代码，然后通过git插件把代码同步到自己的仓库里，然后再回到Ubuntu Bash里pull一下，然后再编译看看结果。

需要完成代码的文件有FusionEKF.cpp，FusionEKF.cpp，FusionEKF.h，kalman_filter.cpp，kalman_filter.h， tools.cpp和tools.h，main.cpp是原项目提供的，无需修改。

### kalman_filter.cpp以及kalman_filter.h

未完待续

### FusionEKF.cpp及FusionEKF.h

未完待续

### tool.cpp及tools.h

未完待续
