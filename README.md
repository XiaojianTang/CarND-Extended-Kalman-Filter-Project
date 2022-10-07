# Extended Kalman Filter Project

本项目源自Udacity的Self Driving Car Engineer课程。

原项目地址为：[udacity/CarND-Extended-Kalman-Filter-Project: Self-Driving Car Nanodegree Program Starter Code for the Extended Kalman Filter Project (github.com)](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

项目模拟了一辆行驶的汽车以及在汽车边上的一辆自行车的场景。并且提供了一份数据集，包含了车上不同传感器探测到的自行车的位置信息。项目要求需要通过“增强卡尔曼滤波器Extended Kalman Filter”进行“感知融合Sensor Fusion”后得到自行车的位置，并通过模拟器Simulator进行可视化的展示。

为了进行可视化展示，项目提供了一个[模拟器Simulator](https://github.com/udacity/self-driving-car-sim/releases "点击可跳转至下载链接")，可以直接下载到本地安装。

同时为了能让代码生成的结果和模拟器连接，还需要使用uWebSocketIO，但这个软件只能用在Linux和Mac OS上，Windows上直接使用比较麻烦。我使用的是Window 10的系统，则需要对开发环境进行特殊的配置，通过安装Linux Bash来获得Linux的环境，从而更好的使用uWebSocketIO，点这里可以查看[安装步骤](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/ "点击可查看安装步骤")。后来发现这样其实也挺麻烦，得把整个项目搬到虚拟机里去做，我又没怎么接触过Linux，估计用起来也很痛苦，所以作罢了。

后来决定还是用Visual Studio来完成项目，参考了fkeidel的[ide-profile-VisualStudio](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio.git "点击跳转至Github仓库")，配置好后就可以直接用Visual Studio开发了，不需要在用Ubuntu Bash来折腾了。

此外本项目还需要使用到由奔驰提供的用于生成并可视化用于感知融合Sensor Fusion的应用，可以生成更多的数据，用于卡尔曼滤波器的学习，项目地址为：[udacity/CarND-Mercedes-SF-Utilities: Tools for Sensor Fusion processing. (github.com)](https://github.com/udacity/CarND-Mercedes-SF-Utilities)
