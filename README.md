# GarbageSortingTrolley(光电垃圾分拣车)

## 1. 文件结构
```python
.
├── CMakeLists.txt          #CMAKE编译配置文件
├── include                 #头文件存放文件夹
│   ├── control.h               #控制类头文件（未使用）
│   ├── data.h                  #数据类头文件
│   ├── defs.h                  #定义类头文件
│   ├── detect.h                #检测类头文件
│   ├── log.h                   #输出LOG类头文件
│   ├── realSenseDriver.h       #相机驱动头文件
│   ├── serial.h                #串口头文件
│   ├── states.h                #状态类头文件
│   └── systime.h               #系统时间头文件
├── main.cpp                #主程序
├── README.md               #README
├── resource                #资源文件夹
│   ├── angle_map.jpg           #像素-角度映射表
│   ├── distance_map.jpg        #像素-距离映射表
│   ├── param.xml               #参数配置文件
│   ├── yolov4.cfg              #网络结构文件
│   └── yolov4.weights          #网络权重文件
├── src                     #源代码文件存放文件夹
│   ├── control.cpp             #控制类源代码文件（未使用）
│   ├── data.cpp                #数据类源代码文件
│   ├── detect.cpp              #检测类源代码文件
│   ├── realSenseDriver.cpp     #相机驱动类源代码文件
│   ├── serial.cpp              #串口类源代码文件
│   └── states.cpp              #状态类源代码文件
└── test.cpp                #测试文件
```
## 2. 项目特性
-   **参数配置方便**。基本参数配置基本都在defs.h，data.cpp中完成，包括图像尺寸、状态种类、垃圾种类、运动方向、相机型号、参数路径等等。
-   **单线程执行**。但也可以方便地改成多线程执行，实际上control类在设计之初的设想就是为了多线程执行，将控制部分剥离出来，但实际上并不需要多线程执行故未使用。
-   **使用TensorRT加速**。最终版本中无TensorRT加速因为使用了两个模型进行推理，加速效果并不明显而且对TensorRT的使用并不熟悉，改成TensoRT加速的话可以参考：https://github.com/enazoe/yolo-tensorrt 这个上手比较快。
## 3. 其他
至于具体的控制逻辑只能看代码了，代码里的注释比较多，应该没问题，附上一张在写代码之处画的状态转换图，图中有些地方已经不符合现在的逻辑了，但可以作为参考。

![状态图](https://github.com/luojunhui1/GarbageSortingTrolley/StatesMachine.png)
