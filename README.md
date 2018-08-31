#loomo-algodev
This is an Android studio project that enables native C++ development of robotics applications on Loomo robot.

## Description
The official Loomo JAVA SDK is targeting Android developers (https://developer.segwayrobotics.com/), which is not friendly to robotics developer that works with C++ or ROS. This kit builds a bridge between JAVA SDK and C++ interfaces (nienbot_algo::RawData, nienbot_algo::AlgoBase, etc..).

Developers can build a ROS like thread, retrieve sensor data, and execute velocity command to the robot. Third party libraries like OpenCV, Eigen, Tensorflow can also be easily used. Several sample apps are provided (modify MACROS in algo_app\src\main\jni\CMakeLists.txt to switch apps)
- app_test: visualize all loomo sensors in realtime
- app_localmapping: generate local occupancy map by fusing Realsense depth images and robot TF odometry
- app_tensorflow: object recognition based on a custormized-built tensorflow library

The 3rd party library should be downloaded and extracted to dependency\3rdparty_android\. (It might be slow to access outside China, better to be replaced)
https://pan.baidu.com/s/16zqUUY-HJKBKbD976b79Ig

**This project is still in beta phases, and more documentation is needed.**
