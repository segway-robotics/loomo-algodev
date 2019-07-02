# loomo-algodev
This is an Android studio project that enables native C++ development of robotics applications on Loomo robot.

## Description
The official Loomo JAVA SDK is targeting Android developers (https://developer.segwayrobotics.com/), which is not friendly to robotics developer that works with C++ or ROS. This kit builds a bridge between JAVA SDK and C++ interfaces (nienbot_algo::RawData, nienbot_algo::AlgoBase, etc..).

Developers can build a ROS like thread, retrieve sensor data, and execute velocity command to the robot. Third party libraries like OpenCV, Eigen, Tensorflow can also be easily used. Several sample apps are provided (modify MACROS in **algo_app\src\main\jni\CMakeLists.txt** to switch apps)
- app_test: visualize all loomo sensors in realtime
- app_localmapping_test: generate local occupancy map by fusing Realsense depth images and robot TF odometry
- app_tensorflow_sample: object recognition based on a custormized-built tensorflow library
- app_socket: communicate with remote computers using sockets 

The 3rd party library should be downloaded and extracted to **dependency\3rdparty_android**:

https://drive.google.com/drive/folders/1FSVhPz7ZhSJqS4sTjBbKZDFhiEfk5ofH

**This project is still in beta phases, and more documentation is needed.**

## Dependency 
- Python 
- CMake 

