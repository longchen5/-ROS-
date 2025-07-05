Ball Tracker ROS System (ball_tracker)
======================================

简介
----
该 ROS 系统使用摄像头检测橙色乒乓球的位置，并通过串口向下位机发送坐标和距离信息。
本系统由 3 个部分组成：
1. USB 摄像头节点（usb_cam）
2. 图像识别节点（ball_tracker.py）
3. 串口控制节点（serial_controller.py）

适配环境
--------
- ROS 环境：建议 melodic 或 noetic
- 摄像头设备：通过 /dev/video0
- 串口设备：通过 /dev/ttyUSB0
- 虚拟机运行需开启 USB 摄像头透传和串口映射

安装依赖
--------
sudo apt install ros-<distro>-usb-cam
pip install opencv-python numpy pyserial

使用方法
--------
1. 插入摄像头与串口模块，确保系统识别：
   ls /dev/video0
   ls /dev/ttyUSB0

2. 启动系统：
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   roslaunch ball_tracker tracker_system.launch

串口发送格式
------------
格式：#{x}${y}*{distance}\r\n
示例：#320$240*28\r\n

注意事项
--------
- 如权限不足，可使用：
  sudo chmod 666 /dev/ttyUSB0
- 虚拟机中务必设置 USB 摄像头透传
- HSV 颜色范围可在 ball_tracker.py 中调整
