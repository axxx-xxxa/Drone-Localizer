# Drone Localizer

## Preparation

1. input topic

```
1. djistate_msgs::state ( x 1)				-> dji_state
2. sensor_msgs::ImageConstPtr ( x CameraNum)		-> 图像输入
```

2. output topic

```
1./{direction}/point_cloud				-> 点云输出
2./{direction}/xperception_msgs				-> msg输出
3./{direction}/detect					-> 2D检测结果输出
```

## Installation

```bash
1. unzip
2. cd drone_localization_v1.0.0
3. catkin_make
4. source devel/setup.bash
```

## Files

```
config
├── camera{N}.yaml					-> 相机内参文件
├── camera_config.yaml					-> 相机配置文件
└── drone_localizer.yaml				-> 运行参数文件
launch
└── drone_localizer.launch				-> launch文件
model
├── drone0709.onnx					-> detect model (onnx)
└── drone0709.ebin					-> detect model (trt)
rviz
├── drone.rviz						-> old
├── drone_AM3D.rviz					-> old
├── drone_posearray.rviz				-> old
└── drone_deploy.rviz					-> use it
src
├── drone_localizer
│   ├── include/drone_localizer_ros.h			-> ros class ros系统实现
│   ├── include/drone_localizer.h			-> localizer 算法实现
│   ├── include/drone_detector.hpp			-> detector 算法实现
│   ├── include/tensorrt_base.hpp			-> tensorrt 封装
│   ├── include/utils.hpp				-> utils
│   ├── nodes/drone_localizer_node.cpp			-> ros node
|   ├── nodes/drone_localizer_ros.cpp	
│   └── nodes/drone_localizer.cpp
├── djistate_msgs					-
└── xperception_msgs					-

build 							-
devel 							-
install 						-
```

## Quick Start

1. Terminal1

```bash
rosbag play -l ~/2024-09-19-12-53-38.bag
```

2. Teriminal2

```   
roslaunch launch/drone_localizer.launch
```

使用默认配置测试程序能否正常运行 

3. Visualize(rviz)

```
rviz-> File-> Open Config-> ~/config/drone_deploy.rviz
```

## Custom Run

主要需要对以下三个文件进行修改

```
config
├── camera{N}.yaml		-> 相机内参文件
├── camera_config.yaml		-> 相机配置文件
└── drone_localizer.yaml	-> 运行参数文件
```

### camera{N}.yaml

相机内参文件 按照格式填写dist，mtx即可

### camera_config.yaml

以前相机为示例

```
Cameras:
  - name: "front"
    Cam_HAngle2Front: 0 # 相机对于front的yaw夹角
    Cam_VAngle2Horizon: 45 # 相机光心与水平线的夹角
    horizontal_scaling: 1.0 # 横向拉伸系数
    vertical_scaling: 1.0 # 纵向拉伸系数
    undistort: true # 去畸变
    flip: false # 纵向翻转
```

注意：

1. 底部相机(bottom)的Cam_VAngle2Horizon是无效参数
2. 根据实际安装角度调整各个相机的Cam_HAngle2Front及Cam_VAngle2Horizon参数

### drone_localizer.yaml

该文件中有各个参数的简单描述 这里给出一些主要参数的详细解释

```
-- Run
Vertical_Fov			-> 相机竖直方向FOV (按照实际相机参数给出)
Channel_Directions		-> 仅支持 "front" "left" "right" "back" "bottom" 的任意组合 方向可以任意顺序给出 但是ImageTopic{N}会按照给出的方向顺序读取topic Calibration_File_{}会根据方向读取 不受影响
MaxPitch/MaxRoll		-> 如果dji_state给出的Pitch/Roll大于该值 则不会返回任何结果 检测结果会返回空图
StaticHeight			-> (true)按照dji_state给出的ultrasonic计算高度 
					效果是同像素点 xy轴方向 高度越低离中心越近
StaticRotate   			-> (true)需要按照实际的安装角度 对camera_config.yaml.Cam_HAngle2Front修改
					效果是同像素点 会根据Cam_HAngle2Front相对于z轴进行旋转
StaticPoint			-> (true)将图像分为十六宫格 取各个格子中心点作为定位输入 用于测试投影和旋转 实际运行时关闭
					效果是检查各个相机投影的运行状态 可以配合dji_state进行动态检查
StaticRollPitch			-> (true)不考虑dji_state pitch/roll的变化 
					效果是像素不会因为无人机姿态变化而变化 可以打开和关闭进行动态对比 
lr/fb/br/bp			-> 各个相机沿x,y轴点云的缩放比例 可用于投影微调 
					与camera_config.yaml.{}_scaling不同在于 该参数主要通过相机角度缩放整体倍率(bottom除外)
							
-- Debug
ShowDetect: true          # true: 定义2D检测结果publish topic->"/{direction}/detect"
LogTimer: true            # (持续打印 glog无关)		打印各个相机线程推理时间和总耗时日志
LogInfo: true             # (程序运行开头glog打印一次)	打印各个相机输入信息
LogCoord: false           # (大量持续glog打印)		打印各个相机检测结果->定位结果 中间的像素坐标->经纬度日志
LogLevel: 0               # GLOG日志等级 默认0即可
GlogSavePath: ""	  # 如果不为空 glog日志会保存到该文件 默认大小100MB
```

