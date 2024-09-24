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
注意 zip包代码在x86平台编译 如果在ORIN编译 需要修改src/drone_localizer/CMakeLists.txt 搜索 "# modify x86" 将下两行的注释互换 (对应的依赖改为ORIN对应的路径)

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

## Quick Start （using 2024-09-19-12-53-38.bag）
1. Modify File Path in drone_localizer.yaml
```bash
Calibration_File_{} 	x N
ImageTopic 		x N
CameraConfig
DetectModel
```

3. Terminal1

```bash
rosbag play -l ~/2024-09-19-12-53-38.bag
```

3. Teriminal2

```   
roslaunch launch/drone_localizer.launch
```

使用默认配置测试程序能否正常运行 正常运行打印日志如下
```
I0924 11:02:09.353560 920298 drone_localizer_ros.cpp:236] [CameraConfig] front Cam_HAngle2Front: 0 Cam_VAngle2Horizon: 45 Horizontal_Scale: 1 Vertical_Scale: 1 undistort: 1 flip: 0
I0924 11:02:09.355885 920298 drone_localizer_ros.cpp:236] [CameraConfig] left Cam_HAngle2Front: 90 Cam_VAngle2Horizon: 45 Horizontal_Scale: 1 Vertical_Scale: 1 undistort: 1 flip: 1
I0924 11:02:09.357946 920298 drone_localizer_ros.cpp:236] [CameraConfig] right Cam_HAngle2Front: -90 Cam_VAngle2Horizon: 45 Horizontal_Scale: 1 Vertical_Scale: 1 undistort: 1 flip: 1
I0924 11:02:09.359985 920298 drone_localizer_ros.cpp:236] [CameraConfig] back Cam_HAngle2Front: 180 Cam_VAngle2Horizon: 45 Horizontal_Scale: 1 Vertical_Scale: 1 undistort: 1 flip: 1
I0924 11:02:09.362033 920298 drone_localizer_ros.cpp:236] [CameraConfig] bottom Cam_HAngle2Front: -90 Cam_VAngle2Horizon: 90 Horizontal_Scale: 2 Vertical_Scale: 0.7 undistort: 1 flip: 0
I0924 11:02:09.362044 920298 drone_localizer_ros.cpp:25] [ROS NODE LOG] DroneLocalizerRos init done
I0924 11:02:09.372550 920298 drone_localizer_ros.cpp:63] [Run] Sub and Pub Channels init done
I0924 11:02:09.373896 920298 drone_localizer.cpp:144] [Localizer Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/config/camera1.yaml
I0924 11:02:09.812831 920298 drone_detector.hpp:109] [Detector Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/model/drone0709_x86.ebin
I0924 11:02:09.812975 920298 drone_localizer.cpp:144] [Localizer Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/config/camera3.yaml
I0924 11:02:10.179229 920298 drone_detector.hpp:109] [Detector Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/model/drone0709_x86.ebin
I0924 11:02:10.179373 920298 drone_localizer.cpp:144] [Localizer Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/config/camera0.yaml
I0924 11:02:10.549350 920298 drone_detector.hpp:109] [Detector Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/model/drone0709_x86.ebin
I0924 11:02:10.549500 920298 drone_localizer.cpp:144] [Localizer Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/config/camera4.yaml
I0924 11:02:10.920450 920298 drone_detector.hpp:109] [Detector Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/model/drone0709_x86.ebin
I0924 11:02:10.920595 920298 drone_localizer.cpp:144] [Localizer Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/config/camera2.yaml
I0924 11:02:11.288950 920298 drone_detector.hpp:109] [Detector Init done] /media/steven/projects/Drone/deploy/drone-project-202409/v1.4/model/drone0709_x86.ebin
I0924 11:02:11.389549 920313 drone_localizer_ros.cpp:257] [Infer_From_Real] Start !
I0924 11:02:11.489663 920314 drone_localizer_ros.cpp:257] [Infer_From_Real] Start !
I0924 11:02:11.589886 920315 drone_localizer_ros.cpp:257] [Infer_From_Real] Start !
I0924 11:02:11.690358 920316 drone_localizer_ros.cpp:257] [Infer_From_Real] Start !
I0924 11:02:11.790421 920298 drone_localizer_ros.cpp:82] [Run] Det and Loc Channels init done
I0924 11:02:11.790469 920317 drone_localizer_ros.cpp:257] [Infer_From_Real] Start !
```

4. Visualize(rviz)

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
DetectModel			-> 填写onnx路径 如果无同名称".ebin"后缀模型 会自动进行转换 日志会打印[Detector Convert Model]及[Detector Convert Model done]信息
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

