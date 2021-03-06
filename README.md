# Bachelor Project项目简介
- 课题名称：面向人机物协同的软件自动生成技术
- 主要使用到的核心软件包括ROS-FPrime-LTLMop-Gazebo
- 本项目主要包括三个部分
  -  Gazebo仿真环境相关配置：包括无人机，无人车以及地图，还有相关ROS控制代码
  - LTLMop扩展实现：扩展实现了FPBuilder（修改自动生成代码只要修改`src/lib/FPBuild.py`就行）
  - FPrime无人控制系统：主要是可运行的单机器人无人控制系统，以及一个多机协同的模板
  
# 项目中FPrime介绍
- fprime_recap：无人机巡逻搜救（可以直接运行）
- fprime_multi：无人机和无人车（未完全实现，模板可编译）

# 运行单机器人无人控制系统步骤
- 将默认的fprime替换为fprime_recap并且完成编译
- 地图和机器人仿真启动：`roslaunch patrol_mission detect_base_world.launch`
- 启动用于控制起飞降落的ROS结点：`roslaunch takeoff_land takeoff_land.launch`
- 启动用于控制无人机前进的ROS结点：`rosrun rescap_controller move_controller.py`
- 启动用于控制无人机拍照的ROS结点：`rosrun image_view image_saver image:=/ResCap/downward_cam/camera/image _save_all_image:=false _filename_format:=/home/nonsense/Pictures/ResCap_img/foo.jpg __name:=image_saver`
- 启动fprime：`fprime-gds`
