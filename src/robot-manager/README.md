# robot_manager

#### 介绍
c++版本的中控后端，jtcx_robot的c++版本。目前的初级版本，将完全按照python版本的后端程序jtcx_robot的框架重写，即将后台功能分为三个node (三个独立的进程)进行编写，后期可能会根据需求合并成一个node（一个进程，多个线程）的方式。

#### 软件架构
按照功能，有以下三个node

- behavior

  该node位于behavior文件夹，主要功能是调度robot的各种功能模块，如自主导航，自动充电，轨迹跟踪等等。

- service_and_map_manager

  该node位于check_services文件夹，主要功能是监控robot的各种功能模块的程序是否正常运行，以及地图服务的相关管理（如编辑地图，保存地图，删除地图等）

- ui_param

  该node位于params_changer文件夹，主要功能是响应前端(webapp)的关于各种参数（初始位姿，充电点，虚拟墙，路径点等）的相关操作（如加载，保存，读取等）。

为了使用方便，使用roslaunch一次性同时加载以上三个node, launch文件位于launch文件夹

#### 依赖

- ros
- mobile_platform_msgs

#### 开发说明

目前改程序仍在开发中，为了保证开发流程，建议写完一个小功能，需要用测试案例测试。test文件夹为存放测试案例的地方。

```bash
# cd到工作空间
cd 9tian_ws
# 编译程序
catkin_make
# source编译好的可执行文件
source 9tian_ws/devel/setup.bash
# 启动 behavior node
rosrun robot_manager behavior
# 启动 service_and_map_manager node
rosrun robot_manager params_changer
# 启动 ui_param node
rosrun robot_manager check_services
# 同时启动所有node
roslaunch robot_manager robot_manager.launch

```

**注**：开发阶段，建议不要roslaunch所有节点，建议每个node要分branch独立开发，独立测试，每个node里面的每个function也要独立测试。

#### TODO
建图时增加录制轨迹