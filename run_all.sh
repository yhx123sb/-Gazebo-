#!/bin/bash
# 一键自动运行Gazebo仿真+轨迹跟踪控制

# 自动检测并启动roscore（如未启动）
if ! pgrep -x "roscore" > /dev/null
then
  gnome-terminal --tab -- bash -c "roscore"
  sleep 3
fi

# **全局设置使用仿真时间**
gnome-terminal --tab -- bash -c "rosparam set /use_sim_time true"
sleep 1

# 启动Gazebo仿真环境
gnome-terminal --tab -- bash -c "source ~/catkin_ws5/devel/setup.bash; roslaunch --screen mickrobot_gazebo bringup.launch"
sleep 10

# 启动RViz用于可视化
gnome-terminal --tab -- bash -c "source ~/catkin_ws5/devel/setup.bash; rosrun rviz rviz -d ~/catkin_ws5/src/mickrobot_gazebo/config/path_following.rviz"
sleep 3

# 启动路径发布节点
gnome-terminal --tab -- bash -c "source ~/catkin_ws5/devel/setup.bash; rosrun mickrobot_gazebo clicked_path_publisher.py"
sleep 2

# 启动路径跟踪控制器
gnome-terminal --tab -- bash -c "source ~/catkin_ws5/devel/setup.bash; rosrun mickrobot_gazebo path_follower.py"

# 启动Pure Pursuit 控制器
gnome-terminal --tab -- bash -c "source ~/catkin_ws5/devel/setup.bash; rosrun mickrobot_gazebo pure_pursuit_follower.py"

echo "所有组件已启动！"
echo "使用说明："
echo "1. 在RViz中使用'Publish Point'工具在地图上点击来创建路径点"
echo "2. 路径点会自动连接成路径"
echo "3. 机器人会自动开始跟踪路径"
echo "4. 当机器人到达最后一个点时，会自动停止"