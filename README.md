# maze_mouse —— 地老鼠走迷宫系统（ROS2 + Gazebo 仿真）

课程代码：(2026-2027-1)-S0610350《机器人感知与控制综合实践》综合设计项目

基于 TurtleBot3 Burger 的迷宫走通仿真：自定义 Gazebo 迷宫环境，实现开环控制、
里程计 P 闭环、雷达分区避障、左墙跟随走迷宫（反应式），以及 Pure Pursuit
纯跟踪（跟踪式）与速度自适应改进的对比实验。

## 实验结果

| 方案 | 类型 | 速度策略 | 走通用时 |
|---|---|---|---|
| 左墙跟随（基础版） | 反应式 | 0.16 m/s 恒定 | 85.4 s |
| 左墙跟随（自适应版） | 反应式 | 0.10~0.22 m/s 分档 | 81.8 s |
| Pure Pursuit | 跟踪式（需预知路径） | 0.20 m/s | 60.7 s |

结论：跟踪式轨迹平滑、速度最快，但依赖先验路径；反应式无需地图、
鲁棒性强，代价是路线绕行、速度受限。自适应调速提升有限（约 4%），
因为转向与找墙时间在全程中占比高，直行段提速空间小。

## 环境与依赖

- Ubuntu 22.04 + ROS2 Humble + Gazebo 11（虚拟机可运行，建议 4C8G）
- 依赖安装：
  ```bash
  sudo apt install ros-humble-turtlebot3-description \
                   ros-humble-turtlebot3-gazebo \
                   ros-humble-turtlebot3-teleop
  echo "export TURTLEBOT3_MODEL=burger" &gt;&gt; ~/.bashrc && source ~/.bashrc
