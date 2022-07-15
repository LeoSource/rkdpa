# 机器人运动学动力学以及规划算法
本仓库包含串联机器人（六轴机械臂）的运动学和动力学建模，笛卡尔空间和关节空间的轨迹规划，以及机器人阻抗和导纳控制算法(`robot kinematic dynamic planning algorithm`)，并建立了simscape模型进行可视化仿真。

## 运行环境
Matlab2018b

## 依赖
- Robotics ToolBox    --  v10.4
- MQTT in MATLAB  --  v1.4
- Real-Time Pacer for Simulink    --  v1.0.0.1

## 架构说明
1. docs/  
机械臂运动学仿真文档，包含机械臂建模，正逆运动学，轨迹规划等内容
2. src/cfiles/  
机械臂运动控制仿真C++实现
3. src/classes/  
机械臂运动控制封装代码，主要包含多种轨迹规划与机械臂运动学相关的class
4. src/simulink/  
基于simscape toolbox搭建的机械臂可视化模型
5. src/data/  
数据存放区域
6. src/tools/  
基础功能函数

