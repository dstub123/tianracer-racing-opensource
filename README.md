# Tianracer Autonomous Racing (纯激光与航点混合自动驾驶竞速算法)

> 🏎️ **Current Best Lap**: 14.4s (Stable: 14.59s) / **当前稳定圈速**: 14.59s/圈，0 碰撞

## 📖 Introduction (项目简介)
This project is an open-source implementation for the Tianracer autonomous racing platform, utilizing a hybrid approach of Follow-the-Gap (Lidar) and Waypoint tracking (Pure Pursuit). 
本项目基于 Tianracer 平台进行二次开发，采用纯激光雷达（Follow-the-Gap）与航点追踪（Pure Pursuit）混合算法。

## ⚠️ Tuning Pitfalls & Guide (核心修改与避坑指南)
- **Wheelbase parameter (`wheelbase=0.42`)**: Keep it as 0.42. Do NOT change it to the physical value 0.265, which will cause understeer and make the car 0.32s slower. (轴距参数 0.42 是经验值，千万别改成物理值 0.265，实测会慢 0.32s)。
- **Speed limiters (`speed`)**: The `speed` field in the YAML is just a speed cap, not the target speed. The actual speed is heavily bounded by the front wall distance (`front_min`). (speed 只是限速帽，真实速度由前墙距离决定)。
- **Slalom Physical Limit (蛇形段物理瓶颈)**: The gap algorithm requires ~44° steering, but the car's Ackermann limit is ~30° (clip 0.5). Therefore, slalom always has understeer and oscillation. (蛇形段受物理转向限制，参数调优已达上限)。

## ⚙️ Dependencies (环境依赖)
- ROS Noetic
- Gazebo
- Python 3

## 📜 License (开源协议)
This project is licensed under the GPL-3.0 License. Based on the original work by Tianbot.
