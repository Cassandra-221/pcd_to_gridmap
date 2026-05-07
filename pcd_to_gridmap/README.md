# PCD to 2D Grid Map

将三维点云（PCD 格式）投影到 XOZ 平面，自动检测地平面并生成带有障碍物/未知/自由标记的二维栅格地图。输出文件为 ROS 兼容的 `gridmap.pgm` 和 `gridmap.yaml`。

## 功能

* 支持多种地面检测方法：RANSAC、MLESAC、MSAC
* 根据相对高度过滤非地面点，提取潜在障碍物
* 按可配置的栅格分辨率将点云向下投影，生成占据栅格图
* 自适应阈值判定障碍物（基于栅格内点云密度的百分比）
* 输出标准 PGM（二进制灰度图）与对应的 YAML 描述文件

## 依赖

* Ubuntu 18.04 / 20.04 / 22.04（其他 Linux 发行版类似）
* g++ 支持 C++14
* CMake ≥ 3.10
* PCL (Point Cloud Library) ≥ 1.10

安装 PCL（Ubuntu）：
sudo apt update
sudo apt install libpcl-dev

