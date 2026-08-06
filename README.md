# Lin_workspace 总览

> Ubuntu 22.04 + ROS2 Humble + 嵌入式开发环境。
> 包含 R2 机器人集成、电机驱动、视觉、SLAM、数据仓库等。

## 目录导航

### 项目 / 代码

| 目录 | 说明 |
|:-----|:-----|
| [r2_integration/](r2_integration/) | **R2 集成项目（权威源）**：底盘/IMU/EKF/雷达建图。文档见 `r2_integration/doc/` |
| [vision/](vision/) | 视觉识别：AstraPro 深度摄像头，跟踪+预测排球轨迹并输出像素坐标+距离（可扩展为三维坐标+落点预测） |
| [control/](control/) | 车体底盘控制（Linux，串口即用）：`R2.py`、`motor_control.py` 等 |
| [command/](command/) | SavvyCAN 对接脚本 + CAN 工具（`can_command.py`、`chassis_control.py`、`auto_measure_ticks.py`） |
| [imu_odom_ws/](imu_odom_ws/) | IMU 里程计 ROS2 工作区 |
| [vlp16_slam_ws/](vlp16_slam_ws/) | VLP-16 激光雷达 SLAM 工作区 |
| [6_Mpu6050t1_ws/](6_Mpu6050t1_ws/) | MPU6050 相关工作区 |

### 电机 / 硬件

| 目录 | 说明 |
|:-----|:-----|
| [robstride_rs00/](robstride_rs00/) | Robstride RS00 电机/关节模组资料 |
| [Lin_rs00/](Lin_rs00/) | RS00 集成项目（详见其 README） |
| [unitree_goM80106/](unitree_goM80106/) | Unitree Go M80106 电机 |
| [librealsense/](librealsense/) | Intel RealSense 源码/工具 |
| E34-2G4H27D_UserManual_CN_v1.1.pdf | 无线透传模块用户手册 |

### 数据 / 仓库

| 目录 | 说明 |
|:-----|:-----|
| [bags/](bags/README.md) | **R2 底盘测试 bag 仓库**：raw（修复前/后）+ csv（分析导出）+ analysis（脚本） |
| [Lin_data/](Lin_data/) | CAN 数据 CSV |
| [frames/](frames/) | TF 树导出（.gv） |

### 比赛 / 文档 / 配置

| 路径 | 说明 |
|:-----|:-----|
| [260612目前/](260612目前/) | ROBOCON 2026 比赛资料（主赛/四足） |
| [CLAUDE.md](CLAUDE.md) | 本工作区 Claude 配置 |
| [fastdds_peer_n97.xml](fastdds_peer_n97.xml) | VM→N97 跨机 DDS 单播配置 |
| terminator_history.md / session-export.txt / One_ClickBurning.md | 历史操作记录/备忘 |

## 说明

- **README 只做导航**，各项目细节见对应子目录文档（`r2_integration` 文档规范见其 `doc/standards.md`）
- 本目录受 git 管理（`r2_integration` 为独立仓库，其他目录有独立 `.git` 或忽略）
- bag 等大文件不入库
