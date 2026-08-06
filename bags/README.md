# R2 底盘测试 bag 仓库

> 统一存放与分析 R2 底盘/IMU/EKF 测试数据。
> **原始数据在 N97** `~/Lin_workspace/r2_integration/bags/`（采集机），本目录为分析副本。
> bag 不入 git（`.gitignore` 已配 `*.bag`）。

## 目录

| 路径 | 内容 |
|:-----|:-----|
| `raw/2026-07-30_r2_chassis_before/` | 修复前对照基准（带 omega 13.2× bug 的代码录制） |
| `raw/2026-08-06_r2_chassis_after/` | 修复后实测（omega + 全向积分修复后） |
| `raw/2026-08-03_imu_test/` | 8-03 IMU ZUPT 测试 |
| `raw/imu_zupt_test.bag` | 7-30 IMU ZUPT 测试（VM 侧副本） |
| `csv/` | 全帧分析导出（t, 轮速x/y/yaw, IMU yaw, 偏差） |
| `analysis/` | 分析脚本（官方 rosbag2_py） |

## bag 清单

| bag | 日期 | 代码版本 | 测试内容 | 备注 |
|:----|:-----|:---------|:---------|:-----|
| r2_turn_test.bag | 07-30 | 修复前 | 转弯 | yaw 偏差最大 179° |
| r2_square_test.bag | 07-30 | 修复前 | 方形闭环 | 闭环误差 1.80m |
| r2_slip_test.bag | 07-30 | 修复前 | 直线滑移 | 闭环误差 5.74m |
| r2_turn_test_after | 08-06 | 修复后 | 转弯 | yaw 偏差 4-14°，KISS 交叉验证一致 |
| r2_square_test_after | 08-06 | 修复后 | 方形闭环 | 闭环 0.27m（KISS 0.24m） |
| r2_slip_test_after | 08-06 | 修复后 | 直线滑移 | 闭环 1.08m（KISS 0.44m） |

## 分析结果摘要（2026-08-06）

| 指标 | 修复前 | 修复后 |
|:-----|:-------|:-------|
| 轮速 yaw vs IMU 最大偏差 | 179°（乱跳） | 7-20°（滑移残余） |
| 方形闭环位移差 | 1.80m | 0.27m |
| EKF z 漂移 | 85m（小时级） | 亚米级（slip 场景 +2.5m 待跟进） |

详见 `doc/retrospect/2026-08-05_chassis_ekf_debug.md`。

## 使用

```bash
# 分析脚本用法（VM，需 source /opt/ros/humble/setup.bash）
python3 analysis/analyze_bags.py raw/2026-07-30_r2_chassis_before/r2_turn_test.bag raw/2026-08-06_r2_chassis_after/r2_turn_test_after
```
