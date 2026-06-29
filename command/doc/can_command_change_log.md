# can_command.py 改动记录

**日期：** 2026-06-22

## 改动内容

### 1. 固定 CAN 接口名为 `can0`
- **之前：** 每次启动计数器自增，生成 `can0` → `can1` → `can2` ... 并在 `.can_counter.json` 中持久化
- **之后：** 固定使用 `can0`

### 2. 启动前自动清理旧接口
- 新增清理步骤：`ip link set can0 down` + `pkill slcand`，确保每次重建干净的 `can0`

### 3. 删除冗余代码
- 删除 `_COUNTER_FILE` 常量和 `_get_and_increment_count()` 函数
- 删除不再需要的 `import json`、`import os`

## 原因
- 自增接口名没有实际收益，每次启动数字变大反而造成困扰
- 旧接口不清理会越积越多
- 当前没有同时接入多个 CAN 设备的需求

## 效果
- 每次运行 `CanCmd` 流程：选设备 → 清理旧 `can0` → 重建 `can0` → 启动 SavvyCAN
- 接口名永远固定为 `can0`，简洁可预测
