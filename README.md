# bw_nav

基于 FAST-LIO2 与 Nav2 的 MID-360 激光雷达移动机器人导航工作空间，覆盖建图、全局 PGO、一致性三维地图重建、Nav2 二维栅格生成、定位与自主导航的完整流程。

## 功能概览

- FAST-LIO2 激光惯性建图，Scan Context 回环检测 + GTSAM 位姿图优化
- PGO 优化后的一致性三维点云重建
- 三维点云到 Nav2 二维栅格（PGM）生成
- 基于 Nav2 的定位、路径规划与自主导航
- 命名点位记录与一键导航

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Livox MID-360 / MID-360S 激光雷达

## 快速开始

以下命令均在工作空间根目录执行（示例路径为 `~/nav`，请按实际部署路径调整）：

```bash
cd ~/nav
./install.sh
```

`install.sh` 会依次完成：

1. 检查并安装 apt / Python / 第三方依赖
2. 发现雷达与主机 IP，写入 Livox 配置
3. 发现底盘串口并写入 udev 规则
4. 执行 `colcon build` 编译工作空间
5. 可选配置 DDS（fastdds / cyclonedds）

> 使用 bash 时，下文所有 `source install/setup.zsh` 请替换为 `source install/setup.bash`。

## 操作流程

### 1. 启动建图

终端 1：

```bash
cd ~/nav
source install/setup.zsh

ros2 launch bw_fastlio_nav2 mid360_pgo_mapping.launch.py \
  rviz:=true
```

使用遥控器缓慢移动机器人，尽量：

- 覆盖需要导航的全部区域
- 多走闭环，回到曾经经过的位置
- 闭环处缓慢通过，避免急转
- 结束时让机器人静止

### 2. 执行全局 PGO

保持终端 1 继续运行，另开终端 2：

```bash
cd ~/nav
source install/setup.zsh

ros2 service call /pgo_batch_optimize std_srvs/srv/Trigger '{}'
```

确认返回 `success: true` 后，回到终端 1 按 `Ctrl+C` 停止建图。

### 3. 重建一致性三维地图

```bash
cd ~/nav
source install/setup.zsh

ros2 run bw_fastlio_nav2 rebuild_pgo_consistent_map \
  --scans-directory maps/pgo_session/Scans \
  --poses maps/pgo_session/optimized_poses_batch.txt \
  --output maps/pgo_session/map_consistent.pcd \
  --voxel-size 0.05 \
  --min-observations 2 \
  --max-range 60
```

### 4. 生成 Nav2 二维栅格

```bash
cd ~/nav
mkdir -p maps/nav2
source install/setup.zsh
ros2 launch bw_fastlio_nav2 generate_nav2_maps.launch.py
```

默认参数已调好，一般直接 launch 即可。

### 5. 启动导航

停止上面的测试进程后：

```bash
cd ~/nav
source install/setup.zsh

# 按网络规划选择，避免与其它 ROS 2 节点重复（范围 0–232）
export ROS_DOMAIN_ID=42
```

> `ROS_DOMAIN_ID` 用于隔离同一网络中的 DDS 通信。如果与其它 ROS 2 节点（仿真环境、其他机器人、调试工具等）使用相同 ID，彼此的节点发现和话题流量会互相干扰，可能导致启动变慢、定位异常或导航性能下降。RViz、`./points` 等所有相关终端都需要设置相同的 `ROS_DOMAIN_ID`。

单底盘：

```bash
ros2 launch bw_fastlio_nav2 mid360_nav2_localization.launch.py \
  enable_motion:=true protocol_version:=v3 robot_version:=2.0 \
  mantis_protocol_version:=v3_chassis \
  baud_rate:=4000000 serial_frequency:=1000 rviz:=true
```

整机：

```bash
ros2 launch bw_fastlio_nav2 mid360_nav2_localization.launch.py \
  enable_motion:=true \
  protocol_version:=v3 \
  robot_version:=2.0 \
  mantis_protocol_version:=v3 \
  port_name:=/dev/bw_chassis \
  baud_rate:=4000000 \
  serial_frequency:=1000 \
  rviz:=true
```

主要参数：

| 参数 | 说明 |
| --- | --- |
| `enable_motion` | 是否允许底盘运动 |
| `protocol_version` | 底盘协议版本，当前为 `v3` |
| `robot_version` | 机器人版本，当前为 `2.0` |
| `mantis_protocol_version` | 单底盘为 `v3_chassis`，整机为 `v3` |
| `port_name` | 整机底盘串口，udev 规则生成的 `/dev/bw_chassis` |
| `baud_rate` | 串口波特率 `4000000` |
| `serial_frequency` | 串口通信频率 `1000` |
| `rviz` | 是否启动 RViz |

### 6. 点位记录与导航

导航程序启动后，在设置了相同 `ROS_DOMAIN_ID` 的终端、工作空间根目录执行：

```bash
./points          # 记录当前位姿，按提示确认并输入点位名称
./points 厨房     # 前往已保存的点位
```

- 每次前往点位前，会先取消 Nav2 上正在执行的所有导航目标
- 点位保存在 `maps/points.json`
- 导航结果只写入 `log/points.log`

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `src/bw_bringup` | 系统启动与 bringup |
| `src/bw_fastlio_nav2` | FAST-LIO2 + Nav2 集成，建图/PGO/栅格生成 launch |
| `src/bw_interface` | 自定义消息与服务接口 |
| `src/bw_serial` | 底盘串口通信 |
| `src/bw_utils` | 通用工具库 |
| `src/bw_std_description` | 机器人 URDF 与网格模型 |
| `src/fastlio_lc_pgo` | Scan Context + GTSAM 回环检测与位姿图优化 |
| `install.sh` | 一键部署脚本 |
