# multi_round_relocalization

把"一轮 BBS + NDT 打分"换成"多轮 BBS + 概率聚合"的离线评估实验。
不依赖 ROS，只依赖 `numpy` + 遗留的 `g2_ndt_score` 扩展（NDT 部分可选）。

结论见仓库根目录 `多轮BBS概率重定位可行性评估.md`。

## 数据

`data/rounds.json` 是 87 帧真实扫描（`maps/pgo_session/Scans/*.pcd`，真值 `optimized_poses.txt`）
在生产 2D 地图上跑 6 轮 BBS 的原始结果。每轮 = 生产参数（10°、depth 4、NMS 2 m、1 格膨胀、
8 候选）+ 单帧内扰动：

- 第 0 轮保留生产用的 `linspace` 等距抽样（与线上逐位一致）
- 第 r>0 轮随机抽 256 个体素唯一点，并把扫描航向偏置 δ_r = r·10°/6

每条记录：`{frame, round, cands: [[x, y, yaw_deg, bbs_score], ...], truth, truth_score, dt}`。

## 脚本

| 脚本 | 作用 |
| --- | --- |
| `run_rounds.py` | 重新生成 `rounds.json`（生产轮 + 扰动轮，可指定帧范围/轮数/候选数） |
| `evaluate.py` | 单轮统计、投票曲线、`(R, k)` 网格、置信度校准；`--ndt` 时做 top-3 复判 |
| `compare_variants.py` | `bbs / ndt_raw / ndt_ref / vote / vote_ref / vote+ndt` 六种排序发布方案对比 |

`common.py` 是共享的聚类/判定 helper：

- 判定容差 `TRANSLATION_TOLERANCE_M = 0.75`、`YAW_TOLERANCE_DEG = 12`（BBS 航向网格是 10°）
- 聚类门限 `CLUSTER_RADIUS_M = 2.0`、`CLUSTER_YAW_DEG = 20`
- 聚类：按 BBS 分降序贪心立簇，簇分 = 参与轮数，并列比簇内最高 BBS 分，发布簇内中位数位姿

## 复现

```bash
P=nav_1/nav/src/lidar_localization_ros2
cd $P/experiments/multi_round_relocalization

# 1) 纯 BBS 聚合统计（不需要 NDT 扩展）
python3 evaluate.py --rounds data/rounds.json

# 2) 排序/发布方案对比（需要 g2_ndt_score 扩展）
python3 compare_variants.py \
  --scans /home/pgd/bw_nav/nav_1/nav/maps/pgo_session/Scans \
  --ndt-map /home/pgd/bw_nav/nav_1/nav/maps/pgo_session/map_consistent.pcd \
  --ndt-module-dir /home/pgd/bw_nav/.probe/bbs
```

回归测试（不需要 NDT）：

```bash
cd nav_1/nav/src/lidar_localization_ros2
PYTHONPATH=$PWD python3 test/test_multi_round_relocalization_experiment.py
```

## 关键数字（87 帧）

| 方案 | top1 正确率 |
| --- | --- |
| 现状：1 轮 8 候选，按 NDT fitness 排序，发布原始格点位姿 | 79.3% |
| 1 轮 + 精修位姿（`registration_refine_candidates=True`） | 97.7% |
| 5 轮投票（k=6） | 94.3% |
| 4～5 轮投票 + NDT 复判 top-3 | 97.7% |
| 6 轮投票（k=4）+ NDT 复判 | 100% |

单轮 BBS 耗时（单进程，Python 后端）：k=4 3.71 s / k=5 3.87 s / k=8 4.24 s —— 耗时由轮数决定。
