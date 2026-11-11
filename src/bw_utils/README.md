# bw_utils - Engineering Specification & README

## 1. 概述 (Overview)
`bw_utils` 是 BlueWorm 遥操作系统的 **通用工具与配置管理层**。它提供了一套跨语言（C++ 与 Python）的工具函数、网络质量评估逻辑以及统一的硬件配置参数管理。该包的核心特性是通过 YAML 配置文件自动生成 C++ 和 Python 代码，确保了系统各层级在处理控制器索引时的高度一致性；同时提供轻量级系统辅助节点用于机器人身份广播。

---

## 2. 框架与职责边界 (Framework & Boundaries)

### 2.1 核心定位
作为系统的 **底层支撑库 (Utility Layer)**，它被 `bw_teleop`、`bw_vr_input` 和 `bw_core` 等多个包依赖。除工具函数外，本包还提供一个系统辅助节点 `sn_publisher_node`，用于机器人身份发现广播。

### 2.2 职责范围 (In-Scope)
- **跨语言配置同步**: 通过 `controller_indices.yaml` 统一管理 VR 手柄的轴 (Axis) 和按钮 (Button) 索引，并自动生成对应的 `.hpp` 和 `.py` 文件。
- **性能监控工具**: 提供频率 (`Frequency`) 和时间间隔 (`Interval`) 的测量工具，用于监控实时控制流。
- **连接质量评估 (Python)**: 实现基于滑动窗口的丢包和延迟检测逻辑，用于评估遥操作的网络环境。
- **通用数学工具**: 包含常用的坐标转换辅助函数。
- **身份广播节点**: 周期发布机器人 `SN` 与 `IP` 到全局话题 `/sn`，用于多机器人发现。

---

## 3. 文件夹结构 (Project Structure)
```text
bw_utils/
├── include/bw_utils/         # C++ 头文件目录
│   └── common_utils.hpp      # C++ 频率测量工具类
├── bw_utils/                 # Python 模块目录
│   ├── common_utils.py       # Python 频率与网络质量测量类
│   ├── controller_indices.py # (生成的) 控制器索引常量
│   ├── robot_profile.py      # 机器人型号 profile 读取与版本归一化
│   ├── sn_utils.py           # SN 生成工具
│   └── sn_publisher_node.py  # SN/IP 广播节点
├── config/                   # 配置与模板
│   ├── controller_indices.yaml # 原始配置文件
│   ├── controller_indices.hpp.in # C++ 代码生成模板
│   └── controller_indices.py.in  # Python 代码生成模板
├── CMakeLists.txt            # 构建脚本 (包含代码生成逻辑)
└── package.xml               # 包描述文件
```

---

## 4. 技术栈 (Technical Stack)
- **编程语言**: C++ 17, Python 3
- **构建系统**: `ament_cmake`, `ament_cmake_python`
- **配置解析**: `yaml-cpp`
- **依赖库**: 
    - `Eigen 3`: 矩阵运算支持。
    - `YAML-CPP`: 用于 CMake 构建时的配置解析。

---

## 5. 核心工具类说明 (Core Utilities)

### 5.1 频率测量 (C++ / Python)
- **C++**: `bw_utils::Utils` 类提供 `measure_frequency_interval()` 方法，用于实时计算回调频率。
- **Python**: `FrequencyMeasurer` 类不仅支持频率测量，还集成了网络质量评估算法。

### 5.2 自动生成代码 (Code Generation)
构建时，CMake 会解析 `config/controller_indices.yaml`，并将以下变量注入模板：
- `JOY_X_AXIS`, `JOY_Y_AXIS`: 摇杆轴索引。
- `BUTTON_TRIGGER`, `BUTTON_GRIP`: 手柄按键索引。

生成的代码位于：
- C++: `install/bw_utils/include/bw_utils/controller_indices.hpp`
- Python: `install/bw_utils/lib/python3.x/site-packages/bw_utils/controller_indices.py`

### 5.3 网络工具 (Network Utility)
- `get_local_ip()`: 获取机器人当前局域网 IP（失败回退 `127.0.0.1`）。

### 5.4 机器人型号 Profile 工具
- `robot_profile.py` 统一读取 `bw_bringup/config/robot_profiles.yaml`。
- `normalize_robot_version()` 将 `1/2/3` 归一化为 `1.0/2.0/3.0`，非法值回退到 `2.0`。
- `load_robot_profile()` 返回 `RobotProfile`，包含 `mantis_model`、IK URDF 相对路径、滑台默认高度/限位和是否支持上半身弯腰。
- `robot_profile_to_pelvis_params()` 将 profile 转为串口/仿真后端复用的 launch 参数。

该模块是 `robot_version` 配置的唯一解析入口。`bw_bringup`、`bw_sim2real` 和 `bw_core` 的 RViz 启动文件都应调用本模块，避免重复维护型号别名、URDF 名称和滑台参数。

---

## 6. 接口定义 (Interface Definitions)
*注：本包既包含工具库，也包含 `sn_publisher_node` 节点。*

### 6.1 网络质量评估 (Python 接口)
```python
def evaluate_connection_quality(self, window_seconds, threshold_ms, max_violations) -> bool:
    """
    True 表示连接质量差（丢包或延迟超标）
    False 表示连接质量良好
    """
```

### 6.2 本机 IP 获取 (Python 接口)
```python
def get_local_ip() -> str:
    """
    返回本机局域网 IP，失败时返回 127.0.0.1
    """
```

### 6.3 机器人型号 Profile (Python 接口)
```python
def load_robot_profile(robot_version: str | None) -> RobotProfile:
    """
    读取机器人型号配置，配置源为 bw_bringup/config/robot_profiles.yaml。
    """
```

### 6.4 机器人身份广播节点 (`sn_publisher_node`)
- **发布 Topic**: `/sn`
- **消息类型**: `bw_interface/msg/RobotIdentity`
- **字段语义**:
  - `ip`: 机器人当前局域网 IP
  - `sn`: 机器人 SN（如 `BW_4TEOGD`）
- **参数**:
  - `topic_name` (`string`, 默认 `/sn`)
  - `publish_frequency` (`double`, 默认 `1.0`)
  - `ip_override` (`string`, 默认空，非空时覆盖自动探测 IP)

---

## 7. 行为语义 (Behavioral Semantics)

### 7.1 配置一致性保障
由于所有控制器索引都源自同一个 YAML 文件，修改 `controller_indices.yaml` 后重新编译，整个遥操作系统的 C++ 节点（如 `bw_teleop`）和 Python 节点（如 `bw_core`）将同步更新，彻底杜绝了手动修改多处代码导致的索引不一致问题。

---

## 8. 编译与运行 (Build & Run)

```bash
# 编译并生成代码
colcon build --packages-select bw_utils

# 检查生成的 Python 模块
python3 -c "import bw_utils; print(bw_utils.controller_indices.JOY_X_AXIS)"

# 单独运行 SN 广播节点（话题固定为全局 /sn）
ros2 run bw_utils sn_publisher_node
```

---

## 9. 工程约束 (Compliance & Enforcement)
- **无状态设计**: 工具类应保持无状态或自包含，避免依赖全局变量。
- **模板限制**: 修改 `.in` 模板文件时需确保符合各语言的语法规范，避免构建失败。
- **性能敏感**: 频率测量函数应保持极低的 CPU 开销，因为它们通常被置于高频控制循环中。
