# Standard 机器人 URDF 模型

`bw_std_description` 是一个 ROS 2 URDF 描述包，包根目录即本仓库根目录，用于在 RViz2 中加载和查看 Standard 机器人的不同形态。项目当前提供 std_auto 运行时模型和 std_v1-std_v6 六种显示模型，并通过 `joint_state_publisher_gui` 的滑动条控制模型中可运动的关节。

## 目录结构

```text
bw_std_description/               # 仓库根目录即 ROS 2 包根目录
├── README.md                     # 项目说明文档
├── CMakeLists.txt                # CMake 构建和资源安装配置
├── package.xml                   # ROS 2 包信息及运行依赖
├── .gitignore                    # Git 忽略规则
├── config/
│   └── joint_names_std.yaml      # 关节名称配置
├── rviz/
│   └── std.rviz                  # RViz2 显示配置
├── launch/
│   ├── display.launch.py         # 启动 URDF、关节控制 GUI 和 RViz2
│   └── gazebo.launch.py          # Gazebo 启动文件
├── urdf/
│   ├── std_auto.urdf             # 运行时模型，也是显示模型的生成源
│   ├── std_v1.urdf               # v1 整体形态
│   ├── std_v2.urdf               # v2 双臂形态
│   ├── std_v3.urdf               # v3 巡检形态
│   ├── std_v4.urdf               # v4 工程形态
│   ├── std_v5.urdf               # v5 单臂形态
│   ├── std_v6.urdf               # v6 底盘形态
│   └── std_source.csv            # SolidWorks 导出源表，生成脚本不读取
├── meshes/
│   ├── v1/ ... v6/               # 各形态独立使用的 STL 模型文件
│   └── ...                       # URDF 生成过程中使用的基础 mesh
└── scripts/
    └── generate_std_urdf_variants.py # 生成和更新各形态 URDF/mesh
```

其中，日常使用主要涉及 `urdf/`、`meshes/`、`launch/`、`rviz/` 和 `config/`。当前目录为未编译的源码状态，不包含 `colcon` 编译产物。执行 `colcon build` 后，包根目录会自动生成：

```text
build/                          # colcon 编译中间文件
install/                        # 编译后的安装空间
log/                            # colcon 编译日志
```

## 模型形态

| `robot_model` | 形态 | 说明 |
| --- | --- | --- |
| `std_auto` | 运行时 | 完整运动学契约，供上层控制与硬件形态探测使用 |
| `std_v1` | 整体形态 | 完整机器人模型 |
| `std_v2` | 双臂形态 | 双机械臂形态 |
| `std_v3` | 巡检形态 | 巡检机器人形态 |
| `std_v4` | 工程形态 | 工程机器人形态 |
| `std_v5` | 单臂形态 | 单机械臂形态 |
| `std_v6` | 底盘形态 | 单独的底盘模型 |

## 编译程序

进入工作区并加载 ROS 2 环境：

```bash
cd bw_std_description
source /opt/ros/humble/setup.zsh
```

编译 `bw_std_description` 功能包：

```bash
colcon build --packages-select bw_std_description --symlink-install
```

编译完成后加载当前工作区环境：

```bash
source ./install/setup.zsh
```

每次打开新的终端，都需要重新执行以下命令：

```bash
source /opt/ros/humble/setup.zsh
source ./install/setup.zsh
```

## 清理编译文件

如需恢复到仅保留源码的状态，在工作区根目录执行：

```bash
rm -rf build install log
```

清理后 `build/`、`install/` 和 `log/` 不可直接恢复，但它们都是可再生成的编译产物。重新执行“编译程序”中的命令即可生成。

## 运行程序

不指定形态时，默认加载 `std_v1` 整体形态：

```bash
ros2 launch bw_std_description display.launch.py
```

指定模型形态时使用 `robot_model` 参数：

```bash
ros2 launch bw_std_description display.launch.py robot_model:=std_v1
ros2 launch bw_std_description display.launch.py robot_model:=std_v2
ros2 launch bw_std_description display.launch.py robot_model:=std_v3
ros2 launch bw_std_description display.launch.py robot_model:=std_v4
ros2 launch bw_std_description display.launch.py robot_model:=std_v5
ros2 launch bw_std_description display.launch.py robot_model:=std_v6
ros2 launch bw_std_description display.launch.py robot_model:=std_auto
```

启动后会打开 RViz2 和关节控制窗口。可以通过关节控制窗口中的滑动条调整当前形态所包含的可运动关节。

## 在 Gazebo 中运行

Gazebo 使用与 RViz2 相同的 `robot_model` 参数。不指定形态时默认加载 `std_v1`：

```bash
ros2 launch bw_std_description gazebo.launch.py
```

指定模型形态时同样使用 `robot_model` 参数，`std_auto` 与 std_v1-std_v6 均可加载：

```bash
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v1
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v2
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v3
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v4
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v5
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v6
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_auto
```

仅启动 Gazebo 服务端、不打开 Gazebo 图形窗口：

```bash
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v1 gazebo_gui:=false
```

可以通过 `x`、`y`、`z` 和 `yaw` 设置模型的初始位置和方向。例如：

```bash
ros2 launch bw_std_description gazebo.launch.py robot_model:=std_v3 x:=1.0 y:=0.5 z:=0.05 yaw:=1.57
```

Gazebo 默认让模型在地面上方 `0.05 m` 处生成，以避免模型初始状态与地面穿插。

启动文件默认禁用 Gazebo Classic 在线模型库请求，并把 mesh 的 `package://bw_std_description/` 前缀替换为本地绝对路径，避免网络不可用或路径无法解析时界面长时间停留在“无响应”状态。

## 可选启动参数

不启动关节控制 GUI：

```bash
ros2 launch bw_std_description display.launch.py robot_model:=std_v1 gui:=false
```

不启动 RViz2：

```bash
ros2 launch bw_std_description display.launch.py robot_model:=std_v1 use_rviz:=false
```

同时关闭 GUI 和 RViz2，仅启动模型发布节点：

```bash
ros2 launch bw_std_description display.launch.py robot_model:=std_v1 gui:=false use_rviz:=false
```

使用任意绝对路径的 URDF 覆盖 `robot_model` 解析结果，`display.launch.py` 与 `gazebo.launch.py` 都支持：

```bash
ros2 launch bw_std_description display.launch.py model:=/absolute/path/model.urdf
ros2 launch bw_std_description gazebo.launch.py model:=/absolute/path/model.urdf
```

指定其他 RViz2 配置：

```bash
ros2 launch bw_std_description display.launch.py rviz_config:=/absolute/path/config.rviz
```

指定 Gazebo 中的模型实体名（默认 `std_robot`）：

```bash
ros2 launch bw_std_description gazebo.launch.py entity_name:=std_robot_2
```

## 修改模型后重新生成

如修改了模型拆分或 mesh 生成逻辑，可执行：

```bash
python3 scripts/generate_std_urdf_variants.py
colcon build --packages-select bw_std_description --symlink-install
source install/setup.bash
```

生成脚本以 `urdf/std_auto.urdf` 为源模型，需要 NumPy 环境。生成后确认所有模型仍使用 `package://bw_std_description/` 资源 URI。

修改 mesh 后建议先完全关闭已运行的 RViz2，再重新启动，避免继续显示缓存中的旧模型。

## 开源协议

本项目使用 MIT License，条款见仓库根目录 `LICENSE`。
