# 多边形禁行区（规划避让）设计

日期：2026-11-13。状态：方案待确认，尚未开始功能实现。

## 1. 需求与语义

用户已确定的要求：

1. 用户通过 GUI 加载 PGM 地图，在地图上绘制任意数量的多边形区域。
2. 多边形内部的全局代价地图代价设置为 Nav2 内部值 **252**。
3. 这些区域只影响路径规划，不加入局部代价地图，不对局部控制、速度输出或脱困动作增加禁行限制。
4. 机器人在区域内启动或误入后，给定区外目标且存在真实可行通路时，仍可从区域内规划出去。
5. RViz 中使用红色填充显示这些区域，且能看到多边形内部，不只是轮廓。
6. 区域可独立保存并在导航重启后恢复。
7. GUI 编辑器、区域配置处理和掩膜发布放在同一个独立新包中。
8. RViz 显示直接修改导航启动使用的现有 RViz 文件。
9. 除 GUI 外提供程序化区域编辑接口，至少支持新建与删除；接口和 GUI 共用区域管理逻辑。

252 表达“尽量绕开”的高代价，可通行，不是致命障碍。由完整路径的累计代价决定走法：允许为了缩短总路线而穿过区域，不保证最短出区，不增加专门的两阶段出区规划，也不自动生成无任务时的出区目标。区域内目标同样按可通行高代价处理，不额外拒绝。

不得将原有墙体、障碍膨胀区或未知格子降为 252。真实可行性的前提仍由既有导航系统判断。

## 2. 设计选择与范围

### 2.1 方案比较

| 方案 | 取舍 |
| --- | --- |
| 直接修改原始 PGM | 撤销和保存多边形编辑语义困难；还会通过静态层影响局部地图，不采用。 |
| 原生 KeepoutFilter + 普通掩膜 | 可复用过滤器，但 Humble 将整数 0…100 线性换算为内部 0…254，99 对应 251、100 对应 254，无法精确写入 252，不采用。 |
| 独立掩膜 + 专用全局代价层 | 保留多边形与基础地图的分离，掩膜只表达覆盖关系，由插件明确写入 252；采用。 |

实现模块包括：地图编辑窗口、配置及几何公共模块、区域发布节点、全局代价层插件、导航 launch/config 接入和 RViz 显示配置。GUI 延续项目现有 tkinter 技术，不引入新的 GUI 框架、RViz 插件或规划器分支。区域消息和服务类型复用现有 `bw_interface` 包定义，不额外新建接口专用包。

### 2.2 包边界

新建独立 ROS 2 Python 包 `bw_planning_zones`，集中提供 GUI、配置/几何处理、掩膜与 MarkerArray 发布，以及自己的测试和发布节点 launch。编辑器是独立程序，不向现有 `bw_nav_debug_gui.py` 加入窗口、入口或业务逻辑。

| 所属位置 | 职责 |
| --- | --- |
| 新包 `bw_planning_zones` | 独立 GUI 编辑器、配置读写、坐标转换、栅格化、三角化、mask/MarkerArray/区域状态发布，以及新建、删除、更新、查询和批量提交服务。 |
| 现有包 `bw_fastlio_nav2` | 全局 `PlanningZoneLayer` 插件、导航参数及启动接入。 |
| 现有包 `bw_interface` | `PlanningZone`、区域状态消息及区域管理服务类型；只定义接口，不实现业务逻辑。 |
| 现有 `bw_fastlio_nav2/rviz/mid360_nav2.rviz` | 直接增加红色填充的 MarkerArray 显示项。 |

新包采用 `ament_python`，不依赖导航包，也不导入原调试 GUI；导航包通过运行时依赖启动新包的发布节点，依赖方向保持为 `bw_fastlio_nav2 → bw_planning_zones → bw_interface`。新包不启动另一个 RViz，也不维护一份替代导航 RViz 配置。

下列是本设计建议的默认行为，属于实现选择：

- GUI 支持离线编辑；导航运行时提供显式“提交修改”，通过服务一次提交完整区域快照，拖动顶点期间只预览。
- 不限制固定区域数量或固定顶点数量；资源开销随地图大小、区域数量和顶点数量增加。
- 支持凸、凹简单多边形；不支持自交轮廓及单区域带孔，校验失败时提示用户修改。
- 提供新增、选择、改名、顶点拖动、撤销顶点/取消绘制、删除、启用/禁用、缩放和平移。
- 不增加区域膨胀或车体半径扩张，填充用户画出的范围；范围遵循全局 costmap 栅格精度。
- 不自动修改现有 planner 的代价倍率、平滑参数和实时重规划频率。

## 3. 数据流

```text
独立 GUI（在线提交） / 外部程序（区域管理服务）
        ↓
bw_planning_zones / planning_zone_server
        ├── 统一校验与版本检查 → 原子保存 *.planning_zones.yaml
        ├── /planning_zones/mask：OccupancyGrid，区域外 0、区域内 100
        │           ↓
        │    global_costmap / PlanningZoneLayer：将覆盖处写成 252
        │           ↓
        │    SmacPlanner2D 与既有规划平滑流程
        │
        ├── /planning_zones/markers：MarkerArray → RViz 红色填充
        └── /planning_zones/state：完整区域状态 → GUI / 外部程序
```

离线 GUI 使用同一套配置与几何库保存文件；在线由服务端作为唯一正常写入端。这里的 mask 值 100 只表示“属于区域”，不经过标准 OccupancyGrid 到 Costmap2D 的占用概率转换。局部地图和底盘速度链路不订阅该掩膜。

## 4. 地图、配置与几何规则

### 4.1 关联正确的地图

编辑器打开导航使用的 map YAML，按其 image 字段加载 PGM。必须关联 launch 的 `map_yaml`，不使用定位用的 BBS 栅格作为底图。

配置默认存储为导航地图 YAML 同目录下的 `<地图名>.planning_zones.yaml`。区域顶点使用 `map` 坐标系米制坐标，不能保存屏幕像素坐标。配置记录格式版本、地图几何元数据与图像 SHA-256 标识，以发现地图被替换的情况。地图标识冲突时拒绝应用，并提示重新确认；文件移动但内容相同不应误判。

配置示意：

```yaml
version: 1
revision: 0
frame_id: map
cost: 252
map:
  yaml: mid360_pgo_csf.yaml
  image_sha256: '<地图内容标识>'
  width: 1000
  height: 800
  resolution: 0.05
  origin: [-10.0, -8.0, 0.0]
zones:
  - id: zone_001
    name: 禁行区 1
    enabled: true
    vertices:
      - [1.0, 2.0]
      - [4.0, 2.0]
      - [3.5, 4.0]
      - [2.0, 3.0]
```

`cost` 在本版固定为 252，读取时验证；不支持每个区域独立设定其他代价。

### 4.2 坐标与几何

GUI 坐标转换先消除缩放和平移，再处理图像纵轴翻转与地图 resolution/origin。显示顶点、栅格化和 RViz 三角化使用同一份规范化后的世界坐标多边形。

导航地图在本版按当前项目的轴对齐地图处理。若 origin yaw 非零，明确拒绝应用并报告不支持，避免与 Nav2 的轴对齐 Costmap2D 产生误配。

几何校验至少包括：有限坐标、至少三个不同顶点、非零面积、无自交和地图边界检查。重复闭合点和连续重复点可规范化删除。新绘制或拖动顶点限制在地图范围内，外部配置越界则拒绝应用，不静默裁剪。

栅格化支持凹多边形，不能用外接矩形、凸包或只支持凸多边形的填充 API 替代。采用保守覆盖：与多边形内部有面积交叠的格子纳入掩膜；仅相切的边界按统一规则处理并测试。多个启用区域取并集，重叠部分不叠加至 253/254。细长区域在栅格分辨率下也应保持覆盖。

## 5. GUI 与在线应用

GUI 完全放在新包 `bw_planning_zones` 中，提供独立可执行入口 `planning_zone_editor`。通过启动参数或“打开地图”选择导航地图 YAML，可单独用于离线配置，不依赖现有调试台启动。窗口、配置和几何处理拆分为该包内部模块；本功能不修改 `bw_nav_debug_gui.py`。

交互建议：

```text
[打开地图] [新建区域] [撤销顶点] [删除] [保存（离线）] [提交修改（在线）]
┌──────────────────────────┬─────────────────┐
│ PGM 地图                  │ 区域列表         │
│ 多边形红色填充与顶点       │ 启用状态、名称   │
│ 缩放、平移、选择和拖动     │ 编辑与保存状态   │
└──────────────────────────┴─────────────────┘
```

左键添加顶点，点击起点或 Enter 闭合，Esc 取消。未保存草稿只在编辑器中预览，不能提前影响在线规划或 RViz。切换地图或关闭有未保存修改的窗口时提示处理修改。

离线保存采用同目录临时文件加原子替换，明确显示未在线应用。在线编辑保持本地草稿，点击“提交修改”后调用 `/planning_zones/replace_all`，携带 map_id 和编辑起点的 expected_revision；由服务端统一保存并发布，GUI 不直接改写当前在线配置文件。

GUI 与程序化接口都使用服务端的同一套校验、栅格化、三角化、持久化及发布流程。GUI 区分“已保存”“已提交/已发布”和失败状态，不把服务提交成功宣称为全局 costmap 已完成刷新。在线区域在下一次全局 costmap 更新及正常重规划后影响路径，不强制停车或取消任务。

GUI 订阅完整区域状态。没有未提交草稿时自动刷新外部程序的修改；有草稿时保留草稿并提示版本变化，提交冲突时要求重新读取和确认，不静默覆盖外部修改。

服务只操作导航启动时绑定的地图和配置路径；GUI 编辑其他地图时不得把它误应用到当前导航实例。

## 6. 区域发布节点

新包中的可执行节点 `planning_zone_server` 启动时接收导航 map YAML 和可选区域配置路径，解析基础地图及区域。新包提供 `planning_zones.launch.py`，只启动该发布节点；导航 launch 负责包含此 launch 并透传地图路径、区域配置路径和 use_sim_time。发布完整的二值 `nav_msgs/OccupancyGrid` 掩膜，尺寸、分辨率、原点与导航地图一致，frame_id 为 map，区域外 0、区域内 100，不使用 -1。

mask、MarkerArray 与完整区域状态均使用 reliable、transient_local、KeepLast(1)，使后启动的代价层和 RViz 获取最新完整快照。发布器生命周期覆盖导航运行过程。

未指定配置时按地图默认路径查找：默认路径不存在表示没有区域，发布全零掩膜及清空显示消息。显式指定的配置不存在、已有配置损坏或地图不匹配时报告错误；初次加载失败不得宣称加载成功。运行中重新加载失败则保留上一份完整有效快照，不发布半成品。

本版不要求外部进程修改文件后自动监听更新；外部程序通过区域管理服务修改，手工离线文件需要显式 reload。区域删除、清空、启停和地图更新均通过完整快照发布，不依赖增量消息顺序。


### 6.1 程序化编辑接口

接口作为本功能的一部分提供可调用的 ROS 2 services，而不只是 GUI 内部函数。请求与响应类型定义在现有 `bw_interface`，由 `bw_planning_zones/planning_zone_server` 实现；Python 配置/几何库不依赖 GUI 窗口，在线节点可在无显示环境下运行。

区域数据 `PlanningZone.msg` 的字段：

```text
string id
string name
bool enabled
geometry_msgs/Point[] vertices
```

vertices 为 map 坐标系的米制顶点，z 必须为 0；代价固定 252，不接受接口随意修改区域代价。id 是持久化稳定标识，删除、更新和启停按 id 操作，不使用列表下标或可能重名的名称。新建时可提供唯一 id，留空则由服务端生成 UUID 并返回。重复的非空 id 返回 ALREADY_EXISTS，不隐式改成更新。

| 服务路径 | 类型（bw_interface） | 输入与行为 |
| --- | --- | --- |
| `/planning_zones/create` | `CreatePlanningZone` | 一份区域定义；校验后新建并返回实际区域 id。 |
| `/planning_zones/delete` | `DeletePlanningZone` | 区域 id；删除对应区域，不存在返回 NOT_FOUND。 |
| `/planning_zones/update` | `UpdatePlanningZone` | 带已有 id 的完整区域定义；替换名称、启用状态和顶点，不支持含糊的隐式字段补丁。 |
| `/planning_zones/list` | `ListPlanningZones` | 查询当前地图的全部区域、map_id 和 revision；不修改文件。 |
| `/planning_zones/set_enabled` | `SetPlanningZoneEnabled` | 区域 id 和 enabled；保留顶点，启用或停用覆盖。 |
| `/planning_zones/replace_all` | `ReplacePlanningZones` | 完整区域列表；原子替换全部区域供 GUI 批量提交，空列表表示清空。 |
| `/planning_zones/reload` | `ReloadPlanningZones` | 显式重新读取启动时绑定的配置路径；不接受客户端指定任意文件路径。 |

其中 create/delete 是用户明确要求，update/list/set_enabled/replace_all 是为完整编辑和 GUI 复用预留的配套接口。所有修改类服务携带 `map_id` 和 `expected_revision`；list 可用空 map_id 查询当前绑定地图，以取得有效标识和初始版本。map_id 由地图内容和几何元数据生成，不依赖文件绝对路径。

所有服务统一返回 `success`、结构化 `error_code`、`message`、当前 `map_id` 和 `revision`；新建返回实际 id，查询返回完整区域列表。修改类响应另带 `published`，区分已提交配置和已经发布。错误码至少包括 INVALID_ARGUMENT、NOT_FOUND、ALREADY_EXISTS、MAP_MISMATCH、REVISION_CONFLICT、IO_ERROR、NOT_READY。

### 6.2 一致性与发布顺序

统一的区域管理核心提供 create/delete/update/list/set_enabled/replace_all/reload 操作；GUI、ROS service 和离线文件工具复用其配置与几何校验，不各自实现不同规则。

在线修改流程：

1. 将修改请求串行化，检查当前地图、expected_revision 和 id 规则。
2. 在当前快照副本上应用操作，校验完整结果，构建新 mask、Marker 和区域状态。
3. 以当前 revision + 1 生成持久化配置，写临时文件并原子替换。此前校验、构建或写盘失败，保留原有效状态。
4. 文件提交后更新内存快照，发布完整 mask、MarkerArray 和区域状态。成功提交返回新版本；不等待全局 costmap 刷新。
5. 若文件已提交而发布失败，不能声称已回滚：返回 success=true、published=false 和明确说明，并重试发布该版本。客户端可通过 list 和状态检查结果，不盲目重试新建。

revision 随成功内容修改单调增加并保存在配置中，重启后继续使用；无实际变化的请求不必增版。expected_revision 不匹配时拒绝提交，客户端重新读取后再决定修改。批量替换须全部校验成功后提交，不能留下只更新一部分区域的结果。

新增 `/planning_zones/state` 完整状态话题，包含 map_id、revision 和所有区域（含禁用区域），类型也定义在 bw_interface。每次成功提交发布自包含快照，供 GUI 和其他客户端同步；mask/Marker 只包含启用区域。

在线正常编辑只由服务端写绑定配置文件。离线编辑器保存时同样检查其读取版本；手工绕过接口改文件属于显式 reload 的运维路径，不与在线服务修改无锁混用。服务类型和行为需要给出 Python 调用示例；新建示例返回的 id 可直接用于删除示例。

## 7. 全局代价层

新增 `bw_fastlio_nav2::PlanningZoneLayer`，仅加载在 global_costmap，位于普通 inflation_layer 之后；若当前恢复代价层存在，则放在恢复代价层之后。既有层先生成环境与恢复代价，本层最后叠加区域的非致命高代价。

在覆盖格子内按下面的规则合成：

```text
原代价 0…252 → 252
原代价 253   → 保持 253
原代价 254   → 保持 254
原代价 255   → 保持 255
区域外      → 保持原结果
```

仅解释掩膜的覆盖标记，不调用标准 OccupancyGrid 占用概率转换。本层不把虚拟区域作为致命障碍源，也不增加额外膨胀，确保用户画出的范围内保持非致命代价。

接收掩膜时严格验证 frame、尺寸、resolution、origin、数据长度及二值取值，拒绝错位或不完整消息。最新快照替换在锁内完成；一轮 updateBounds/updateCosts 使用一致的快照，不能半轮切换数据。

为保证删除、移动、缩小、禁用后无残留，刷新范围需要覆盖旧掩膜和新掩膜的并集。本版可在每次完整掩膜替换时请求一次全图重建，优先保证正确性；平时按更新窗口叠加，不能持续累加到上一帧最终 costmap 上。

`reset`、ClearEntireCostmap、生命周期重新激活及地图尺寸变化时保留/重新获取有效区域定义，并在下一次更新重绘。初次有效掩膜到达前不报告区域层已就绪；空掩膜是合法的已就绪状态。区域从全局 costmap 清除时仍要保留真实障碍和其他层的贡献。

插件正常启停与区域 enabled 状态不能形成两套互相矛盾的公开控制入口。本版用户级区域启停由同一配置快照统一控制。

## 8. RViz 红色填充

为每个启用区域发布：

- `TRIANGLE_LIST`：对简单凹多边形进行正确三角化，填充色 RGBA 建议为 (1.0, 0.0, 0.0, 0.35)。
- `LINE_STRIP`：闭合的红色边界，建议比填充更不透明，线宽约 0.02 m。

坐标 frame_id 为 map，填充位于地图上方约 0.02 m，边线再略微抬高，避免与地图重叠闪烁。Marker pose 使用单位四元数，scale 为 (1,1,1)，lifetime 为 0。

凹多边形采用耳切等支持凹形的三角化，不用可能填满凹口的简单三角扇。三角化和 mask 生成基于同一份多边形定义。RViz 显示向量边界，costmap 是栅格近似，边缘存在一个栅格量级的量化差异。

专用 `/planning_zones/markers` 话题每次发布自包含 MarkerArray：先清空该显示的旧 marker，再发布全部当前启用区域的填充与边界。这样区域删除、禁用、全部清空以及 RViz 后启动都不会留下旧红块。禁用区域不在 RViz 中显示红色，GUI 列表可继续显示其配置。

直接修改现有导航启动文件实际加载的 `src/bw_fastlio_nav2/rviz/mid360_nav2.rviz`，默认启用名为“禁行区（规划避让）”的 MarkerArray 显示，使用 transient_local 订阅。`mid360_nav2_localization.launch.py` 继续加载这同一个 RViz 文件。红色表示有效配置中的启用区域，不改变地图本身配色；新包不复制此 RViz 文件。

## 9. 与当前项目的集成

- 保留原始 PGM、导航 map YAML、定位用 BBS 栅格及局部 costmap 结构。
- 新增 `src/bw_planning_zones`，使用 `package.xml`、`setup.py`、`setup.cfg` 和 resource 标识安装 Python 模块、编辑器/发布节点入口及 launch。
- 导航 launch 增加区域配置路径参数，并从 `map_yaml` 推导默认配置路径，包含新包的 `planning_zones.launch.py` 来启动发布节点。
- global_costmap 增加本插件，local_costmap 不增加。
- 已有恢复代价层继续保留其点事件、TTL、清除及最大值合成语义；两层重叠时 252 保持为 252，真实 253…255 仍保留。
- 现有 SmacPlanner2D 和平滑流程继续读取全局 costmap；现有局部控制、behavior_server 与速度转发/脱困不订阅区域掩膜，不增加虚拟区域速度拦截。
- `bw_fastlio_nav2` 的 CMake 只增加 C++ 代价层库与插件导出，package.xml 增加对 `bw_planning_zones` 的运行时依赖；该导航包不安装本功能的 GUI 或 Python 发布节点。
- `bw_planning_zones` 自己声明 rclpy、nav_msgs、visualization_msgs、bw_interface、YAML 和 GUI 等直接依赖，负责安装 GUI、配置/几何模块、发布节点及其 launch；不反向依赖 `bw_fastlio_nav2`。
- 在现有 `bw_interface` 增加区域消息和类型化服务，并纳入其现有 rosidl 生成流程；不新建第二个功能包或额外接口包。
- RViz 的 MarkerArray 显示直接编辑现有 `bw_fastlio_nav2/rviz/mid360_nav2.rviz`，保留导航默认加载关系。
- README 补充绘制、保存、应用、RViz 显示和高代价可穿越语义。

## 10. 验收与验证

1. GUI 可连续创建、编辑、保存、重载多个凸/凹多边形；缩放平移后坐标不漂移。
2. 拒绝自交、退化、非有限坐标、越界及地图不匹配配置；凹口不会被错误填满。
3. 覆盖处原 0…252 精确变成 252；原 253、254、255 保持；区域外保持原层结果；多个区域重叠不累加。
4. 机器人位于 252 区域内时，给定区外目标并具有真实可行通路，可成功规划出去；不把最短出区作为验收要求。
5. 存在明显代价优势的绕行路径时规划器优先绕行；另外构造只能穿区/穿区总代价更低的案例验证其仍可通行语义，不断言所有地图都绝不穿区。
6. 删除、缩小、移动、禁用和清空后，下一次更新清除本层旧贡献；真实地图与恢复代价仍正确。
7. clear costmaps、生命周期切换及节点重启后，区域配置持续生效或自动重载。
8. 全局 costmap 受到影响，局部 costmap 不含区域贡献；不因区域增加速度拦截。
9. RViz 对凸/凹多边形呈现红色半透明内部填充和闭合边界；删除无残留、后启动可见；原 PGM 内容未改变。
10. 在线配置失败保留旧有效状态，GUI 不把只保存/只发布误报为已写入 costmap。
11. 新包可单独启动 GUI 和发布节点，不依赖原调试 GUI；导航启动使用修改后的原 RViz 文件，无需另开 RViz 或手动添加显示。
12. 不启动 GUI，仅调用接口即可新建、删除、更新、查询和启停区域；成功修改持久化并更新 mask、Marker 和状态，重启后保留。
13. 验证重复 ID、删除不存在的 ID、无效几何、地图不匹配、写盘失败和版本冲突；失败不产生半份区域状态。
14. GUI 和外部客户端并发编辑时，旧 expected_revision 被拒绝，不静默覆盖；GUI 收到外部修改时同步或明确提示草稿冲突。

验证分为几何与配置单元测试、插件合成与刷新测试、隔离 ROS 规划集成测试、GUI/RViz 可视化检查。隔离集成测试不启动串口或底盘运动节点。

## 11. 留给实现者的常规选择

几何函数拆分、窗口具体布局、三角化实现细节、缓存布局和测试夹具可由实现者决定，只要遵守上述语义和验收条件。没有固定性能数值承诺，需报告测试地图的尺寸、顶点数量和重载耗时。

上游核对基于 ROS 2 Humble：Costmap2D 的 OccupancyGrid 转换、KeepoutFilter 的掩膜转换、Smac Node2D 的代价累计与碰撞阈值，以及 visualization_msgs/Marker 的 TRIANGLE_LIST/LINE_STRIP/DELETEALL 定义。
