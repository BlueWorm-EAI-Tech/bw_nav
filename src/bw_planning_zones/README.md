# 多边形禁行区

区域只对全局规划加代价。全局代价地图中，被覆盖的可通行格子变为 252，原有 253、254、255 不变。代价 252 仍可通行，因此搜索倾向绕行，但必要时可以穿过，现有路径平滑也可能切入区域。已在区内且存在可行通路时可以规划离开，不保证走最近出口。局部代价地图、控制和脱困不使用禁行掩膜。

## 编辑地图

在已加载工作区环境的终端运行：

```bash
ros2 run bw_planning_zones planning_zone_editor --map /path/to/map.yaml
```

用导航启动时的 `map_yaml`，不是用于重定位的 BBS 地图。可省略 `--map`，从窗口选择地图。滚轮缩放，右键或中键拖动地图。点击「＋ 区域」后逐点画边界，点击首点或按 Enter 闭合；退格撤销最后一个未闭合顶点，Esc 取消。选中区域可拖动顶点、重命名、禁用或删除；凹多边形有效，自交轮廓无效。

导航未运行时使用「离线保存」，配置默认写入与地图 YAML 同目录的 `<地图名>.planning_zones.yaml`。当前地图的区域服务可用时，使用「提交修改」；提交后红色填充在导航自带 RViz 中显示，新代价在后续全局地图更新和重规划时使用。原始 PGM 不会被改写。

如果线上区域已被其他客户端修改，编辑器会保留草稿并停用提交；可通过「读取当前」确认放弃草稿并加载线上配置。如果提交未收到响应，先用「读取当前」核对结果再重试。

导航 launch 可通过 `planning_zones_file:=/path/to/existing.planning_zones.yaml` 绑定已有配置。编辑器离线打开该配置使用 `--map /path/to/map.yaml --zones-file /path/to/existing.planning_zones.yaml`。

只运行区域服务、不启动导航或底盘时：

```bash
ros2 launch bw_planning_zones planning_zones.launch.py map_yaml:=/path/to/map.yaml
```

## 程序化编辑

默认服务命名空间是 `/planning_zones`。服务类型由 `bw_interface` 提供：

| 服务 | 作用 |
| --- | --- |
| `list` | 获取地图标识、修订号及所有区域 |
| `create` | 新建区域；未指定 ID 时由服务端生成 |
| `delete` | 按 ID 删除区域 |
| `update` | 按 ID 替换完整定义 |
| `set_enabled` | 按 ID 启用或禁用 |
| `replace_all` | 原子替换全部区域；空列表为清空 |
| `reload` | 重新读取当前服务端绑定的配置，不接受其他路径 |

请求使用 `map` 坐标系的米制顶点，`z=0`。修改请求携带 `map_id` 和 `expected_revision`；遇到 `REVISION_CONFLICT` 时重新查询并确认编辑内容。`success=true` 表示已提交，`published=true` 表示发布调用已成功，不代表规划器已经使用该版本。

以下脚本演示创建后按 ID 删除；测试前先修改坐标，使其位于当前地图内：

```python
import rclpy
from bw_interface.msg import PlanningZone
from bw_interface.srv import CreatePlanningZone, DeletePlanningZone, ListPlanningZones
from geometry_msgs.msg import Point

rclpy.init()
node = rclpy.create_node('zone_example')

def call(name, service, request):
    client = node.create_client(service, '/planning_zones/' + name)
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError(f'Service unavailable: {name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    result = future.result()
    if result is None or not result.success:
        raise RuntimeError(result.message if result else 'Request timed out; query state before retry')
    return result

try:
    state = call('list', ListPlanningZones, ListPlanningZones.Request())
    result = call('create', CreatePlanningZone, CreatePlanningZone.Request(
        map_id=state.map_id, expected_revision=state.revision,
        zone=PlanningZone(name='Example', enabled=True,
                          vertices=[Point(x=0.0, y=0.0), Point(x=1.0, y=0.0), Point(x=1.0, y=1.0)])))
    call('delete', DeletePlanningZone, DeletePlanningZone.Request(
        map_id=result.map_id, expected_revision=result.revision, id=result.id))
finally:
    node.destroy_node()
    rclpy.shutdown()
```
