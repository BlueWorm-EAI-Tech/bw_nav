import argparse
from dataclasses import replace
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk
import uuid

from bw_interface.msg import PlanningZoneState
from bw_interface.srv import ListPlanningZones, ReplacePlanningZones
from PIL import Image
import rclpy
from rclpy.utilities import remove_ros_args
from shapely.geometry import Point, Polygon
import yaml

from .geometry import MapSpec, Zone
from .server import SNAPSHOT_QOS, zone_from_msg, zone_msg
from .store import Store

AFFINE = Image.Transform.AFFINE if hasattr(Image, 'Transform') else Image.AFFINE
REQUEST_TIMEOUT = 5.0


class Editor:
    def __init__(self, root, node):
        self.root, self.node = root, node
        self.store = self.bitmap = self.photo = self.online = self.pending = None
        self.zones, self.points = [], None
        self.base_zones = self.online_zones = ()
        self.selected = self.drag = self.pan = None
        self.revision, self.dirty = 0, False
        self.renaming = self.uncertain = False
        self.scale, self.u, self.v = 1.0, 0.0, 0.0
        root.title('规划禁行区')
        root.geometry('1040x660')
        root.minsize(740, 420)
        root.protocol('WM_DELETE_WINDOW', self.close)

        toolbar = ttk.Frame(root, padding=(10, 8))
        toolbar.pack(fill='x')
        actions = [('打开地图', self.open), ('＋ 区域', self.new), ('↶', self.undo),
                   ('闭合', self.finish), ('适应窗口', self.fit)]
        for label, command in actions:
            ttk.Button(toolbar, text=label, command=command, width=10).pack(side='left', padx=(0, 4))
        self.submit = ttk.Button(toolbar, text='提交修改', command=self.save_online)
        self.submit.pack(side='right', padx=(4, 0))
        self.save = ttk.Button(toolbar, text='离线保存', command=self.save_offline)
        self.save.pack(side='right')

        body = ttk.Panedwindow(root, orient='horizontal')
        body.pack(fill='both', expand=True)
        self.canvas = tk.Canvas(body, bg='#303437', highlightthickness=0, width=510)
        body.add(self.canvas, weight=5)
        panel = ttk.Frame(body, padding=(9, 8), width=215)
        body.add(panel, weight=1)
        ttk.Label(panel, text='区域').pack(anchor='w', pady=(0, 6))
        self.tree = ttk.Treeview(panel, columns=('enabled',), show='tree headings', selectmode='browse')
        self.tree.heading('#0', text='名称', anchor='w')
        self.tree.heading('enabled', text='状态', anchor='w')
        self.tree.column('#0', width=140, minwidth=60)
        self.tree.column('enabled', width=65, stretch=False)
        self.tree.pack(fill='both', expand=True)
        self.tree.bind('<<TreeviewSelect>>', self.pick)
        self.tree.bind('<Double-1>', lambda _: self.rename())
        for label, command in [('重命名', self.rename), ('启用 / 禁用', self.toggle),
                               ('删除', self.delete), ('读取当前', self.refresh)]:
            ttk.Button(panel, text=label, command=command).pack(fill='x', pady=(5, 0))

        footer = ttk.Frame(root, padding=(10, 5))
        footer.pack(fill='x')
        self.status = ttk.Label(footer, text='未打开地图')
        self.status.pack(side='left')
        self.position = ttk.Label(footer)
        self.position.pack(side='right')
        self.canvas.bind('<Configure>', lambda _: self.draw())
        self.canvas.bind('<Button-1>', self.click)
        self.canvas.bind('<B1-Motion>', self.move_vertex)
        self.canvas.bind('<ButtonRelease-1>', self.release)
        for button in (2, 3):
            self.canvas.bind(f'<Button-{button}>', self.pan_start)
            self.canvas.bind(f'<B{button}-Motion>', self.pan_move)
        for event in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
            self.canvas.bind(event, self.zoom)
        self.canvas.bind('<Motion>', self.cursor)
        root.bind('<Return>', lambda _: self.finish())
        root.bind('<Escape>', lambda _: self.cancel())
        root.bind('<BackSpace>', lambda _: self.undo())
        root.bind('<Control-s>', lambda _: self.save_online() if self.connected() else self.save_offline())
        self.node.create_subscription(PlanningZoneState, 'planning_zones/state', self.on_state, SNAPSHOT_QOS)
        self.client = self.node.create_client(ReplacePlanningZones, 'planning_zones/replace_all')
        self.list_client = self.node.create_client(ListPlanningZones, 'planning_zones/list')
        self.controls()
        self.spin_timer = root.after(40, self.spin)

    def connected(self):
        return (self.store is not None and self.online is not None and
                self.online.map_id == self.store.spec.id and self.client.service_is_ready())

    def editing(self):
        return self.points is not None or self.drag is not None or self.renaming

    def matches_base(self):
        return self.revision == self.online.revision and self.base_zones == self.online_zones

    def can_submit(self):
        return (self.connected() and not self.editing() and not self.pending and
                not self.uncertain and self.matches_base())

    def can_save_offline(self):
        return self.store and not (self.connected() or self.editing() or self.pending or self.uncertain)

    def controls(self):
        self.submit.configure(state='normal' if self.can_submit() else 'disabled')
        self.save.configure(state='normal' if self.can_save_offline() else 'disabled')

    def spin(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)
        self.expire_request()
        self.controls()
        self.spin_timer = self.root.after(40, self.spin)

    def expire_request(self):
        if self.pending is not None and time.monotonic() >= self.pending_deadline:
            future, self.pending = self.pending, None
            future.cancel()
            self.mark_uncertain()

    def discard(self):
        return (self.pending is None and (not (self.dirty or self.uncertain or self.editing()) or
                messagebox.askyesno('未保存的修改', '放弃当前草稿？', parent=self.root)))

    def close(self):
        if self.discard():
            self.root.after_cancel(self.spin_timer)
            self.root.destroy()

    def open(self):
        if not self.discard():
            return
        path = filedialog.askopenfilename(filetypes=[('地图 YAML', '*.yaml *.yml')], parent=self.root)
        if path:
            self.load(path)

    def load(self, path, config=None):
        try:
            store = Store(MapSpec.load(path), config)
            with Image.open(store.spec.image) as bitmap:
                self.bitmap = bitmap.convert('RGB')
        except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
            self.error(error)
            return
        self.store = store
        self.dirty = self.uncertain = False
        self.points = self.drag = self.selected = None
        self.base_zones = store.snapshot.zones
        self.zones = list(self.base_zones)
        self.revision = store.snapshot.revision
        self.root.title(f'规划禁行区 · {store.spec.path.name}')
        self.sync_list()
        if self.connected():
            self.receive_online()
        else:
            self.status.configure(text=f'离线 · 修订 {self.revision}')
        self.root.after_idle(self.fit)

    def on_state(self, msg):
        self.online = msg
        self.online_zones = tuple(zone_from_msg(zone) for zone in msg.zones)
        self.sync_online()

    def sync_online(self):
        msg = self.online
        if not self.store or msg is None or msg.map_id != self.store.spec.id or self.uncertain:
            return
        if not (self.dirty or self.editing() or self.pending):
            self.receive_online()
        elif not self.matches_base():
            self.status.configure(text='线上区域与草稿基准不同 · 草稿已保留')

    def receive_online(self):
        self.base_zones = self.online_zones
        self.zones = list(self.base_zones)
        self.revision = self.online.revision
        self.dirty = False
        self.points = self.drag = None
        self.status.configure(text=f'在线 · 修订 {self.revision}')
        self.sync_list()
        self.draw()

    def refresh(self):
        if self.uncertain:
            if not self.pending and self.list_client.service_is_ready():
                request = ListPlanningZones.Request(map_id=self.store.spec.id)
                self.request(self.list_client, request, self.reconciled)
            return
        if not self.store or not self.discard():
            return
        if self.connected():
            self.receive_online()
        else:
            self.load(self.store.spec.path, self.store.path if self.store.required else None)

    def sync_list(self):
        self.tree.delete(*self.tree.get_children())
        for zone in self.zones:
            self.tree.insert('', 'end', iid=zone.id, text=zone.name or zone.id,
                             values=('已启用' if zone.enabled else '已禁用',))
        if self.selected is not None and self.tree.exists(self.selected):
            self.tree.selection_set(self.selected)
        else:
            self.selected = None

    def pick(self, _event=None):
        selected = self.tree.selection()
        self.selected = selected[0] if selected else None
        self.draw()

    def edit(self, zones):
        self.zones = list(zones)
        self.dirty = True
        self.status.configure(text=f'未提交 · 基于修订 {self.revision}')
        self.sync_online()
        self.sync_list()
        self.draw()

    def selected_zone(self):
        return next((zone for zone in self.zones if zone.id == self.selected), None)

    def update_zone(self, changed):
        self.edit(changed if zone.id == changed.id else zone for zone in self.zones)

    def new(self):
        if self.store and not (self.editing() or self.pending):
            self.points = []
            self.draw()

    def undo(self):
        if self.points:
            self.points.pop()
            self.draw()

    def cancel(self):
        self.points = self.drag = None
        self.sync_online()
        self.draw()

    def finish(self):
        if self.points is None:
            return
        try:
            zone = Zone(uuid.uuid4().hex, f'区域 {len(self.zones) + 1}', True,
                        tuple(self.points)).normalized(self.store.spec)
        except ValueError as error:
            self.error(error)
            return
        self.selected, self.points = zone.id, None
        self.edit([*self.zones, zone])

    def rename(self):
        zone = self.selected_zone()
        if zone and not (self.pending or self.editing()):
            self.renaming = True
            try:
                name = simpledialog.askstring('重命名', '名称', initialvalue=zone.name, parent=self.root)
            finally:
                self.renaming = False
            if name is not None:
                self.update_zone(replace(zone, name=name))
            else:
                self.sync_online()

    def toggle(self):
        zone = self.selected_zone()
        if zone and not (self.pending or self.editing()):
            self.update_zone(replace(zone, enabled=not zone.enabled))

    def delete(self):
        if self.selected_zone() and not (self.pending or self.editing()):
            self.edit(zone for zone in self.zones if zone.id != self.selected)

    def error(self, error):
        messagebox.showerror('未完成', str(error), parent=self.root)

    def save_offline(self):
        if not self.can_save_offline():
            return
        if self.base_zones != self.store.snapshot.zones:
            self.error('离线文件与草稿基准不同')
            return
        try:
            self.store.mutate('replace_all', self.store.spec.id, self.revision, zones=self.zones)
        except (OSError, ValueError, yaml.YAMLError) as error:
            self.error(error)
            return
        self.revision = self.store.snapshot.revision
        self.base_zones = self.store.snapshot.zones
        self.dirty = False
        self.status.configure(text=f'已离线保存 · 修订 {self.revision}')

    def save_online(self):
        if not self.can_submit():
            return
        request = ReplacePlanningZones.Request(map_id=self.store.spec.id, expected_revision=self.revision,
                                              zones=[zone_msg(zone) for zone in self.zones])
        self.request(self.client, request, self.saved)

    def request(self, client, request, completed):
        try:
            self.pending = client.call_async(request)
        except Exception as error:
            self.error(error)
            return
        self.pending_deadline = time.monotonic() + REQUEST_TIMEOUT

        def done(future):
            if future is not self.pending:
                return
            self.pending = None
            try:
                result = future.result()
            except Exception:
                self.mark_uncertain()
                return
            if result is None:
                self.mark_uncertain()
            elif not result.success:
                self.error(f'{result.error_code}: {result.message}')
            else:
                completed(result)

        self.pending.add_done_callback(done)

    def mark_uncertain(self):
        self.uncertain = True
        self.status.configure(text='未收到服务响应 · 请点「读取当前」核对提交结果')

    def saved(self, result):
        self.revision, self.dirty = result.revision, False
        self.base_zones = tuple(self.zones)
        self.status.configure(text=f'已提交 · 修订 {self.revision}' if result.published else result.message)
        if self.online and self.online.map_id == result.map_id and self.online.revision >= self.revision:
            self.receive_online()

    def reconciled(self, result):
        # A timed-out write may have committed. Keep the draft unless it already matches.
        self.on_state(PlanningZoneState(map_id=result.map_id, revision=result.revision, zones=result.zones))
        self.uncertain = False
        if tuple(self.zones) == self.online_zones:
            self.receive_online()
        elif self.matches_base():
            self.status.configure(text=f'可重新提交 · 基于修订 {self.revision}')
        else:
            self.dirty = True
            self.sync_online()

    def fit(self):
        if not self.bitmap:
            return
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        self.scale = min(w / self.bitmap.width, h / self.bitmap.height) * 0.96
        self.u = (w - self.scale * self.bitmap.width) / 2
        self.v = (h - self.scale * self.bitmap.height) / 2
        self.draw()

    def to_world(self, x, y):
        return self.store.spec.pixel_to_world((x - self.u) / self.scale, (y - self.v) / self.scale)

    def to_screen(self, point):
        x, y = self.store.spec.world_to_pixel(*point)
        return self.u + x * self.scale, self.v + y * self.scale

    def cursor(self, event):
        if self.store:
            x, y = self.to_world(event.x, event.y)
            self.position.configure(text=f'{x:8.2f}, {y:8.2f} m   ·   {self.scale * 100:.0f}%')

    def click(self, event):
        if not self.store or self.pending:
            return
        x, y = event.x, event.y
        if not (0 <= (x - self.u) / self.scale <= self.bitmap.width and
                0 <= (y - self.v) / self.scale <= self.bitmap.height):
            return
        point = self.to_world(x, y)
        if self.points is not None:
            if len(self.points) >= 3 and self.near(self.points[0], x, y):
                self.finish()
            else:
                self.points.append(point)
                self.draw()
            return
        zone = self.selected_zone()
        if zone:
            for i, vertex in enumerate(zone.vertices):
                if self.near(vertex, x, y):
                    self.drag = (zone, i, vertex)
                    return
        self.selected = next((z.id for z in reversed(self.zones)
                              if Polygon(z.vertices).covers(Point(point))), None)
        self.sync_list()
        self.draw()

    def near(self, vertex, x, y):
        u, v = self.to_screen(vertex)
        return (u - x)**2 + (v - y)**2 <= 8**2

    def move_vertex(self, event):
        if self.drag:
            zone, index, _ = self.drag
            u = min(max((event.x - self.u) / self.scale, 0), self.bitmap.width)
            v = min(max((event.y - self.v) / self.scale, 0), self.bitmap.height)
            self.drag = zone, index, self.store.spec.pixel_to_world(u, v)
            self.draw()

    def release(self, _event=None):
        if self.drag:
            zone, index, point = self.drag
            self.drag = None
            vertices = list(zone.vertices)
            if point == vertices[index]:
                self.sync_online()
                return
            vertices[index] = point
            try:
                self.update_zone(replace(zone, vertices=tuple(vertices)).normalized(self.store.spec))
            except ValueError as error:
                self.sync_online()
                self.draw()
                self.error(error)

    def pan_start(self, event):
        self.pan = event.x - self.u, event.y - self.v

    def pan_move(self, event):
        if self.pan:
            self.u, self.v = event.x - self.pan[0], event.y - self.pan[1]
            self.draw()

    def zoom(self, event):
        if not self.bitmap:
            return
        factor = 1.22 if event.num == 4 or event.delta > 0 else 1 / 1.22
        new = max(0.005, min(80.0, self.scale * factor))
        ratio = new / self.scale
        self.u = event.x - (event.x - self.u) * ratio
        self.v = event.y - (event.y - self.v) * ratio
        self.scale = new
        self.draw()

    def draw(self):
        self.canvas.delete('all')
        if not self.bitmap:
            return
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        inv = 1 / self.scale
        img = self.bitmap.transform((w, h), AFFINE,
                                    (inv, 0, -self.u * inv, 0, inv, -self.v * inv),
                                    fillcolor='#303437')
        self.photo = tk.PhotoImage(data=f'P6\n{w} {h}\n255\n'.encode() + img.tobytes(), format='PPM')
        self.canvas.create_image(0, 0, image=self.photo, anchor='nw')
        for zone in self.zones:
            vertices = list(zone.vertices)
            if self.drag and self.drag[0].id == zone.id:
                vertices[self.drag[1]] = self.drag[2]
            xy = [self.to_screen(p) for p in vertices]
            selected = zone.id == self.selected
            self.canvas.create_polygon(*xy, fill='#d63531' if zone.enabled else '', stipple='gray50',
                                       outline='#ff8176' if zone.enabled else '#838e96',
                                       width=2 if selected else 1)
            if selected:
                self.dots(xy, '#ffe7df')
        if self.points:
            xy = [self.to_screen(p) for p in self.points]
            if len(xy) > 1:
                self.canvas.create_line(*xy, fill='#ffd480', width=2)
            self.dots(xy, '#ffd480')

    def dots(self, points, color):
        for x, y in points:
            self.canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill=color, outline='#303437')


def main():
    parser = argparse.ArgumentParser(description='Edit polygon planning zones')
    parser.add_argument('--map', type=Path)
    parser.add_argument('--zones-file', type=Path)
    args = parser.parse_args(remove_ros_args()[1:])
    if args.zones_file and not args.map:
        parser.error('--zones-file requires --map')
    rclpy.init()
    node = rclpy.create_node(f'planning_zone_editor_{uuid.uuid4().hex[:8]}')
    try:
        root = tk.Tk()
        editor = Editor(root, node)
        if args.map:
            editor.load(args.map, args.zones_file)
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
