import os
from dataclasses import replace
import tkinter as tk
from types import SimpleNamespace

import pytest
from PIL import Image

pytestmark = pytest.mark.skipif(not os.getenv('DISPLAY'), reason='Tk needs an X11 display')
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('bw_interface.srv._replace_planning_zones')

from bw_planning_zones.editor import Editor
from bw_planning_zones.geometry import Zone
from bw_planning_zones.server import zone_msg
from bw_planning_zones.store import Store
from bw_interface.msg import PlanningZoneState
from bw_interface.srv import ListPlanningZones
from rclpy.task import Future


@pytest.fixture
def gui(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    Image.new('L', (80, 60), 254).save(tmp_path / 'map.pgm')
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    rclpy.init(domain_id=185)
    node = rclpy.create_node('planning_zone_gui_test')
    root = tk.Tk()
    errors = []
    root.report_callback_exception = lambda _, error, __: errors.append(error)
    try:
        editor = Editor(root, node)
        editor.load(path)
        root.update()
        editor.fit()
        yield editor
        assert not errors
    finally:
        root.after_cancel(editor.spin_timer)
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()


def event(editor, xy):
    x, y = editor.to_screen(xy)
    return SimpleNamespace(x=round(x), y=round(y))


def state(editor, revision, *zones):
    return PlanningZoneState(map_id=editor.store.spec.id, revision=revision,
                             zones=[zone_msg(zone) for zone in zones])


ZONE = Zone('demo', 'Demo', True, ((0.5, 0.5), (2., 0.5), (2., 2.)))


def test_editor_draws_moves_and_saves(gui):
    gui.new()
    for xy in [(0.5, 0.5), (2.0, 0.5), (2.0, 2.0), (1.0, 1.2), (0.5, 2.0)]:
        gui.click(event(gui, xy))
    gui.finish()
    assert len(gui.zones) == 1 and gui.photo.width() > 1
    original = gui.zones[0].vertices
    gui.click(event(gui, original[0]))
    gui.move_vertex(event(gui, (0.3, 0.4)))
    gui.release()
    assert gui.zones[0].vertices[0] != original[0]
    gui.save_offline()
    assert not gui.dirty
    assert Store(gui.store.spec).snapshot.zones == tuple(gui.zones)


@pytest.mark.parametrize('interaction', ['drag', 'rename'])
def test_external_edit_does_not_rebase_active_interaction(gui, monkeypatch, interaction):
    gui.on_state(state(gui, 1, ZONE))
    gui.selected = ZONE.id
    newer = state(gui, 2, replace(ZONE, enabled=False))
    if interaction == 'drag':
        gui.click(event(gui, ZONE.vertices[0]))
        gui.on_state(newer)
        gui.move_vertex(event(gui, (0.3, 0.4)))
        gui.release()
    else:
        def rename(*args, **kwargs):
            gui.on_state(newer)
            return 'Renamed'
        monkeypatch.setattr('tkinter.simpledialog.askstring', rename)
        gui.rename()
    assert gui.dirty and gui.revision == 1
    gui.client = SimpleNamespace(service_is_ready=lambda: True,
                                 call_async=lambda _: pytest.fail('cannot submit a stale draft'))
    gui.save_online()
    assert gui.pending is None


def test_equal_revision_with_different_base_cannot_submit(gui):
    gui.edit([ZONE])
    gui.on_state(state(gui, gui.revision, replace(ZONE, name='Different config')))
    gui.client = SimpleNamespace(service_is_ready=lambda: True,
                                 call_async=lambda _: pytest.fail('cannot replace a different base'))
    gui.save_online()
    assert gui.pending is None and gui.dirty


@pytest.mark.parametrize('committed', [False, True])
def test_timeout_reconciles_before_retry(gui, monkeypatch, committed):
    gui.on_state(state(gui, 1, ZONE))
    changed = replace(ZONE, name='Changed')
    gui.edit([changed])
    first, query = Future(), Future()
    gui.client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda _: first)
    gui.list_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda _: query)
    gui.save_online()
    monkeypatch.setattr('time.monotonic', lambda: float('inf'))
    gui.expire_request()
    assert gui.pending is None and gui.dirty and gui.uncertain
    gui.client.call_async = lambda _: pytest.fail('must reconcile before retrying')
    gui.save_online()
    assert gui.pending is None
    gui.refresh()
    recorded = state(gui, 2, changed) if committed else state(gui, 1, ZONE)
    query.set_result(ListPlanningZones.Response(success=True, map_id=recorded.map_id,
                                                revision=recorded.revision, zones=recorded.zones))
    assert gui.pending is None and not gui.uncertain and gui.dirty != committed
    assert gui.zones == [changed]
