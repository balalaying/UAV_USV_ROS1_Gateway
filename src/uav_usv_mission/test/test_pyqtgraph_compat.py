import pyqtgraph.opengl as gl
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from uav_usv_mission.pyqtgraph_compat import gl_text_item


def test_gl_text_item_is_available_on_legacy_pyqtgraph():
    item = gl_text_item(pos=(1.0, 2.0, 3.0), text='vehicle')

    assert isinstance(item, GLGraphicsItem)
