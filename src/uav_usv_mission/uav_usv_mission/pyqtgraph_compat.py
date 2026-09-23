"""Compatibility helpers for the pyqtgraph versions shipped with ROS 1."""

import pyqtgraph.opengl as gl
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QFont


if not hasattr(gl, 'GLTextItem'):
    from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

    class _LegacyGLTextItem(GLGraphicsItem):
        """Backport the small GLTextItem API missing from pyqtgraph 0.11."""

        def __init__(
            self, pos=(0.0, 0.0, 0.0), color=None, text='', font=None,
            parentItem=None,
        ):
            super().__init__(parentItem=parentItem)
            self.setGLOptions('translucent')
            self.pos = tuple(float(value) for value in pos)
            self.color = QColor(color) if color is not None else QColor('white')
            self.text = str(text)
            self.font = QFont(font) if font is not None else QFont()

        def paint(self):
            self.setupGLState()
            view = self.view()
            if view is None or not self.text:
                return
            view.qglColor(self.color)
            view.renderText(
                self.pos[0], self.pos[1], self.pos[2], self.text, self.font
            )

else:
    _LegacyGLTextItem = None


def gl_text_item(**kwargs):
    """Create a 3D text item on both legacy and current pyqtgraph."""
    item_class = getattr(gl, 'GLTextItem', None) or _LegacyGLTextItem
    return item_class(**kwargs)
