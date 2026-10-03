"""Optional gamepad via pygame (SDL). No window: SDL's dummy video driver.

Left stick: forward/back + turn.  A: stand.  B: sit.  BACK/SELECT: E-STOP.
Button numbers follow SDL's usual XInput layout; other pads may differ.
"""
import os

from PySide6.QtCore import QObject, QTimer, Signal

DEAD = 0.15


class Gamepad(QObject):
    walk = Signal(float, float)
    stand = Signal()
    sit = Signal()
    estop = Signal()
    status = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pg = None
        self.js = None
        self.active = False
        self.prev_buttons = {}
        try:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            import pygame
            pygame.display.init()
            pygame.joystick.init()
            self.pg = pygame
        except Exception as e:                         # pygame missing or SDL trouble
            self.status.emit(f"gamepad disabled: {e}")
            return
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(40)

    def poll(self):
        pg = self.pg
        pg.event.pump()
        if self.js is None:
            if pg.joystick.get_count():
                self.js = pg.joystick.Joystick(0)
                self.js.init()
                self.status.emit(f"gamepad: {self.js.get_name()}")
            return
        if not pg.joystick.get_count():
            self.js = None
            self.status.emit("gamepad disconnected")
            return

        def axis(i):
            v = self.js.get_axis(i) if self.js.get_numaxes() > i else 0.0
            return 0.0 if abs(v) < DEAD else v

        fwd, turn = -axis(1), -axis(0)
        moving = fwd != 0 or turn != 0
        if moving or self.active:                      # don't fight the keyboard when idle
            self.walk.emit(fwd, turn)
        self.active = moving

        for b, sig in ((0, self.stand), (1, self.sit), (6, self.estop)):
            down = self.js.get_numbuttons() > b and self.js.get_button(b)
            if down and not self.prev_buttons.get(b):
                sig.emit()
            self.prev_buttons[b] = down
