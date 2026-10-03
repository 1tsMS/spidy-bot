"""Graphite + mint. One accent, used sparingly; colour always means something:
mint = sim / ok, amber = real hardware, red = armed / danger."""

BG = "#0b0d11"          # window
SURFACE = "#12151b"     # panels
SURFACE_2 = "#181c24"   # controls, inputs
SURFACE_3 = "#20252f"   # hover / selected
BORDER = "#1f242d"
BORDER_2 = "#2b313c"
TEXT = "#e7eaf0"
TEXT_2 = "#9aa3b2"
TEXT_3 = "#5f6878"
ACCENT = "#6ee7b7"      # mint
ACCENT_INK = "#06281c"  # text on mint
BLUE = "#7aa2ff"
AMBER = "#f5b94a"
RED = "#ff5c6c"
GREEN = ACCENT

# legacy names some modules still use
ORANGE, YELLOW, DUST, BONE, INK, RIVET, PLATE = ACCENT, AMBER, TEXT_2, TEXT, BG, BORDER_2, SURFACE

UI = "Segoe UI"
MONO = "Cascadia Mono"
ICONS = "Segoe Fluent Icons"
FONT = UI

MODE_COLOR = {"sim": ACCENT, "real": AMBER, "both": BLUE}

QSS = f"""
* {{ font-family: "{UI}"; font-size: 13px; color: {TEXT}; outline: none; }}
QMainWindow, QWidget#root, QStackedWidget {{ background: {BG}; }}
QWidget {{ background: transparent; }}
QToolTip {{ background: {SURFACE_3}; color: {TEXT}; border: 1px solid {BORDER_2}; border-radius: 6px; padding: 5px 8px; }}

QFrame#panel {{ background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 12px; }}
QFrame#topbar {{ background: {BG}; border-bottom: 1px solid {BORDER}; }}
QFrame#rail {{ background: {SURFACE}; border-right: 1px solid {BORDER}; }}
QFrame#statusbar {{ background: {SURFACE}; border-top: 1px solid {BORDER}; }}
QFrame#drawer {{ background: {SURFACE}; border-top: 1px solid {BORDER}; }}
QFrame#sep {{ background: {BORDER}; max-height: 1px; min-height: 1px; border: none; }}

QLabel#h1 {{ font-size: 16px; font-weight: 600; }}
QLabel#title {{ font-size: 15px; font-weight: 600; }}
QLabel#caps {{ color: {TEXT_3}; font-size: 11px; font-weight: 600; }}
QLabel#dim, QLabel#muted {{ color: {TEXT_2}; }}
QLabel#faint {{ color: {TEXT_3}; font-size: 12px; }}
QLabel#callout {{ background: {SURFACE_2}; border: 1px solid {BORDER_2}; border-radius: 10px; padding: 10px 12px; color: {TEXT}; }}
QFrame#panel QFrame#panel {{ background: {SURFACE_2}; border: 1px solid {BORDER_2}; border-radius: 10px; }}
QLabel#mono {{ font-family: "{MONO}"; font-size: 12px; color: {TEXT_2}; }}
QLabel#big, QLabel#value {{ font-family: "{MONO}"; font-size: 15px; font-weight: 600; color: {TEXT}; }}
QLabel#stamp {{ color: {TEXT_2}; background: {SURFACE_2}; border: 1px solid {BORDER_2}; border-radius: 6px;
  padding: 1px 7px; font-size: 11px; font-weight: 600; }}

QPushButton {{ background: {SURFACE_2}; border: 1px solid {BORDER_2}; border-radius: 8px;
  padding: 6px 12px; font-weight: 500; }}
QPushButton:hover {{ background: {SURFACE_3}; border-color: #3a4250; }}
QPushButton:pressed {{ background: {BORDER_2}; }}
QPushButton:checked {{ background: {SURFACE_3}; border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton:disabled {{ color: {TEXT_3}; background: {SURFACE}; border-color: {BORDER}; }}
QPushButton#accent, QPushButton#primary {{ background: {ACCENT}; color: {ACCENT_INK}; border: 1px solid {ACCENT}; font-weight: 600; }}
QPushButton#accent:hover, QPushButton#primary:hover {{ background: #8ff0c9; }}
QPushButton#accent:disabled {{ background: {SURFACE_2}; color: {TEXT_3}; border-color: {BORDER}; }}
QPushButton#ghost {{ background: transparent; border: 1px solid transparent; color: {TEXT_2}; }}
QPushButton#ghost:hover {{ background: {SURFACE_2}; color: {TEXT}; }}
QPushButton#estop {{ background: {RED}; color: #ffffff; border: none; border-radius: 9px;
  padding: 7px 14px; font-weight: 700; }}
QPushButton#estop:hover {{ background: #ff7683; }}
QPushButton#estop:pressed {{ background: #d9404f; }}

QToolButton#nav {{ background: transparent; border: none; border-radius: 10px; color: {TEXT_3};
  font-size: 10px; font-weight: 600; padding: 7px 2px 5px 2px; }}
QToolButton#nav:hover {{ background: {SURFACE_2}; color: {TEXT_2}; }}
QToolButton#nav:checked {{ background: {SURFACE_3}; color: {TEXT}; }}

QFrame#seg {{ background: {SURFACE_2}; border: 1px solid {BORDER_2}; border-radius: 9px; }}
QPushButton#segbtn {{ background: transparent; border: none; border-radius: 7px; padding: 4px 12px;
  color: {TEXT_2}; font-weight: 600; }}
QPushButton#segbtn:hover {{ color: {TEXT}; }}
QPushButton#segbtn:checked {{ background: {SURFACE_3}; color: {TEXT}; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
  background: {SURFACE_2}; border: 1px solid {BORDER_2}; border-radius: 8px; padding: 5px 9px;
  selection-background-color: {ACCENT}; selection-color: {ACCENT_INK}; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {ACCENT}; }}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ min-height: 18px; }}
QSpinBox, QDoubleSpinBox {{ font-family: "{MONO}"; font-size: 12px; }}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{ width: 0; border: none; }}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{ background: {SURFACE_2}; border: 1px solid {BORDER_2}; border-radius: 8px;
  padding: 4px; selection-background-color: {SURFACE_3}; selection-color: {TEXT}; }}

QPlainTextEdit#console {{ background: {BG}; border: 1px solid {BORDER}; border-radius: 10px; padding: 6px;
  font-family: "{MONO}"; font-size: 12px; color: {TEXT_2}; }}

QListWidget {{ background: transparent; border: none; }}
QListWidget::item {{ border-radius: 8px; padding: 6px; margin: 1px 0; color: {TEXT_2}; }}
QListWidget::item:hover {{ background: {SURFACE_2}; color: {TEXT}; }}
QListWidget::item:selected {{ background: {SURFACE_3}; color: {TEXT}; }}

QTableWidget {{ background: transparent; border: none; gridline-color: {BORDER}; font-family: "{MONO}"; font-size: 12px; }}
QTableWidget::item {{ padding: 4px; }}
QHeaderView::section {{ background: transparent; color: {TEXT_3}; border: none; border-bottom: 1px solid {BORDER};
  padding: 6px 4px; font-size: 11px; font-weight: 600; }}

QSlider::groove:horizontal {{ height: 4px; background: {BORDER_2}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 2px; }}
QSlider::handle:horizontal {{ background: {TEXT}; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }}

QCheckBox {{ color: {TEXT_2}; spacing: 8px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border-radius: 5px; border: 1px solid {BORDER_2}; background: {SURFACE_2}; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}

QScrollBar:vertical {{ background: transparent; width: 8px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {BORDER_2}; border-radius: 3px; min-height: 30px; }}
QScrollBar:horizontal {{ background: transparent; height: 8px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {BORDER_2}; border-radius: 3px; min-width: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line, QScrollBar::add-page, QScrollBar::sub-page {{ background: none; height: 0; width: 0; }}
QSplitter::handle {{ background: {BORDER}; }}
QScrollArea {{ border: none; }}
"""


def load_color(pct):
    """Servo load colour: mint < 60%, amber < 85%, red above."""
    return ACCENT if pct < 60 else (AMBER if pct < 85 else RED)


def rgba(hex_color, a=1.0):
    h = hex_color.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)) + (a,)
