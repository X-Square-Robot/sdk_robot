from __future__ import annotations

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPlainTextEdit, QPushButton, QScrollArea, QStackedWidget, QVBoxLayout, QWidget

from ..controllers.lifecycle import shutdown_pages
from ..pages.base_page import BasePage
from .navigation import PAGE_SPECS
from .styles import build_stylesheet


class AppWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("X2 Examples GUI")
        self.group_buttons: dict[str, QPushButton] = {}
        self.nav_buttons: dict[str, QPushButton] = {}
        self.nav_specs = {spec.key: spec for spec in PAGE_SPECS}
        self.page_keys: list[str] = []
        self.active_group = "system"

        self._log_buffer: list[str] = []
        self._log_timer = QTimer(self)
        self._log_timer.timeout.connect(self._flush_log_buffer)
        self._log_timer.start(50)

        root = QWidget()
        root.setObjectName("appRoot")
        self.setCentralWidget(root)

        outer = QVBoxLayout(root)
        outer.setContentsMargins(18, 12, 18, 8)
        outer.setSpacing(10)

        outer.addWidget(self._build_header())
        outer.addLayout(self._build_workspace(), 3)
        outer.addWidget(self._build_log_panel(), 1)
        outer.addWidget(self._build_status_bar())

        self._refresh_group_buttons()
        self.set_active_group("system")
        self.setStyleSheet(build_stylesheet())

    def append_log(self, message: str) -> None:
        self._log_buffer.append(message)

    def _flush_log_buffer(self) -> None:
        if not self._log_buffer:
            return
        batch = self._log_buffer[:200]
        del self._log_buffer[:200]
        for line in batch:
            self.log_output.appendPlainText(line)
        cursor = self.log_output.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        self.log_output.setTextCursor(cursor)

    def clear_log(self) -> None:
        self._log_buffer.clear()
        self.log_output.clear()

    def closeEvent(self, event) -> None:  # noqa: N802
        shutdown_pages(self.stack)
        super().closeEvent(event)

    def _build_header(self) -> QWidget:
        shell = QFrame()
        shell.setObjectName("heroShell")
        layout = QHBoxLayout(shell)
        layout.setContentsMargins(18, 12, 18, 12)
        layout.setSpacing(12)

        text_block = QWidget()
        text_layout = QVBoxLayout(text_block)
        text_layout.setContentsMargins(0, 0, 0, 0)
        text_layout.setSpacing(2)

        eyebrow = QLabel("X2 ROBOT CONTROL SURFACE")
        eyebrow.setObjectName("heroEyebrow")
        text_layout.addWidget(eyebrow)

        title = QLabel("X2 Examples GUI")
        title.setObjectName("heroTitle")
        text_layout.addWidget(title)

        subtitle = QLabel("将系统控制、机械臂操作、主从对齐与远程运维整合到统一工作台。")
        subtitle.setObjectName("heroSubtitle")
        subtitle.setWordWrap(True)
        text_layout.addWidget(subtitle)

        layout.addWidget(text_block, 1)
        layout.addWidget(self._build_group_switcher())
        return shell

    def _build_group_switcher(self) -> QWidget:
        box = QFrame()
        box.setObjectName("modeSwitch")
        layout = QHBoxLayout(box)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8)
        for key, label, tip in [
            ("system", "系统控制", "Core robot pages"),
            ("ops", "远程运维", "SSH and container operations"),
        ]:
            button = QPushButton(label)
            button.setCheckable(True)
            button.setToolTip(tip)
            button.clicked.connect(lambda _checked, key=key: self.set_active_group(key))
            self.group_buttons[key] = button
            layout.addWidget(button)
        return box

    def _build_workspace(self):
        layout = QHBoxLayout()
        layout.setSpacing(16)
        layout.addWidget(self._build_sidebar(), 0)
        layout.addWidget(self._build_content_shell(), 1)
        return layout

    def _build_sidebar(self) -> QWidget:
        shell = QFrame()
        shell.setObjectName("sidebarShell")
        shell.setMinimumWidth(260)

        layout = QVBoxLayout(shell)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(10)

        info = QLabel("连接设置")
        info.setObjectName("panelTitle")
        layout.addWidget(info)

        server_card = QFrame()
        server_card.setObjectName("serverCard")
        server_layout = QVBoxLayout(server_card)
        server_layout.setContentsMargins(16, 16, 16, 16)
        server_layout.setSpacing(12)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        label = QLabel("Server")
        label.setObjectName("fieldLabel")
        self.server_input = QLineEdit("localhost:50051")
        self.server_input.setPlaceholderText("例如: 192.168.10.1:50051")
        self.server_input.textChanged.connect(self.update_status_labels)
        clear_button = QPushButton("清空日志")
        clear_button.clicked.connect(self.clear_log)
        top_row.addWidget(label)
        top_row.addWidget(self.server_input, 1)
        top_row.addWidget(clear_button)
        server_layout.addLayout(top_row)

        hint = QLabel("当前 server 会被所有系统控制页面共享使用。")
        hint.setObjectName("fieldHint")
        hint.setWordWrap(True)
        server_layout.addWidget(hint)
        layout.addWidget(server_card)

        nav_title = QLabel("功能导航")
        nav_title.setObjectName("panelTitle")
        layout.addWidget(nav_title)

        nav_shell = QFrame()
        nav_shell.setObjectName("navShell")
        self.nav_layout = QVBoxLayout(nav_shell)
        self.nav_layout.setContentsMargins(8, 8, 8, 8)
        self.nav_layout.setSpacing(8)
        layout.addWidget(nav_shell, 1)
        return shell

    def _build_content_shell(self) -> QWidget:
        shell = QFrame()
        shell.setObjectName("contentShell")
        layout = QVBoxLayout(shell)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(8)

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(12)

        header_text = QVBoxLayout()
        header_text.setContentsMargins(0, 0, 0, 0)
        header_text.setSpacing(2)
        self.current_group_label = QLabel("SYSTEM")
        self.current_group_label.setObjectName("contentEyebrow")
        header_text.addWidget(self.current_group_label)
        self.current_page_title = QLabel("Check Connect")
        self.current_page_title.setObjectName("contentTitle")
        header_text.addWidget(self.current_page_title)
        self.current_page_subtitle = QLabel("")
        self.current_page_subtitle.setObjectName("contentSubtitle")
        self.current_page_subtitle.setWordWrap(True)
        header_text.addWidget(self.current_page_subtitle)
        header_layout.addLayout(header_text, 1)

        self.page_badge = QLabel("PING")
        self.page_badge.setObjectName("pageBadge")
        header_layout.addWidget(self.page_badge, 0)
        layout.addWidget(header)

        line = QFrame()
        line.setObjectName("divider")
        line.setFrameShape(QFrame.Shape.HLine)
        layout.addWidget(line)

        self.stack = QStackedWidget()
        self.stack.currentChanged.connect(self.on_page_changed)
        for spec in PAGE_SPECS:
            self._register_page(spec)

        self.nav_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(self.stack)
        layout.addWidget(scroll, 1)
        return shell

    def _register_page(self, spec) -> None:
        page = spec.page_cls(self)
        self.page_keys.append(spec.key)
        button = QPushButton("进入")
        button.setCheckable(True)
        button.clicked.connect(lambda _checked, key=spec.key: self.set_current_page(key))
        self.nav_buttons[spec.key] = button

        row = QFrame()
        row.setObjectName("navButtonRow")
        row.setProperty("page_key", spec.key)
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(10, 10, 10, 10)
        row_layout.setSpacing(12)

        badge = QLabel(spec.badge)
        badge.setObjectName("navBadge")
        title_col = QVBoxLayout()
        title_col.setContentsMargins(0, 0, 0, 0)
        title_col.setSpacing(2)
        title_label = QLabel(spec.label)
        title_label.setObjectName("navLabel")
        subtitle_label = QLabel("System tools" if spec.group == "system" else "Remote diagnostics")
        subtitle_label.setObjectName("navSubLabel")
        title_col.addWidget(title_label)
        title_col.addWidget(subtitle_label)
        row_layout.addWidget(badge)
        row_layout.addLayout(title_col, 1)
        row_layout.addWidget(button)
        self.nav_layout.addWidget(row)
        self.stack.addWidget(page)

    def _build_log_panel(self) -> QWidget:
        box = QFrame()
        box.setObjectName("logShell")
        layout = QVBoxLayout(box)
        layout.setContentsMargins(14, 8, 14, 10)
        layout.setSpacing(6)

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(10)
        title = QLabel("日志输出")
        title.setObjectName("panelTitle")
        header_layout.addWidget(title)
        header_layout.addStretch(1)
        log_hint = QLabel("Live terminal feed")
        log_hint.setObjectName("logHint")
        header_layout.addWidget(log_hint)
        layout.addWidget(header)

        self.log_output = QPlainTextEdit()
        self.log_output.setObjectName("logOutput")
        self.log_output.setReadOnly(True)
        self.log_output.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.log_output.setMinimumHeight(80)
        layout.addWidget(self.log_output)
        return box

    def _build_status_bar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("statusShell")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(8, 0, 8, 0)
        layout.setSpacing(14)
        self.status_server = QLabel()
        self.status_page = QLabel()
        self.status_group = QLabel()
        for label in (self.status_server, self.status_page, self.status_group):
            label.setObjectName("statusChip")
            layout.addWidget(label)
        layout.addStretch(1)
        self.update_status_labels()
        return bar

    def set_active_group(self, group: str) -> None:
        self.active_group = group
        self._refresh_group_buttons()

        visible_keys = []
        for index in range(self.nav_layout.count()):
            item = self.nav_layout.itemAt(index)
            widget = item.widget()
            if widget is None or widget.objectName() != "navButtonRow":
                continue
            key = widget.property("page_key")
            spec = self.nav_specs[key]
            should_show = spec.group == group
            widget.setVisible(should_show)
            if should_show:
                visible_keys.append(key)

        current_key = self.page_keys[self.stack.currentIndex()] if self.stack.count() else ""
        if current_key not in visible_keys and visible_keys:
            self.set_current_page(visible_keys[0])
        else:
            self._sync_nav_state(current_key)
            self.update_content_header()

    def set_current_page(self, key: str) -> None:
        if key not in self.nav_specs:
            return
        self.stack.setCurrentIndex(self.page_keys.index(key))
        self._sync_nav_state(key)
        self.update_content_header()

    def on_page_changed(self, index: int) -> None:
        if index < 0:
            return
        key = self.page_keys[index]
        self._sync_nav_state(key)
        self.update_content_header()

    def update_content_header(self) -> None:
        if self.stack.currentIndex() < 0:
            return
        key = self.page_keys[self.stack.currentIndex()]
        spec = self.nav_specs[key]
        page = self.stack.currentWidget()
        self.current_group_label.setText(spec.group.upper())
        self.current_page_title.setText(spec.label)
        subtitle = page.subtitle if isinstance(page, BasePage) else ""
        self.current_page_subtitle.setText(subtitle)
        self.page_badge.setText(spec.badge)
        self.update_status_labels()

    def update_status_labels(self) -> None:
        if not hasattr(self, "status_server"):
            return
        page_label = "No page"
        group_label = self.active_group.upper()
        if self.stack.currentIndex() >= 0:
            key = self.page_keys[self.stack.currentIndex()]
            page_label = self.nav_specs[key].label
        self.status_server.setText(f"SDK  {self.server_input.text().strip() or 'localhost:50051'}")
        self.status_page.setText(f"PAGE  {page_label}")
        self.status_group.setText(f"MODE  {group_label}")

    def _refresh_group_buttons(self) -> None:
        for key, button in self.group_buttons.items():
            button.setChecked(key == self.active_group)

    def _sync_nav_state(self, current_key: str) -> None:
        for key, button in self.nav_buttons.items():
            button.setChecked(key == current_key)
