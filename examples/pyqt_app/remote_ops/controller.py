from __future__ import annotations

import shlex
from collections.abc import Callable

from ..core.command_worker import SshCommandWorker
from ..core.ssh_client import SshClient
from ..models.service import ServiceResult
from .models import SSH_PRESETS


class RemoteOpsController:
    def __init__(self, page) -> None:
        self.page = page
        self.ssh = SshClient()
        self.ssh_worker: SshCommandWorker | None = None
        self.streaming_active = False

    def on_connect_toggle(self) -> None:
        if self.ssh.is_configured:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        host = self.page.ssh_bar.host_input.text().strip()
        port = self.page.ssh_bar.port_spin.value()
        user = self.page.ssh_bar.user_input.text().strip()
        pwd = self.page.ssh_bar.pwd_input.text()

        if not host or not user:
            self.page.append_log("[ERROR] Host and User are required.")
            return

        self.ssh.configure(host, port, user, pwd)

        def task(log, _cancelled):
            log(f"Connecting to {user}@{host}:{port} …")
            result = self.ssh.test_connection()
            if result.exit_code == 0:
                log(f"Connected to {host} — {result.stdout.strip()}")
                return ServiceResult(True, f"SSH connected to {host}")
            self.ssh.configure("", 0, "", "")
            return ServiceResult(False, f"SSH connection failed:\n{result.stderr}")

        self.page.start_worker(task)

    def disconnect(self) -> None:
        self.ssh.configure("", 0, "", "")
        self.page.ssh_bar.connect_button.setText("连接")
        self.page.append_log("SSH disconnected.")

    def apply_preset(self, index: int) -> None:
        if index == 0:
            return
        preset = SSH_PRESETS[index - 1]
        self.page.ssh_bar.host_input.setText(preset.host)
        self.page.ssh_bar.user_input.setText(preset.user)
        self.page.ssh_bar.port_spin.setValue(preset.port)

    def set_running_state(self, running: bool) -> None:
        self.page.set_base_running_state(running)
        if running:
            self.page.ssh_bar.connect_button.setText("断开")
        else:
            connected = self.ssh.is_configured
            self.page.ssh_bar.connect_button.setText("断开" if connected else "连接")

    def refresh_containers(self) -> None:
        if self._ssh_worker_running():
            self.page.append_log("[WARN] Another command is already running.")
            return
        self.page.append_log("Refreshing container list …")
        self.run_ssh_command("docker ps --format '{{.Names}}' 2>/dev/null", on_finish=self._on_containers_loaded)

    def show_container_logs(self) -> None:
        container = self.page.container_panel.container_combo.currentText()
        if not container:
            self.page.append_log("[WARN] No container selected.")
            return
        self.run_ssh_command(f"docker logs --tail 200 {shlex.quote(container)} 2>&1")

    def exec_command_from_input(self) -> None:
        cmd = self.page.command_panel.cmd_input.text().strip()
        if not cmd:
            return
        self.page.command_panel.cmd_input.clear()
        self.run_ssh_command(self.maybe_wrap_container_exec(cmd))

    def run_quick_command(self, command: str, in_container: bool) -> None:
        wrapped = self.maybe_wrap_container_exec(command) if in_container else command
        self.run_ssh_command(wrapped)

    def maybe_wrap_container_exec(self, cmd: str) -> str:
        if not self.page.command_panel.container_exec_cb.isChecked():
            return cmd
        container = self.page.container_panel.container_combo.currentText()
        if not container:
            self.page.append_log("[WARN] 已在容器模式但未选择容器，直接执行于宿主机")
            return cmd
        source_cmd = self.page.command_panel.source_input.text().strip()
        inner = f"{source_cmd} && {cmd}" if source_cmd else cmd
        wrapped = f"docker exec {shlex.quote(container)} bash -c {shlex.quote(inner)}"
        self.page.append_log(f"[容器内执行] {inner}")
        return wrapped

    def run_ssh_command(
        self,
        command: str,
        *,
        streaming: bool = False,
        on_finish: Callable[[int], None] | None = None,
    ) -> None:
        if self._ssh_worker_running():
            self.page.append_log("[WARN] Another command is already running.")
            return
        if not self.ssh.is_configured:
            self.page.append_log("[ERROR] SSH not connected. Please connect first.")
            return

        self.page.append_log(f"$ {command}")
        self.streaming_active = streaming
        self.ssh_worker = SshCommandWorker(self.ssh, command, streaming=streaming)
        self.ssh_worker.output.connect(self.page.append_log)
        self.ssh_worker.output_err.connect(lambda line: self.page.append_log(f"[stderr] {line}"))
        self.ssh_worker.error.connect(lambda msg: self.page.append_log(f"[ERROR] {msg}"))
        self.ssh_worker.finished.connect(self._on_ssh_command_done)

        def handle_finish(exit_code: int) -> None:
            if exit_code != 0:
                self.page.append_log(f"[exit {exit_code}]")
            if on_finish is not None:
                on_finish(exit_code)

        self.ssh_worker.finished.connect(handle_finish)
        self.ssh_worker.start()
        self.set_running_state(True)

    def stop(self) -> None:
        if self._ssh_worker_running():
            self.page.append_log("Stopping …")
            self.ssh_worker.request_stop()

    def shutdown(self) -> None:
        if self._ssh_worker_running():
            self.ssh_worker.request_stop()
            self.ssh_worker.wait(2000)

    def _on_containers_loaded(self, exit_code: int) -> None:
        if exit_code != 0:
            return
        combo = self.page.container_panel.container_combo
        combo.clear()
        for name in self.ssh.list_containers():
            combo.addItem(name)
        self.page.append_log(f"Found {combo.count()} container(s).")

    def _on_ssh_command_done(self, _exit_code: int) -> None:
        self.streaming_active = False
        self.set_running_state(False)
        self.ssh_worker = None

    def _ssh_worker_running(self) -> bool:
        return self.ssh_worker is not None and self.ssh_worker.isRunning()
