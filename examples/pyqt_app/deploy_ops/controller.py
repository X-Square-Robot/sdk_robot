from __future__ import annotations

import os
import shlex
import subprocess

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFileDialog, QListWidgetItem

from ..core.ssh_client import SshClient
from ..models.service import ServiceResult
from .models import DEPLOY_PRESETS


class DeployController:
    def __init__(self, page) -> None:
        self.page = page
        self.ssh = SshClient()

    @property
    def _bar(self):
        return self.page.ssh_bar

    @property
    def _source(self):
        return self.page.source_panel

    @property
    def _target(self):
        return self.page.target_panel

    # ── SSH ────────────────────────────────────────────────────────

    def on_connect_toggle(self) -> None:
        if self.ssh.is_configured:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        host = self._bar.host_input.text().strip()
        port = self._bar.port_spin.value()
        user = self._bar.user_input.text().strip()
        pwd = self._bar.pwd_input.text()

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
        self._bar.connect_button.setText("连接")
        self.page.append_log("SSH disconnected.")

    def apply_preset(self, index: int) -> None:
        if index == 0:
            return
        preset = DEPLOY_PRESETS[index - 1]
        self._bar.host_input.setText(preset.host)
        self._bar.user_input.setText(preset.user)
        self._bar.port_spin.setValue(preset.port)
        self._target.container_combo.setCurrentText(preset.container_name)
        self._target.container_path_input.setText(preset.container_path)
        self._target.temp_dir_input.setText(preset.temp_dir)

    def set_running_state(self, running: bool) -> None:
        self.page.set_base_running_state(running)
        if running:
            self._bar.connect_button.setText("断开")
        else:
            connected = self.ssh.is_configured
            self._bar.connect_button.setText("断开" if connected else "连接")

    # ── Source scanning ────────────────────────────────────────────

    def scan_subdirs(self) -> None:
        base = self._source.dir_input.text().strip() or "."
        self._source.subdir_list.clear()
        if not os.path.isdir(base):
            self.page.append_log(f"[WARN] 目录不存在: {base}")
            return
        try:
            entries = sorted(os.listdir(base))
        except OSError as exc:
            self.page.append_log(f"[ERROR] 无法读取目录: {exc}")
            return
        for name in entries:
            if os.path.isdir(os.path.join(base, name)) and not name.startswith("."):
                item = QListWidgetItem(name)
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(Qt.CheckState.Checked)
                self._source.subdir_list.addItem(item)
        self.page.append_log(f"扫描完成，发现 {self._source.subdir_list.count()} 个子目录。")

    def _checked_subdirs(self) -> list[str]:
        result = []
        for i in range(self._source.subdir_list.count()):
            item = self._source.subdir_list.item(i)
            if item.checkState() == Qt.CheckState.Checked:
                result.append(item.text())
        return result

    # ── Browse ─────────────────────────────────────────────────────

    def browse_source_dir(self) -> None:
        path = QFileDialog.getExistingDirectory(self.page, "选择本地部署源目录", self._source.dir_input.text())
        if path:
            self._source.dir_input.setText(path)

    # ── Step 1: Rsync ──────────────────────────────────────────────

    def do_rsync(self) -> None:
        if not self._ensure_ssh():
            return

        subdirs = self._checked_subdirs()
        if not subdirs:
            self.page.append_log("[ERROR] 请先扫描并勾选要同步的子目录。")
            return

        base_dir = self._source.dir_input.text().strip() or "."
        host = self._bar.host_input.text().strip()
        port = self._bar.port_spin.value()
        user = self._bar.user_input.text().strip()
        pwd = self._bar.pwd_input.text()
        temp_dir = self._target.temp_dir_input.text().strip()

        ssh_opts = (
            f"-o StrictHostKeyChecking=no "
            f"-o UserKnownHostsFile=/dev/null "
            f"-o ConnectTimeout=10 "
            f"-p {port}"
        )
        rsh = f"sshpass -e ssh {ssh_opts}"

        tasks = []
        for subdir in subdirs:
            src = os.path.join(base_dir, subdir)
            dst = f"{user}@{host}:{temp_dir}/"
            tasks.append((subdir, src, dst))

        self.page.append_log(f"Rsync {len(tasks)} 个子目录 → {user}@{host}:{temp_dir}/")

        def task(log, cancelled):
            os.environ["SSHPASS"] = pwd
            failed = []
            for subdir, src, dst in tasks:
                if cancelled():
                    log("Rsync cancelled.")
                    return ServiceResult(False, "Rsync cancelled by user.")
                log(f"\n--- rsync: {subdir} ---")
                cmd = ["rsync", "-avz", "--rsh", rsh, src, dst]
                log(f"$ {' '.join(cmd)}")
                try:
                    with subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        bufsize=1,
                        env=os.environ,
                    ) as p:
                        for line in p.stdout:
                            if cancelled():
                                p.terminate()
                                log("Rsync cancelled.")
                                return ServiceResult(False, "Rsync cancelled by user.")
                            log(line.rstrip("\n"))
                        p.wait()
                        if p.returncode != 0:
                            err = p.stderr.read()
                            if err:
                                log(err)
                            failed.append(subdir)
                except FileNotFoundError:
                    return ServiceResult(False, "rsync (or sshpass) not found on this machine.")
            if failed:
                return ServiceResult(False, f"Rsync failed for: {', '.join(failed)}")
            return ServiceResult(True, f"Rsync completed — {len(tasks)} dir(s) synced.")

        self.page.start_worker(task)

    # ── Step 2: Container replace ──────────────────────────────────

    def do_container_replace(self) -> None:
        if not self._ensure_ssh():
            return

        subdirs = self._checked_subdirs()
        if not subdirs:
            self.page.append_log("[ERROR] 请先扫描并勾选要替换的子目录。")
            return

        container = self._target.container_combo.currentText().strip()
        container_path = self._target.container_path_input.text().strip()
        temp_dir = self._target.temp_dir_input.text().strip()

        if not container:
            self.page.append_log("[ERROR] 请填写容器名。")
            return

        self.page.append_log(f"容器替换: {container}")

        def task(log, cancelled):
            ssh = self.ssh

            # 1) stop
            log(f"\n--- docker stop {container} ---")
            r = ssh.exec(f"docker stop {shlex.quote(container)} 2>&1")
            log(r.stdout or "(no output)")
            if r.stderr:
                log(f"[stderr] {r.stderr}")
            if cancelled():
                return ServiceResult(False, "Cancelled.")
            if r.exit_code != 0:
                log(f"[WARN] docker stop exited {r.exit_code}, continuing…")

            # 2) remove old files
            for subdir in subdirs:
                if cancelled():
                    return ServiceResult(False, "Cancelled.")
                target_path = f"{container_path.rstrip('/')}/{subdir}"
                log(f"\n--- docker exec {container} rm -rf {target_path} ---")
                r = ssh.exec(f"docker exec {shlex.quote(container)} rm -rf {shlex.quote(target_path)} 2>&1")
                log(r.stdout or "(no output)")
                if r.stderr:
                    log(f"[stderr] {r.stderr}")

            # 3) copy new files
            for subdir in subdirs:
                if cancelled():
                    return ServiceResult(False, "Cancelled.")
                src = f"{temp_dir.rstrip('/')}/{subdir}/"
                dst = f"{shlex.quote(container)}:{container_path.rstrip('/')}/"
                log(f"\n--- docker cp {src} {dst} ---")
                r = ssh.exec(f"docker cp {src} {dst} 2>&1")
                log(r.stdout or "(no output)")
                if r.stderr:
                    log(f"[stderr] {r.stderr}")
                if r.exit_code != 0:
                    return ServiceResult(False, f"docker cp failed for {subdir} (exit {r.exit_code})")

            # 4) restart
            if cancelled():
                return ServiceResult(False, "Cancelled.")
            log(f"\n--- docker restart {container} ---")
            r = ssh.exec(f"docker restart {shlex.quote(container)} 2>&1")
            log(r.stdout or "(no output)")
            if r.stderr:
                log(f"[stderr] {r.stderr}")
            if r.exit_code != 0:
                return ServiceResult(False, f"docker restart failed (exit {r.exit_code})")

            return ServiceResult(True, "容器替换完成。")

        self.page.start_worker(task)

    # ── Step 3: View logs ──────────────────────────────────────────

    def do_view_logs(self) -> None:
        if not self._ensure_ssh():
            return

        container = self._target.container_combo.currentText().strip()
        if not container:
            self.page.append_log("[ERROR] 请填写容器名。")
            return

        self.page.append_log(f"docker logs --tail 200 {container}")

        def task(log, cancelled):
            r = self.ssh.exec(f"docker logs --tail 200 {shlex.quote(container)} 2>&1")
            if r.stdout:
                for line in r.stdout.splitlines():
                    log(line)
            if r.stderr:
                for line in r.stderr.splitlines():
                    log(f"[stderr] {line}")
            return ServiceResult(True, "日志输出完毕。")

        self.page.start_worker(task)

    # ── Stop / shutdown ────────────────────────────────────────────

    def stop(self) -> None:
        pass  # BasePage.stop_worker() handles worker cancellation

    def shutdown(self) -> None:
        pass  # BasePage handles worker cleanup

    # ── Container refresh ──────────────────────────────────────────

    def refresh_containers(self) -> None:
        if not self._ensure_ssh():
            return
        self.page.append_log("Refreshing container list (docker ps -a) …")

        def task(log, _cancelled):
            r = self.ssh.exec("docker ps -a --format '{{.Names}}' 2>/dev/null")
            if r.exit_code != 0:
                return ServiceResult(False, f"docker ps -a failed: {r.stderr}")
            names = [name for name in r.stdout.strip().splitlines() if name]
            combo = self._target.container_combo
            current = combo.currentText()
            combo.clear()
            for name in names:
                combo.addItem(name)
            if current:
                idx = combo.findText(current)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                else:
                    combo.setCurrentText(current)
            return ServiceResult(True, f"Found {len(names)} container(s).")

        self.page.start_worker(task)

    # ── helpers ────────────────────────────────────────────────────

    def _ensure_ssh(self) -> bool:
        if not self.ssh.is_configured:
            self.page.append_log("[ERROR] 请先通过 SSH 连接到机器人。")
            return False
        return True
