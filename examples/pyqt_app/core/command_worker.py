from __future__ import annotations

import time

from PyQt6.QtCore import QThread, pyqtSignal

from .ssh_client import SshClient


class SshCommandWorker(QThread):
    """Run a single SSH command in the background, streaming output line-by-line.

    For one-shot commands the worker calls ``SshClient.exec`` and emits the
    complete output on ``output``, then ``finished``.

    For streaming (long-running) commands the worker calls
    ``SshClient.exec_streaming`` and lines arrive as they are read.  Call
    ``request_stop()`` to terminate.
    """

    output = pyqtSignal(str)       # one line of stdout
    output_err = pyqtSignal(str)   # one line of stderr
    finished = pyqtSignal(int)     # exit_code
    error = pyqtSignal(str)        # connection-level error

    def __init__(self, ssh: SshClient, command: str, streaming: bool = False):
        super().__init__()
        self._ssh = ssh
        self._command = command
        self._streaming = streaming

    def run(self) -> None:
        if not self._ssh.is_configured:
            self.error.emit("SSH not configured — cannot execute command.")
            self.finished.emit(-1)
            return

        if self._streaming:
            self._run_streaming()
        else:
            self._run_oneshot()

    def request_stop(self) -> None:
        self.requestInterruption()

    # ── internals ──────────────────────────────────────────────────

    def _run_oneshot(self) -> None:
        try:
            result = self._ssh.exec(self._command)
            if result.stdout:
                for line in result.stdout.splitlines():
                    if self.isInterruptionRequested():
                        break
                    self.output.emit(line)
            if result.stderr:
                for line in result.stderr.splitlines():
                    if self.isInterruptionRequested():
                        break
                    self.output_err.emit(line)
            self.finished.emit(result.exit_code)
        except Exception as exc:
            self.error.emit(str(exc))
            self.finished.emit(-1)

    def _run_streaming(self) -> None:
        try:
            for line, is_err in self._ssh.exec_streaming(self._command):
                if self.isInterruptionRequested():
                    break
                if is_err:
                    self.output_err.emit(line)
                else:
                    self.output.emit(line)
            self.finished.emit(0)
        except Exception as exc:
            self.error.emit(str(exc))
            self.finished.emit(-1)
