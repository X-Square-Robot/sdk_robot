from __future__ import annotations

import logging
import os
import select
import shlex
import subprocess
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class SshResult:
    stdout: str = ""
    stderr: str = ""
    exit_code: int = -1


class SshClient:
    """Thin wrapper around `ssh` CLI — zero Python dependencies beyond stdlib."""

    def __init__(self) -> None:
        self._host: str = ""
        self._port: int = 22
        self._user: str = ""
        self._password: str = ""

    @property
    def host(self) -> str:
        return self._host

    def configure(self, host: str, port: int, user: str, password: str) -> None:
        self._host = host
        self._port = port
        self._user = user
        self._password = password

    @property
    def is_configured(self) -> bool:
        return bool(self._host and self._user)

    def test_connection(self) -> SshResult:
        return self.exec("echo ok")

    def exec(self, command: str, timeout_sec: int | None = None) -> SshResult:
        """Execute *command* on the remote host and return stdout/stderr/exit_code."""
        if not self.is_configured:
            return SshResult(stderr="SSH not configured", exit_code=-1)

        cmd = self._build_ssh_command(command)
        try:
            p = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
            return SshResult(
                stdout=p.stdout,
                stderr=p.stderr,
                exit_code=p.returncode,
            )
        except subprocess.TimeoutExpired:
            return SshResult(stderr="Command timed out", exit_code=-1)
        except FileNotFoundError:
            return SshResult(
                stderr="ssh (or sshpass) not found on this machine",
                exit_code=-1,
            )

    def exec_streaming(self, command: str):
        """Run *command* on the remote host and yield (line, is_stderr) tuples.

        This uses a persistent channel so long-running commands (e.g. ``docker logs -f``)
        can be read line-by-line.  The caller is responsible for terminating the process
        via the returned ``Popen`` object when done.
        """
        if not self.is_configured:
            yield ("SSH not configured", True)
            return

        cmd = self._build_ssh_command(command)
        env = os.environ.copy()
        env.pop("PYTHONUNBUFFERED", None)
        try:
            with subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env,
            ) as p:
                fds = {}
                if p.stdout:
                    fds[p.stdout.fileno()] = (p.stdout, False)
                if p.stderr:
                    fds[p.stderr.fileno()] = (p.stderr, True)

                while fds:
                    try:
                        ready, _, _ = select.select(list(fds), [], [], 0.5)
                    except (ValueError, OSError):
                        break
                    if not ready:
                        if p.poll() is not None:
                            break
                        continue
                    for fd in ready:
                        pipe, is_err = fds[fd]
                        line = pipe.readline()
                        if line:
                            yield (line.rstrip("\n"), is_err)
                        else:
                            del fds[fd]
        except FileNotFoundError:
            yield ("ssh (or sshpass) not found on this machine", True)

    # ── helpers ────────────────────────────────────────────────────

    def list_containers(self) -> list[str]:
        """Return list of Docker container names on the remote host."""
        r = self.exec("docker ps --format '{{.Names}}' 2>/dev/null")
        if r.exit_code != 0:
            logger.warning("docker ps failed: %s", r.stderr)
            return []
        return [name for name in r.stdout.strip().splitlines() if name]

    # ── internals ──────────────────────────────────────────────────

    def _build_ssh_command(self, remote_command: str) -> list[str]:
        """Build the local command list for ``ssh`` (or ``sshpass``)."""
        ssh_opts = [
            "-o", "StrictHostKeyChecking=no",
            "-o", "UserKnownHostsFile=/dev/null",
            "-o", f"ConnectTimeout=10",
            "-p", str(self._port),
        ]

        if self._password:
            # sshpass reads password from an env var to avoid leaking in `ps`
            base = [
                "sshpass", "-e",
                "ssh",
            ] + ssh_opts
            env = os.environ.copy()
            env["SSHPASS"] = self._password
            # Update current process env so subprocess picks it up
            os.environ["SSHPASS"] = self._password
        else:
            base = ["ssh"] + ssh_opts

        return base + [f"{self._user}@{self._host}", remote_command]
