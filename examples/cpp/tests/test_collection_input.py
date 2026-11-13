"""Exercise the real collector's terminal/pipe input against a loopback server."""

import errno
import json
import os
import pathlib
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest

BINARY = SERVER = None


class CollectorProcess:
    def __init__(self, address, output, *, pipe=False, args=()):
        self.master, slave = pty.openpty()
        try:
            self.process = subprocess.Popen(
                [BINARY, address, str(output), "desktop", *args],
                stdin=subprocess.PIPE if pipe else slave,
                stdout=slave, stderr=slave,
            )
        finally:
            os.close(slave)
        self.pending = b""
        self.transcript = b""

    def send(self, text):
        data = text.encode()
        if self.process.stdin is not None:
            self.process.stdin.write(data)
            self.process.stdin.flush()
        else:
            os.write(self.master, data)

    def wait_for(self, text, timeout=4):
        target = text.encode()
        deadline = time.monotonic() + timeout
        while target not in self.pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.master], [], [], remaining)[0]:
                raise AssertionError(f"Missing {text!r}:\n{self.transcript.decode(errors='replace')}")
            try:
                data = os.read(self.master, 65536)
            except OSError as error:
                if error.errno != errno.EIO:
                    raise
                data = b""
            if not data:
                raise AssertionError(f"Exited before {text!r}:\n{self.transcript.decode(errors='replace')}")
            self.pending += data
            self.transcript += data
        self.pending = self.pending.split(target, 1)[1]

    def finish(self, expected=0):
        actual = self.process.wait(timeout=6)
        if actual != expected:
            raise AssertionError(f"Exit {actual}, expected {expected}: {self.transcript!r}")

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=5)
        if self.process.stdin is not None:
            self.process.stdin.close()
        os.close(self.master)


class CollectionInputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not BINARY or not SERVER:
            raise unittest.SkipTest("Run via CTest with the built examples")
        cls.errors = tempfile.TemporaryFile()
        cls.server = subprocess.Popen(
            [SERVER, "collection"], stdout=subprocess.PIPE, stderr=cls.errors, text=True)
        cls.addClassCleanup(cls.stop_server)
        if not select.select([cls.server.stdout], [], [], 5)[0]:
            raise AssertionError("Loopback server did not start")
        cls.address = cls.server.stdout.readline().strip()
        if not cls.address.startswith("127.0.0.1:"):
            raise AssertionError(f"Invalid loopback address: {cls.address!r}")

    @classmethod
    def stop_server(cls):
        cls.server.terminate()
        try:
            cls.server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.server.kill()
            cls.server.wait(timeout=5)
        cls.server.stdout.close()
        cls.errors.seek(0)
        errors = cls.errors.read()
        cls.errors.close()
        if b"UNEXPECTED_WRITE" in errors:
            raise AssertionError(f"Collection must remain read-only: {errors!r}")

    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix="cpp-collection-input-")
        self.addCleanup(directory.cleanup)
        self.output = pathlib.Path(directory.name)

    def launch(self, **kwargs):
        process = CollectorProcess(self.address, self.output, **kwargs)
        self.addCleanup(process.close)
        return process

    def start_episode(self, process, task="terminal_task"):
        process.wait_for("Enter task name")
        process.send(task + "\n")
        process.wait_for("press Enter to start")
        process.send("\n")
        process.wait_for("Recording in progress")
        process.wait_for("records=")

    def check_episode(self, index=0, task="terminal_task", require_records=True):
        episode = self.output / f"episode_{index:04d}"
        info = json.loads((episode / "episode.json").read_text())
        self.assertEqual(info["task"], task)
        count = 0
        for stream in info["streams"]:
            data = (episode / f"stream_{stream['name']}.pb").read_bytes()
            offset = records = 0
            while offset < len(data):
                self.assertGreaterEqual(len(data) - offset, 12, "Truncated record header")
                _, size = struct.unpack_from("=QI", data, offset)
                offset += 12 + size
                self.assertLessEqual(offset, len(data), "Truncated protobuf record")
                records += 1
            self.assertEqual(records, stream["records"])
            count += records
        self.assertEqual(count, info["num_records"])
        if require_records:
            self.assertGreater(count, 0)

    def test_terminal_enter_stops_and_allows_another_episode(self):
        process = self.launch()
        for index in range(2):
            self.start_episode(process)
            process.send("\n")
            process.wait_for("Continue recording next episode?")
            self.check_episode(index)
            process.send("y\n" if index == 0 else "n\n")
        process.finish()

    def test_pipe_partial_line_keeps_statistics_live_until_enter(self):
        process = self.launch(pipe=True, args=("--once",))
        self.start_episode(process)
        process.send("stop after newline")
        process.wait_for("records=")
        self.assertIsNone(process.process.poll())
        process.send("\n")
        process.finish()
        self.check_episode()

    def test_prebuffered_pipe_lines_are_not_lost(self):
        process = self.launch(pipe=True)
        process.send("piped_task\n\n\nn\n")
        process.process.stdin.close()
        process.finish()
        self.check_episode(task="piped_task", require_records=False)

    def test_eof_during_recording_saves_episode(self):
        process = self.launch(pipe=True)
        self.start_episode(process)
        process.process.stdin.close()
        process.finish()
        self.check_episode()

    def test_eof_at_prompts_does_not_start_recording(self):
        for prepare in (False, True):
            with self.subTest(prepare=prepare):
                process = self.launch(pipe=True)
                process.wait_for("Enter task name")
                if prepare:
                    process.send("unused_task\n")
                    process.wait_for("press Enter to start")
                process.process.stdin.close()
                process.finish()
                self.assertEqual(list(self.output.iterdir()), [])

    def test_signals_during_recording_save_episode(self):
        for index, sig in enumerate((signal.SIGINT, signal.SIGTERM)):
            with self.subTest(signal=sig):
                process = self.launch()
                self.start_episode(process)
                process.process.send_signal(sig)
                process.finish(130)
                # Each invocation starts numbering at zero and avoids overwrite.
                manifests = list(self.output.glob("*/episode.json"))
                self.assertEqual(len(manifests), index + 1)
                self.assertTrue(all(json.loads(p.read_text())["num_records"] > 0 for p in manifests))

    def test_signal_at_prompt_exits_without_recording(self):
        process = self.launch()
        process.wait_for("Enter task name")
        process.process.send_signal(signal.SIGINT)
        process.finish(130)
        self.assertEqual(list(self.output.iterdir()), [])

    def test_seconds_does_not_wait_for_stdin(self):
        process = self.launch(pipe=True, args=("--seconds", "1"))
        process.finish()
        self.check_episode(task="cpp_noninteractive")

    def test_signal_interrupts_timed_recording(self):
        process = self.launch(pipe=True, args=("--seconds", "60"))
        process.wait_for("Recording in progress")
        process.wait_for("records=")
        process.process.send_signal(signal.SIGINT)
        process.finish(130)
        self.check_episode(task="cpp_noninteractive")


if __name__ == "__main__":
    BINARY, SERVER = sys.argv[1:3]
    del sys.argv[1:3]
    unittest.main()
