"""Actual example binaries against large/malformed payloads and stream errors."""
import pathlib
import os
import select
import signal
import subprocess
import sys
import time
import unittest
SERVER = BINARY_DIR = None

class SensorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SERVER or not BINARY_DIR:
            raise unittest.SkipTest("Run via CTest with the built examples")

    def run_example(self, scenario, binary, action, expected=1, *, model=None, seconds=None, interrupt=False):
        lidar = binary == "quanta_x3_lidar_points"
        model = model or ("quanta_x3" if lidar else "quanta_x1")
        server = subprocess.Popen([SERVER, scenario, model], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        address = None
        client = None
        try:
            self.assertTrue(select.select([server.stdout], [], [], 5)[0])
            address = server.stdout.readline().strip()
            args = (["--server", address, "--action", action] if lidar else [address, action])
            if seconds is not None:
                args += ["--seconds", str(seconds)]
            command = [str(pathlib.Path(BINARY_DIR) / binary), *args]
            if interrupt:
                client = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                prefix = bytearray()
                deadline = time.monotonic() + 5
                while b"Total Points:" not in prefix:
                    remaining = deadline - time.monotonic()
                    self.assertGreater(remaining, 0, prefix.decode())
                    self.assertTrue(select.select([client.stdout], [], [], remaining)[0])
                    chunk = os.read(client.stdout.fileno(), 4096)
                    self.assertTrue(chunk, prefix.decode())
                    prefix.extend(chunk)
                client.send_signal(signal.SIGINT)
                stdout, stderr = client.communicate(timeout=3)
                result = subprocess.CompletedProcess(command, client.returncode, prefix.decode() + stdout, stderr)
            else:
                result = subprocess.run(command, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
            if expected:
                self.assertIn("failed", result.stderr)
            return result
        finally:
            if client is not None and client.poll() is None:
                client.kill()
                client.communicate()
            server.terminate()
            try:
                _, errors = server.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill(); _, errors = server.communicate()
            if address and address.startswith("unix:"):
                pathlib.Path(address[5:]).unlink(missing_ok=True)
            self.assertNotIn("UNEXPECTED_WRITE", errors)
            if lidar:
                self.assertNotIn("CHASSIS_READ", errors)
                self.assertEqual(errors.count("LIDAR_READ"), 0 if model != "quanta_x3" else 2 if action == "verify" else 1)

    def test_stream_failures(self):
        for binary, action in [("imu", "stream"), ("radar", "stream"), ("depth_points", "stream"),
                               ("tof", "sensor-1-stream"), ("tof", "both-stream"),
                               ("ultrasonic", "sensor-1-stream"), ("ultrasonic", "all-stream")]:
            for scenario in ("open_error", "disconnect", "sdk_error", "stall"):
                with self.subTest(binary=binary, action=action, scenario=scenario):
                    self.run_example(scenario, binary, action)

    def test_radar_withdrawal_is_model_and_error_specific(self):
        for action in ("single", "stream"):
            result = self.run_example("x3_withdrawn", "radar", action, 0)
            self.assertIn("Quanta X3; skipped", result.stdout)
            for scenario in ("withdrawn", "x3_unavailable", "x3_success"):
                with self.subTest(action=action, scenario=scenario):
                    self.run_example(scenario, "radar", action)

    def test_tof_withdrawal_is_model_and_error_specific(self):
        for action in ("sensor-1", "sensor-2", "sensor-1-stream", "sensor-2-stream",
                       "both", "both-stream"):
            result = self.run_example("x3_withdrawn", "tof", action, 0)
            self.assertIn("Quanta X3; skipped", result.stdout)
            for scenario in ("withdrawn", "x3_unavailable", "x3_success"):
                with self.subTest(action=action, scenario=scenario):
                    self.run_example(scenario, "tof", action)

    def test_padded_rows_and_endianness(self):
        for scenario in ("padded", "big_endian"):
            out = self.run_example(scenario, "depth_points", "single", 0)
            self.assertIn("Point 2: (-2.25, -2.5, -3.75)", out.stdout)

    def test_compacted_single_row_with_stale_driver_stride(self):
        for scenario in ("packed_stale_stride", "packed_zero_stride"):
            out = self.run_example(scenario, "depth_points", "single", 0)
            self.assertIn("Total Points: 2", out.stdout)
            self.assertIn("Point 2: (-2.25, -2.5, -3.75)", out.stdout)
            self.assertIn("ignoring stale row_step=", out.stderr)

    def test_ordinary_robot_large_cloud(self):
        # FLOAT32 bytes use repeated uint32 on the wire, exceeding 4 MiB.
        out = self.run_example("large", "depth_points", "single", 0)
        self.assertIn("Total Points: 307200", out.stdout)

    def test_malformed_cloud_is_failure(self):
        for scenario in ("missing_xyz", "truncated", "bad_stride", "non_byte", "bad_field", "packed_truncated"):
            for action in ("single", "stream"):
                with self.subTest(scenario=scenario, action=action):
                    self.run_example(scenario, "depth_points", action)

    def test_lidar_reads_and_verify_use_only_lidar_rpc(self):
        for action in ("single", "stream", "verify"):
            with self.subTest(action=action):
                out = self.run_example("continuous", "quanta_x3_lidar_points", action, 0, seconds=0.1)
                self.assertIn("Total Points: 2", out.stdout)

    def test_lidar_cloud_layouts_and_large_payload(self):
        for scenario in ("padded", "big_endian", "packed_stale_stride", "packed_zero_stride", "large"):
            with self.subTest(scenario=scenario):
                out = self.run_example(scenario, "quanta_x3_lidar_points", "single", 0)
                self.assertIn("Total Points: " + ("307200" if scenario == "large" else "2"), out.stdout)
                if scenario != "large":
                    self.assertIn("Point 2: (-2.25, -2.5, -3.75)", out.stdout)

    def test_lidar_rejects_malformed_empty_and_failed_streams(self):
        for scenario in ("missing_xyz", "truncated", "bad_stride", "non_byte", "bad_field", "packed_truncated", "empty_stream", "open_error"):
            for action in ("single", "stream"):
                with self.subTest(scenario=scenario, action=action):
                    self.run_example(scenario, "quanta_x3_lidar_points", action)
        for scenario in ("disconnect", "sdk_error"):
            with self.subTest(scenario=scenario):
                self.run_example(scenario, "quanta_x3_lidar_points", "stream")

    def test_lidar_rejects_other_models_before_sensor_read(self):
        for action in ("single", "stream", "verify"):
            self.run_example("continuous", "quanta_x3_lidar_points", action, model="quanta_x1")

    def test_lidar_ctrl_c_cancels_an_unbounded_stream(self):
        out = self.run_example("continuous", "quanta_x3_lidar_points", "stream", 0, interrupt=True)
        self.assertIn("Stopped by user.", out.stdout)

    def test_lidar_duration_stops_even_when_publishing_pauses(self):
        self.run_example("stall", "quanta_x3_lidar_points", "stream", 0, seconds=0.1)

    def test_lidar_arguments_fail_before_connection(self):
        for args in (("--action", "invalid"), ("--seconds", "nan"), ("--seconds", "1s"), ("--server",), ("--source", "chassis")):
            with self.subTest(args=args):
                out = subprocess.run([str(pathlib.Path(BINARY_DIR) / "quanta_x3_lidar_points"), *args], capture_output=True, text=True)
                self.assertEqual(out.returncode, 2)
                self.assertNotIn("connect begin", out.stdout + out.stderr)

if __name__ == "__main__":
    SERVER, BINARY_DIR = sys.argv[1:3]
    del sys.argv[1:3]
    unittest.main()
