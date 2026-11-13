"""Exercise the real C++ example and public SDK against a loopback gRPC fixture."""
import subprocess
import select
import sys
import time
import unittest

EXAMPLE = SERVER = None


class TfExampleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not EXAMPLE or not SERVER:
            raise unittest.SkipTest("Run via CTest with the built TF binaries")

    def run_example(self, scenario, action, expected):
        server = subprocess.Popen([SERVER, scenario], stdout=subprocess.PIPE, text=True)
        try:
            self.assertTrue(select.select([server.stdout], [], [], 5)[0],
                            "TF fixture did not start within five seconds")
            address = server.stdout.readline().strip()
            self.assertTrue(address.startswith("127.0.0.1:"), address)
            started = time.monotonic()
            result = subprocess.run([EXAMPLE, address, action], text=True,
                                    capture_output=True, timeout=12)
            elapsed = time.monotonic() - started
            self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
            return result, elapsed
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()
            server.stdout.close()

    def test_static_snapshot_does_not_wait_for_second_sample(self):
        result, elapsed = self.run_example("success", "static", 0)
        self.assertIn("world -> base", result.stdout)
        self.assertNotIn("TIMEOUT", result.stderr)
        self.assertLess(elapsed, 2)

    def test_static_failures_return_nonzero(self):
        for scenario in ("no_data", "empty", "rpc_error", "sdk_error", "eof"):
            with self.subTest(scenario=scenario):
                result, _ = self.run_example(scenario, "static", 1)
                self.assertTrue(result.stderr)

    def test_dynamic_timeout_and_disconnect_return_nonzero(self):
        for scenario in ("no_data", "stalled", "disconnect"):
            with self.subTest(scenario=scenario):
                result, _ = self.run_example(scenario, "stream", 1)
                self.assertTrue(result.stderr)

    def test_tree_requires_both_streams(self):
        for scenario in ("static_error", "dynamic_error"):
            with self.subTest(scenario=scenario):
                self.run_example(scenario, "tree", 1)

    def test_tree_combines_static_snapshot_and_dynamic_window(self):
        result, elapsed = self.run_example("success", "tree", 0)
        self.assertIn("Total frames: 2", result.stdout)
        self.assertNotIn("failed", result.stderr)
        self.assertGreaterEqual(elapsed, 2.8)
        self.assertLess(elapsed, 5)


if __name__ == "__main__":
    EXAMPLE, SERVER = sys.argv[1:3]
    del sys.argv[1:3]
    unittest.main()
