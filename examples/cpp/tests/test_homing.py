"""Run the actual C++ homing example against a delayed loopback RPC."""
import re
import select
import subprocess
import sys
import unittest

BINARY = SERVER = None


class HomingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not BINARY or not SERVER:
            raise unittest.SkipTest("Run via CTest with the built C++ example")

    def run_homing(self, timeout=None):
        server = subprocess.Popen([SERVER], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.assertTrue(select.select([server.stdout], [], [], 5)[0], "fixture startup timeout")
            address = server.stdout.readline().strip()
            self.assertTrue(address.startswith("127.0.0.1:"), address)
            args = [] if timeout is None else ["--timeout", str(timeout)]
            result = subprocess.run([BINARY, address, "homing", *args],
                                    text=True, capture_output=True, timeout=10)
        finally:
            server.terminate()
            try:
                _, events = server.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                _, events = server.communicate()
        self.assertEqual(re.findall(r"WORK_MODE (\d+)", events), ["3", "0"],
                         events + result.stdout + result.stderr)
        budget_ms = int(re.search(r"HOMING_BUDGET_MS (\d+)", events)[1])
        expected_ms = (60 if timeout is None else timeout) * 1000
        self.assertTrue(expected_ms - 1000 < budget_ms <= expected_ms + 100, events)
        return result, events

    def test_default_waits_for_completion(self):
        result, events = self.run_homing()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("homing result: true", result.stdout)
        self.assertIn("HOMING_FINISHED", events)

    def test_configured_timeout_is_used(self):
        result, events = self.run_homing(1)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("homing result: false", result.stdout)
        self.assertIn("Deadline Exceeded", result.stderr)
        self.assertNotIn("HOMING_FINISHED", events)

    def test_invalid_timeout_fails_before_connecting(self):
        cases = [["--timeout"], ["--unknown", "5"], ["--timeout", "5", "extra"]]
        cases += [["--timeout", value] for value in ("0", "-1", "1.5", "5s", "nan", "", "99999999999999999999")]
        for args in cases:
            with self.subTest(args=args):
                result = subprocess.run([BINARY, "127.0.0.1:1", "homing", *args],
                                        text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn("positive integer", result.stderr)
                self.assertNotIn("connect begin", result.stdout + result.stderr)


if __name__ == "__main__":
    BINARY, SERVER = sys.argv[1:3]
    del sys.argv[1:3]
    unittest.main()
