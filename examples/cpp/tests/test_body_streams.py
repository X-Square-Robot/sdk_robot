"""Check the actual C++ examples through the public Client and loopback RPCs."""
import pathlib
import select
import subprocess
import sys
import unittest

SERVER = BINARY_DIR = None


def cases():
    for model in ("desktop", "quanta_x1", "quanta_x2"):
        for side in ("left", "right"):
            for mode in ("joint_pos", "end_pose"):
                yield model + "_arm_control", [side, "stream", mode], "end_pose:" if mode == "end_pose" else "joint_state:"
            yield model + "_gripper_control", ["stream", side], "position:" if model == "quanta_x2" else "joint_state:"
    yield "head_control", ["stream"], "joint_state:"
    yield "quanta_x1_lift_control", ["stream"], "joint_state:"
    for mode in ("joint_pos", "end_pose"):
        yield "quanta_x2_waist_control", ["stream", mode], "end_pose:" if mode == "end_pose" else "joint_state:"


class BodyStreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SERVER or not BINARY_DIR:
            raise unittest.SkipTest("Run via CTest with the built C++ examples")

    def run_example(self, scenario, case):
        executable, arguments, expected_data = case
        server = subprocess.Popen([SERVER, scenario], stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True)
        try:
            self.assertTrue(select.select([server.stdout], [], [], 5)[0], "fixture startup timeout")
            address = server.stdout.readline().strip()
            self.assertTrue(address.startswith("127.0.0.1:"), address)
            result = subprocess.run([str(pathlib.Path(BINARY_DIR) / executable), address, *arguments],
                                    text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("failed", result.stderr)
            if scenario != "open_error":
                self.assertIn(expected_data, result.stdout, "failure must occur after a valid first frame")
                self.assertIn("stream read", result.stderr)
        finally:
            server.terminate()
            try:
                _, errors = server.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                _, errors = server.communicate()
            self.assertNotIn("UNEXPECTED_WRITE", errors, "stream commands must remain read-only")

    def test_disconnect_and_sdk_error_after_data(self):
        for case in cases():
            for scenario in ("disconnect", "sdk_error"):
                with self.subTest(example=case[:2], scenario=scenario):
                    self.run_example(scenario, case)

    def test_timeout_after_data(self):
        # Cover each independent implementation/branch once, without repeating
        # the identical left/right template instantiations for the timeout wait.
        for case in cases():
            if "right" not in case[1]:
                with self.subTest(example=case[:2]):
                    self.run_example("stall", case)

    def test_open_failures_stay_errors(self):
        for case in cases():
            if "right" not in case[1]:
                with self.subTest(example=case[:2]):
                    self.run_example("open_error", case)


if __name__ == "__main__":
    SERVER, BINARY_DIR = sys.argv[1:3]
    del sys.argv[1:3]
    unittest.main()
