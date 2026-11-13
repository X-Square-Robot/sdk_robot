"""A single SIGINT must save the C++ audio example's PCM as a valid WAV."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import wave


BINARY = SERVER = None


class AudioInterruptTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not BINARY or not SERVER:
            raise unittest.SkipTest("Run via CTest with the built examples")

    def check_interrupt(self, scenario):
        server = subprocess.Popen([SERVER, scenario, "quanta_x3"],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.addCleanup(self.stop_process, server)
        self.assertTrue(select.select([server.stdout], [], [], 5)[0])
        address = server.stdout.readline().decode().strip()
        self.assertTrue(address.startswith("unix:"), address)
        self.addCleanup(Path(address[5:]).unlink, missing_ok=True)
        with tempfile.TemporaryDirectory(prefix="sdk-audio-interrupt-") as directory:
            output = Path(directory) / "recording.wav"
            client = subprocess.Popen([BINARY, address, "stream", str(output)],
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                received = bytearray()
                deadline = time.monotonic() + 5
                while b"Frames: 1, Size: 4 bytes" not in received:
                    remaining = deadline - time.monotonic()
                    self.assertGreater(remaining, 0, received.decode())
                    self.assertTrue(select.select([client.stdout], [], [], remaining)[0])
                    chunk = os.read(client.stdout.fileno(), 4096)
                    self.assertTrue(chunk, received.decode())
                    received.extend(chunk)
                # Exactly one Ctrl+C. A continuing stream must not require a
                # second signal; a paused stream may finish its 2 s Read first.
                if scenario == "stall":
                    time.sleep(0.1)
                client.send_signal(signal.SIGINT)
                stdout, stderr = client.communicate(timeout=4)
                self.assertEqual(client.returncode, 0, (received + stdout + stderr).decode())
                self.assertIn(b"Saved to:", stdout)
                with wave.open(str(output), "rb") as wav:
                    self.assertEqual((wav.getnchannels(), wav.getframerate(), wav.getsampwidth()),
                                     (1, 16000, 2))
                    pcm = wav.readframes(wav.getnframes())
                    self.assertGreaterEqual(len(pcm), 4)
                    self.assertEqual(len(pcm) % 4, 0)
                    self.assertEqual(pcm, b"\x01\x00\xff\x7f" * (len(pcm) // 4))
            finally:
                self.stop_process(client)

    @staticmethod
    def stop_process(process):
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)

    def test_ctrl_c_saves_while_frames_keep_arriving(self):
        self.check_interrupt("continuous")

    def test_ctrl_c_saves_after_a_paused_stream_read(self):
        self.check_interrupt("stall")


if __name__ == "__main__":
    BINARY, SERVER = sys.argv[1:3]
    del sys.argv[1:3]
    unittest.main()
