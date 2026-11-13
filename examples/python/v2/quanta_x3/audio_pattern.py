"""Create an audible quanta_x3 test WAV locally; never connects or plays audio."""
import argparse
from array import array
import math
from pathlib import Path
import sys
import wave


def create_pattern(path):
    # Six low/high/gap groups (0.65 s/0.65 s/0.7 s), totaling 12 s so the
    # acceptance test can be stopped midway.
    rate = 16000
    samples = array("h")
    for _ in range(6):
        for frequency, seconds in ((440, .65), (880, .65), (0, .7)):
            count = round(seconds * rate)
            for index in range(count):
                fade = min(1, index/(rate*.02), (count-1-index)/(rate*.02))
                samples.append(round(8191 * fade * math.sin(2*math.pi*frequency*index/rate)))
    if sys.byteorder != "little": samples.byteswap()
    with Path(path).open("xb") as output, wave.open(output, "wb") as wav:
        wav.setparams((1, 2, rate, 0, "NONE", "not compressed"))
        wav.writeframes(samples.tobytes())
    print(f"Created {path}: 12 s, six low-high-gap groups, mono 16 kHz S16LE; not played")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New WAV path; existing files are never overwritten")
    args = parser.parse_args()
    create_pattern(args.output)
