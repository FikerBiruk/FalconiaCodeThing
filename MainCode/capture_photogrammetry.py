#!/usr/bin/env python3
"""Capture a fixed number of images using rpicam-still."""

import os
import subprocess
import time

# User settings
TOTAL_IMAGES = 60
INTERVAL_SECONDS = 3
OUTPUT_DIR = "captures"


def countdown(seconds: int) -> None:
    if seconds <= 0:
        return
    print(f"Next photo in {seconds}...", end="", flush=True)
    for remaining in range(seconds - 1, -1, -1):
        time.sleep(1)
        if remaining > 0:
            print(f"{remaining}...", end="", flush=True)
    print()


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for index in range(1, TOTAL_IMAGES + 1):
        countdown(INTERVAL_SECONDS)
        filename = f"frame_{index:04d}.jpg"
        filepath = os.path.join(OUTPUT_DIR, filename)

        try:
            subprocess.run(
                ["rpicam-still", "-o", filepath],
                check=True,
            )
        except subprocess.CalledProcessError as exc:
            print(f"Capture failed for {filename}: {exc}")
            return

        print(f"Saved {filename} ({index}/{TOTAL_IMAGES})")


if __name__ == "__main__":
    main()
