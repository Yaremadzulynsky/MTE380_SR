#!/usr/bin/env python3
"""Install or uninstall the mte380-button-controller systemd service.

The service runs button_stack_controller.py on boot. That script:
- GO button: verifies camera is detected, then starts docker-compose in this
  directory and run_perception_rpicam.sh, with error logging.
- STOP button: stops the perception script, then stops the docker containers.

Usage:
  sudo python3 install_button_controller_service.py install   # install and enable
  sudo python3 install_button_controller_service.py uninstall
  sudo python3 install_button_controller_service.py status
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

SERVICE_NAME = "mte380-button-controller.service"
SYSTEMD_DIR = Path("/etc/systemd/system")
SCRIPT_DIR = Path(__file__).resolve().parent
SERVICE_FILE = SCRIPT_DIR / SERVICE_NAME


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(args, capture_output=True, text=True)
    if check and result.returncode != 0:
        print(result.stderr or result.stdout, file=sys.stderr)
        raise SystemExit(result.returncode)
    return result


def install() -> None:
    if not SERVICE_FILE.exists():
        raise SystemExit(f"Service file not found: {SERVICE_FILE}")
    dest = SYSTEMD_DIR / SERVICE_NAME
    shutil.copy2(SERVICE_FILE, dest)
    print(f"Installed {dest}")
    run("systemctl", "daemon-reload")
    run("systemctl", "enable", SERVICE_NAME)
    run("systemctl", "start", SERVICE_NAME)
    print("Service enabled and started.")


def uninstall() -> None:
    run("systemctl", "stop", SERVICE_NAME, check=False)
    run("systemctl", "disable", SERVICE_NAME, check=False)
    dest = SYSTEMD_DIR / SERVICE_NAME
    if dest.exists():
        dest.unlink()
        print(f"Removed {dest}")
    run("systemctl", "daemon-reload")
    print("Service uninstalled.")


def status() -> None:
    subprocess.run(["systemctl", "status", SERVICE_NAME])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument(
        "action",
        choices=["install", "uninstall", "status"],
        help="install (enable + start), uninstall, or show status",
    )
    args = parser.parse_args()
    if args.action == "install":
        install()
    elif args.action == "uninstall":
        uninstall()
    else:
        status()


if __name__ == "__main__":
    main()
