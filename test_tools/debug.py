# Copyright (c) 2026 eric6227
# Released under the MIT License. See LICENSE file in the project root for full text.
import subprocess
import sys

subprocess.Popen(
    ["python", "./control_software/infinite_rc_controller.py"],
    creationflags=subprocess.CREATE_NEW_CONSOLE,
)
subprocess.Popen(
    ["python", "./test_tools/telemetry_monitor.py"],
    creationflags=subprocess.CREATE_NEW_CONSOLE,
)