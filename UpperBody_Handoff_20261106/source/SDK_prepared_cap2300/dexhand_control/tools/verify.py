"""Offline profiles, SDK, C++ firmware and wire integration checks. Never opens CAN."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import ast
import re
import xml.etree.ElementTree as ET
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT.parent / "humanoid_sdk-main/ros2_ws/src"


def run(*command, **kwargs):
    subprocess.run([str(v) for v in command], check=True, **kwargs)


def main():
    for generator in ("generate_board_config.py", "prepare_firmware.py", "generate_sdk_config.py", "sync_release_docs.py"):
        run(sys.executable, "-X", "utf8", "-B", ROOT / "tools" / generator, "--check")
    sources = [*SDK.rglob("*.py"), *(ROOT / "tools").glob("*.py"), *(ROOT / "pc").glob("*.py")]
    for path in sources:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path), feature_version=(3, 10))
    for path in SDK.glob("*/package.xml"):
        package = ET.parse(path).getroot()
        if package.findtext("version") != "0.2.0":
            raise RuntimeError("Package version mismatch: " + str(path))
    project = ROOT.parent
    for path in (project / "README.md", project / "humanoid_sdk-main/README.md",
                 SDK / "bhl_upper_body_sdk/README.md", ROOT / "firmware/안내사항/UPPER_BODY_SDK_v020.md",
                 ROOT / "firmware/안내사항/HAND_TEST_QUICKSTART.md",
                 ROOT / "firmware/안내사항/HAND_GUI.md",
                 ROOT / "firmware/안내사항/CAN_WIRING_GPIO1_3.md",
                 ROOT / "firmware/안내사항/CURRENT_SETUP_20261104.md"):
        for link in re.findall(r"\[[^\]]*\]\(([^)]+)\)", path.read_text(encoding="utf-8")):
            if "://" in link or link.startswith("#"):
                continue
            target = unquote(link.split("#")[0])
            if not (path.parent / target).exists():
                raise RuntimeError(f"Broken current documentation link: {path}: {link}")
    print(f"PASS: {len(sources)} Python sources parse on Python 3.10 syntax, package XML and current documentation links")
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler:
        cxx = [compiler]
    elif importlib.util.find_spec("ziglang"):
        cxx = [sys.executable, "-m", "ziglang", "c++"]
    else:
        raise SystemExit("C++17 compiler required: install g++/clang++ or pip install ziglang")
    with tempfile.TemporaryDirectory(prefix="bhl-offline-verify-") as work:
        output = Path(work).resolve()
        if output.parent != Path(tempfile.gettempdir()).resolve():
            raise RuntimeError("unexpected verification workspace")
        suffix = ".exe" if os.name == "nt" else ""
        for name in ("receiver_test", "controller_test", "ack_test", "wire_left", "wire_right"):
            source = "wire_harness" if name.startswith("wire_") else name
            hand = "DexHandRight" if name.endswith("right") else "DexHandLeft"
            executable = output / (name + suffix)
            run(*cxx, "-std=c++17", "-O0", "-Wall", "-Wextra", "-Werror", "-I", ROOT / "firmware" / hand,
                ROOT / "tests" / (source + ".cpp"), "-o", executable)
            if not name.startswith("wire_"):
                run(executable)
        environment = dict(os.environ, BHL_WIRE_HARNESS_DIR=str(output), PYTHONUTF8="1")
        run(sys.executable, "-X", "utf8", "-B", "-m", "pytest", "-q", "-p", "no:cacheprovider",
            SDK / "bhl_arm_sdk/test", SDK / "bhl_hand_sdk/test", SDK / "bhl_bringup/test",
            SDK / "bhl_upper_body_sdk/test", ROOT / "tests",
            env=environment)
    print("PASS: offline verification complete; no hardware accessed")


if __name__ == "__main__":
    main()
