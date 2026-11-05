"""Offline workspace tests can import all sibling ament Python packages."""
import sys
from pathlib import Path

for package in Path(__file__).parent.glob("bhl_*_sdk"):
    sys.path.insert(0, str(package))
