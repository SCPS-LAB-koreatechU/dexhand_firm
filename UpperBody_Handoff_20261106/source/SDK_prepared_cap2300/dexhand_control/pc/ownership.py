"""Advisory per-CAN-node process ownership. Does not authenticate CAN peers."""
from pathlib import Path
import os
import re
import tempfile


class NodeLease:
    def __init__(self, channel, nodes):
        self.files = []
        owner = str(os.getuid()) if hasattr(os, "getuid") else os.environ.get("USERNAME", "user")
        root = Path(tempfile.gettempdir()) / ("bhl-can-" + re.sub(r"[^\w.-]", "_", owner))
        root.mkdir(mode=0o700, exist_ok=True)
        try:
            for node in sorted(set(nodes)):
                path = root / (re.sub(r"[^\w.-]", "_", str(channel)) + f"-{node}.lock")
                handle = path.open("a+b")
                try:
                    if os.name == "nt":
                        import msvcrt
                        if path.stat().st_size == 0:
                            handle.write(b"0"); handle.flush()
                        handle.seek(0)
                        msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
                    else:
                        import fcntl
                        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError as error:
                    handle.close()
                    raise RuntimeError(f"CAN {channel} node {node} already owned by another SDK process") from error
                self.files.append(handle)
        except Exception:
            self.close()
            raise

    def close(self):
        for handle in self.files:
            if os.name == "nt":
                import msvcrt
                handle.seek(0)
                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            handle.close()
        self.files = []
