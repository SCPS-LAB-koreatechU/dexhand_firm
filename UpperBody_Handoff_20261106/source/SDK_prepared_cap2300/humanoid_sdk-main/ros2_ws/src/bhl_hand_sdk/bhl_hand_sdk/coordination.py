"""Freshness/fault checks for required upper-body peers, independent of ROS."""
import time


class PeerMonitor:
    def __init__(self, names=(), dry_run=True, timeout=1.5):
        self.names, self.dry_run, self.timeout = tuple(names), dry_run, timeout
        self.seen = {}

    def update(self, name, status, now=None):
        if name not in self.names or not isinstance(status, dict):
            return False
        if type(status.get("dry_run")) is not bool or not ("joints" in status or "hand" in status):
            return False
        if "joints" in status and not isinstance(status["joints"], dict):
            return False
        reason = ""
        if bool(status.get("dry_run")) != self.dry_run:
            reason = "dry_run mismatch"
        elif status.get("estop") or status.get("motion_stopped") or status.get("fault"):
            reason = "peer stopped/faulted"
        elif "hand" in status and status.get("present") is False:
            reason = "hand hardware not responding"
        elif any(j.get("fault") for j in status.get("joints", {}).values() if isinstance(j, dict)):
            reason = "peer joint fault"
        self.seen[name] = (time.monotonic() if now is None else now, reason)
        return True

    def issue(self, require_all=True, now=None):
        now = time.monotonic() if now is None else now
        for name in self.names:
            if name not in self.seen:
                if require_all:
                    return name + ": no status"
                continue
            stamp, reason = self.seen[name]
            if now - stamp > self.timeout:
                return name + ": status expired"
            if reason:
                return name + ": " + reason
        return ""
