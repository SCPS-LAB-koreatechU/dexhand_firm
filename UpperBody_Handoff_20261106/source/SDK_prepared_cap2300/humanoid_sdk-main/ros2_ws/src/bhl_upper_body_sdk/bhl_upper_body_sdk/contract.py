"""ROS-independent wire contract. Angles are radians; hand values are tendon pull fractions."""
from __future__ import annotations

import json
import math
import uuid

SCHEMA = "bhl.upper_body/1"
PARTS = {"arms", "hand_left", "hand_right"}
MAX_BYTES = 16384


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def decode(text, max_bytes=MAX_BYTES):
    if not isinstance(text, str) or len(text.encode("utf-8")) > max_bytes:
        raise ValueError("message exceeds size limit")
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate JSON key")
            result[key] = value
        return result
    value = json.loads(text, object_pairs_hook=unique)
    if not isinstance(value, dict):
        raise ValueError("object required")
    return value


def validate(command, now, required):
    if not isinstance(command, dict) or set(command) != {"schema", "session", "sequence", "stamp_s", "ttl_s", "targets"}:
        raise ValueError("invalid command fields")
    if command["schema"] != SCHEMA:
        raise ValueError("unsupported schema")
    session = command["session"]
    if not isinstance(session, str) or len(session) != 32 or uuid.UUID(hex=session).hex != session:
        raise ValueError("session must be lowercase UUID hex")
    if type(command["sequence"]) is not int or not 0 <= command["sequence"] < 2**63:
        raise ValueError("sequence must be an integer in 0..2^63-1")
    stamp, ttl = command["stamp_s"], command["ttl_s"]
    if not number(stamp) or not number(ttl) or not 0.05 <= ttl <= 0.5:
        raise ValueError("finite timestamp and 0.05..0.5 second TTL required")
    if stamp - now > 0.02 or now >= stamp + ttl:
        raise ValueError("expired command or clock skew >20ms")
    targets = command["targets"]
    if not isinstance(targets, dict) or set(targets) != set(required) or not set(required) <= PARTS:
        raise ValueError("targets must include exactly the configured parts")
    for part, values in targets.items():
        if part == "arms":
            if not isinstance(values, dict) or not values or len(values) > 10:
                raise ValueError("arms must be a joint-name/radian mapping")
            if any(not isinstance(k, str) or len(k) > 128 or not number(v) for k, v in values.items()):
                raise ValueError("invalid arm joint/value")
        else:
            if not isinstance(values, list) or len(values) != 15 or any(not number(v) or not 0 <= v <= 1 for v in values):
                raise ValueError("each hand requires 15 pull fractions in 0..1")
            a, b = (12, 14) if part == "hand_right" else (12, 13)
            if values[a] + values[b] > 1 + 1e-9:
                raise ValueError("opposing thumb pull sum exceeds 1")
    return command


def key(command):
    return command["session"], command["sequence"]


class CommandBuilder:
    def __init__(self):
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def build(self, *, stamp_s, arms_rad=None, hand_left=None, hand_right=None, ttl_s=0.25):
        targets = {}
        if arms_rad is not None:
            targets["arms"] = dict(arms_rad)
        for name, values in (("hand_left", hand_left), ("hand_right", hand_right)):
            if values is not None:
                targets[name] = list(values)
        result = dict(schema=SCHEMA, session=self.session, sequence=self.sequence,
                      stamp_s=stamp_s, ttl_s=ttl_s, targets=targets)
        validate(result, stamp_s, targets)
        self.sequence += 1
        return result

