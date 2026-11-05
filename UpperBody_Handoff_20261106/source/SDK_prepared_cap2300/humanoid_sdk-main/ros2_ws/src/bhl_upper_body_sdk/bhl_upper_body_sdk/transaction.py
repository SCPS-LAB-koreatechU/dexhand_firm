"""Nonblocking prepare/commit coordinator and local endpoints.

Prepare never changes a motor target. Commit is best-effort distributed delivery,
not an atomic physical motion; failure after any commit requests HOLD everywhere.
All deadlines additionally use monotonic time so ROS clock freezes cannot extend TTL.
"""
from __future__ import annotations

import copy
import uuid

from .contract import validate, key


class Coordinator:
    def __init__(self, parts, emit, report, *, phase_timeout=0.08):
        self.parts = tuple(parts)
        if not self.parts or len(set(self.parts)) != len(self.parts):
            raise ValueError("unique nonempty parts required")
        self.emit, self.report = emit, report
        self.phase_timeout = phase_timeout
        self.epoch = uuid.uuid4().hex
        self.pending = None
        self.active = None
        self.owner = None
        self.last_sequence = -1
        self.fault = ""
        self.last_result = None

    def _report(self, status, command=None, **extra):
        command = command or (self.pending or {}).get("command") or self.active
        self.last_result = dict(status=status, epoch=self.epoch,
                                session=None if command is None else command["session"],
                                sequence=None if command is None else command["sequence"], **extra)
        self.report(self.last_result)

    def submit(self, command, wall, mono):
        validate(command, wall, self.parts)
        if self.fault:
            raise ValueError("coordinator HOLD latched: " + self.fault)
        if self.pending is not None:
            self._report("BUSY", command)
            return False
        if self.owner is not None and command["session"] != self.owner:
            raise ValueError("another session owns control; HOLD and explicitly clear first")
        if command["session"] == self.owner and command["sequence"] <= self.last_sequence:
            raise ValueError("duplicate or reordered sequence")
        self.owner, self.last_sequence = key(command)
        remaining = command["stamp_s"] + command["ttl_s"] - wall
        command = copy.deepcopy(command)
        self.pending = dict(command=command, phase="prepare", replies={},
                            expires=mono + min(remaining, command["ttl_s"]),
                            deadline=mono + min(remaining, self.phase_timeout))
        self.emit(dict(phase="prepare", epoch=self.epoch, command=command))
        return True

    def reply(self, reply, wall, mono):
        p = self.pending
        if not p or not isinstance(reply, dict):
            return
        command = p["command"]
        if (reply.get("epoch") != self.epoch or reply.get("session") != command["session"] or
                reply.get("sequence") != command["sequence"] or reply.get("part") not in self.parts or
                reply.get("phase") != p["phase"]):
            return
        if self.tick(wall, mono):
            return
        if reply.get("ok") is not True:
            self.stop("PART_REJECTED: " + str(reply.get("part")) + ": " + str(reply.get("reason", "")))
            return
        p["replies"][reply["part"]] = copy.deepcopy(reply)
        if len(p["replies"]) != len(self.parts):
            return
        if p["phase"] == "prepare":
            p["phase"], p["replies"] = "commit", {}
            p["deadline"] = min(p["expires"], mono + self.phase_timeout)
            self.emit(dict(phase="commit", epoch=self.epoch, command=command))
        else:
            self.active = command
            self.active_expires = p["expires"]
            self._report("ACCEPTED", command, parts=p["replies"], physical_reached=None)
            self.pending = None

    def tick(self, wall, mono):
        p = self.pending
        if p and (mono >= p["deadline"] or wall >= p["command"]["stamp_s"] + p["command"]["ttl_s"]):
            self.stop("TRANSACTION_TIMEOUT")
            return True
        if self.active and (mono >= self.active_expires or wall >= self.active["stamp_s"] + self.active["ttl_s"]):
            self.stop("COMMAND_EXPIRED")
            return True
        return False

    def stop(self, reason):
        self.fault = str(reason)
        self._report("HOLD", reason=self.fault)
        self.emit(dict(phase="hold", epoch=self.epoch, reason=self.fault))
        self.pending = self.active = None

    def clear(self):
        # Controllers must separately be resumed and reported healthy by the ROS wrapper.
        if self.pending or self.active:
            raise ValueError("HOLD before clearing")
        self.fault, self.owner, self.last_sequence = "", None, -1
        self.epoch = uuid.uuid4().hex


class Endpoint:
    def __init__(self, part, validate_local, apply, hold, report):
        self.part = part
        self.validate_local, self.apply, self.hold, self.report = validate_local, apply, hold, report
        self.prepared = None
        self.active = None
        self.owner = None
        self.last_sequence = -1
        self.last_receipt = None

    def receive(self, message, wall, mono):
        if not isinstance(message, dict):
            return
        phase = message.get("phase")
        if phase == "hold":
            self.prepared = self.active = None
            self.owner, self.last_sequence = None, -1
            self.hold("UPPER_BODY: " + str(message.get("reason", "HOLD")))
            return
        if phase not in ("prepare", "commit"):
            return
        command = message.get("command")
        receipt = dict(part=self.part, phase=phase, epoch=message.get("epoch"),
                       session=command.get("session") if isinstance(command, dict) else None,
                       sequence=command.get("sequence") if isinstance(command, dict) else None, ok=False)
        try:
            if not isinstance(command, dict) or not isinstance(message.get("epoch"), str):
                raise ValueError("invalid transaction")
            validate(command, wall, command.get("targets", {}))
            if self.part not in command["targets"]:
                raise ValueError("required local part missing")
            token = (message["epoch"], *key(command))
            if phase == "prepare":
                if self.owner is not None and token[:2] != self.owner:
                    raise ValueError("different coordinator/session; HOLD before takeover")
                if self.owner == token[:2] and command["sequence"] <= self.last_sequence:
                    raise ValueError("replayed sequence")
                self.validate_local(command["targets"][self.part])
                self.owner, self.last_sequence = token[:2], command["sequence"]
                self.prepared = dict(token=token, command=copy.deepcopy(command),
                                     expires=mono + min(command["ttl_s"], command["stamp_s"] + command["ttl_s"] - wall))
            else:
                p = self.prepared
                if not p or p["token"] != token or p["command"] != command or mono >= p["expires"]:
                    raise ValueError("commit does not match a fresh prepared command")
                self.validate_local(command["targets"][self.part])
                self.apply(command["targets"][self.part])
                self.active = p
                self.prepared = None
            receipt.update(ok=True, acceptance="goal_stored" if phase == "commit" else "validated",
                           physical_reached=None)
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            receipt["reason"] = str(error)
        self.last_receipt = receipt
        self.report(receipt)

    def tick(self, wall, mono):
        if self.prepared and mono >= self.prepared["expires"]:
            self.prepared = None
        if self.active:
            command = self.active["command"]
            if mono >= self.active["expires"] or wall >= command["stamp_s"] + command["ttl_s"]:
                self.active = self.prepared = None
                self.hold("UPPER_BODY_COMMAND_EXPIRED")

    def status(self):
        return dict(receipt=self.last_receipt,
                    active=None if self.active is None else dict(session=self.active["command"]["session"],
                                                               sequence=self.active["command"]["sequence"]),
                    physical_reached=None)
