import copy
import math

import pytest

from bhl_upper_body_sdk.contract import CommandBuilder, decode, validate
from bhl_upper_body_sdk.transaction import Coordinator, Endpoint


def command():
    return CommandBuilder().build(stamp_s=100, arms_rad={"a": .1}, hand_left=[0.0]*15,
                                  hand_right=[0.0]*15)


def harness():
    messages, reports, applied, held, endpoints = [], [], [], [], {}
    core = Coordinator(("arms", "hand_left", "hand_right"), messages.append, reports.append)
    for part in core.parts:
        endpoints[part] = Endpoint(part, lambda values: None, lambda v, p=part: applied.append((p, v)),
                                    lambda r, p=part: held.append((p, r)),
                                    lambda r: core.reply(r, 100.01, 1.01))
    return core, messages, reports, applied, held, endpoints


def test_no_target_change_until_every_prepare_and_full_commit_receipts():
    core, queue, reports, applied, held, endpoints = harness()
    cmd = command()
    assert core.submit(cmd, 100, 1)
    prepare = queue.pop(0)
    endpoints["arms"].receive(prepare, 100, 1)
    endpoints["hand_left"].receive(prepare, 100, 1)
    assert not applied and not queue
    endpoints["hand_right"].receive(prepare, 100, 1)
    assert not applied and queue[0]["phase"] == "commit"
    commit = queue.pop()
    for endpoint in endpoints.values():
        endpoint.receive(commit, 100.01, 1.01)
    assert len(applied) == 3 and not held
    assert reports[-1]["status"] == "ACCEPTED" and reports[-1]["physical_reached"] is None
    assert set(reports[-1]["parts"]) == set(endpoints)


def test_one_prepare_rejection_never_changes_any_target_and_holds_all():
    core, queue, reports, applied, held, endpoints = harness()
    def invalid(values):
        raise ValueError("bad calibration")
    endpoints["hand_right"].validate_local = invalid
    core.submit(command(), 100, 1)
    prepare = queue.pop()
    for endpoint in endpoints.values():
        endpoint.receive(prepare, 100, 1)
    assert core.fault and not applied and queue[-1]["phase"] == "hold"
    for endpoint in endpoints.values():
        endpoint.receive(queue[-1], 100, 1)
    assert len(held) == 3


def test_commit_loss_stops_remaining_parts_and_no_false_all_accepted():
    core, queue, reports, applied, held, endpoints = harness()
    core.submit(command(), 100, 1)
    prepare = queue.pop()
    for endpoint in endpoints.values():
        endpoint.receive(prepare, 100, 1)
    commit = queue.pop()
    endpoints["arms"].receive(commit, 100.01, 1.01)
    core.tick(100.1, 1.1)
    assert core.fault == "TRANSACTION_TIMEOUT" and len(applied) == 1
    assert not any(r["status"] == "ACCEPTED" for r in reports)
    assert queue[-1]["phase"] == "hold"


def test_commit_must_equal_prepared_target_no_mutation_or_replay():
    results, applied = [], []
    endpoint = Endpoint("arms", lambda x: None, applied.append, lambda r: None, results.append)
    prepare = dict(phase="prepare", epoch="e", command=command())
    endpoint.receive(prepare, 100, 1)
    modified = copy.deepcopy(prepare)
    modified["phase"] = "commit"
    modified["command"]["targets"]["arms"]["a"] = .9
    endpoint.receive(modified, 100.01, 1.01)
    assert not results[-1]["ok"] and not applied
    commit = {**prepare, "phase": "commit"}
    endpoint.receive(commit, 100.01, 1.01)
    endpoint.receive(commit, 100.01, 1.01)
    assert len(applied) == 1 and not results[-1]["ok"]
    endpoint.receive(prepare, 100.01, 1.01)
    assert not results[-1]["ok"]


def test_deadline_local_even_without_coordinator_and_frozen_ros_clock():
    held = []
    endpoint = Endpoint("arms", lambda x: None, lambda x: None, held.append, lambda x: None)
    prepare = dict(phase="prepare", epoch="e", command=command())
    endpoint.receive(prepare, 100, 1)
    endpoint.receive({**prepare, "phase": "commit"}, 100, 1)
    endpoint.tick(100, 1.26)
    assert held == ["UPPER_BODY_COMMAND_EXPIRED"]


def test_old_epoch_or_wrong_phase_receipt_cannot_complete_transaction():
    core, queue, reports, *_ = harness()
    cmd = command()
    core.submit(cmd, 100, 1)
    for phase, epoch in (("commit", core.epoch), ("prepare", "old")):
        for part in core.parts:
            core.reply(dict(phase=phase, epoch=epoch, part=part, session=cmd["session"],
                            sequence=cmd["sequence"], ok=True), 100.01, 1.01)
    assert len(queue) == 1 and core.pending["phase"] == "prepare" and not reports


def test_busy_and_different_owner_cannot_replace_pending_or_active():
    core, queue, reports, *_ = harness()
    cmd = command()
    core.submit(cmd, 100, 1)
    assert not core.submit(command(), 100, 1)
    assert reports[-1]["status"] == "BUSY" and core.pending["command"] == cmd
    core.stop("operator")
    with pytest.raises(ValueError, match="latched"):
        core.submit(cmd, 100, 1)
    epoch = core.epoch
    core.clear()
    assert core.epoch != epoch


@pytest.mark.parametrize("change", [
    {"ttl_s": float("nan")}, {"ttl_s": True}, {"ttl_s": .001}, {"ttl_s": 10},
    {"stamp_s": 99}, {"stamp_s": 101}, {"session": "wrong"}, {"sequence": -1}, {"sequence": True},
    {"sequence": 2**63}, {"schema": "wrong"}, {"extra": 1}, {"targets": {}},
])
def test_invalid_envelopes_never_submit(change):
    cmd = {**command(), **change}
    with pytest.raises(ValueError):
        validate(cmd, 100, ("arms", "hand_left", "hand_right"))


def test_bad_targets_and_duplicate_json_keys():
    for values in ([0]*14, [math.nan]*15, [True]*15, [1]*15):
        cmd = command()
        cmd["targets"]["hand_left"] = values
        with pytest.raises(ValueError):
            validate(cmd, 100, cmd["targets"])
    with pytest.raises(ValueError, match="duplicate"):
        decode('{"sequence":1,"sequence":2}')
    with pytest.raises(ValueError):
        decode(" " * 16385)


def test_expired_prepare_and_takeover_rejected_locally():
    applied, results = [], []
    endpoint = Endpoint("arms", lambda x: None, applied.append, lambda r: None, results.append)
    first = dict(phase="prepare", epoch="one", command=command())
    endpoint.receive(first, 100, 1)
    endpoint.receive(dict(phase="prepare", epoch="two", command=command()), 100, 1)
    assert not results[-1]["ok"]
    endpoint.receive({**first, "phase": "commit"}, 100, 1.26)
    assert not applied and not results[-1]["ok"]
