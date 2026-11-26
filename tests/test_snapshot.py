import json
import threading

import pytest

from odometry import snapshot


def test_blocked_disk_keeps_submission_nonblocking_and_coalesces_latest(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = snapshot._write_payload
    written = []

    def blocked(folder, name, payload):
        written.append(json.loads(payload))
        if len(written) == 1:
            entered.set()
            assert release.wait(5)
        original(folder, name, payload)

    monkeypatch.setattr(snapshot, "_write_payload", blocked)
    clock = iter([100., 101., 102.])
    monkeypatch.setattr(snapshot, "monotonic", lambda: next(clock))
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("odometry", "scan"))
    try:
        assert writer.submit("odometry", {"sample": 1})
        assert entered.wait(2)
        # A single stalled write cannot stall these sensor callbacks. A stale
        # queued observation is replaced, rather than replayed after recovery.
        assert writer.submit("odometry", {"sample": 2})
        assert writer.submit("odometry", {"sample": 3})
        assert not (tmp_path / "odometry.json").exists()
    finally:
        release.set()
        writer.close()
    assert [x["sample"] for x in written] == [1, 3]
    assert json.loads((tmp_path / "odometry.json").read_text()) == {
        "sample": 3, "issued_monotonic": 102.0
    }
    assert not list(tmp_path.glob("*.tmp"))


def test_disk_failure_keeps_previous_observation_and_reports_failure(tmp_path, monkeypatch, capsys):
    snapshot.write_snapshot(tmp_path, "scan", {"sample": "previous"})
    previous = (tmp_path / "scan.json").read_bytes()

    def fail(*args):
        raise OSError("disk unavailable")

    monkeypatch.setattr(snapshot, "_write_payload", fail)
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("scan",))
    assert writer.submit("scan", {"sample": "new"})
    writer.close()
    assert writer.error == "OSError"
    assert not writer.submit("scan", {"sample": "later"})
    assert (tmp_path / "scan.json").read_bytes() == previous
    assert "Diagnostic snapshots stopped: OSError" in capsys.readouterr().err


def test_both_names_advance_when_pending_updates_are_coalesced(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = snapshot._write_payload
    written = []

    def blocked(folder, name, payload):
        written.append((name, json.loads(payload)["sample"]))
        if len(written) == 1:
            entered.set()
            assert release.wait(5)
        original(folder, name, payload)

    monkeypatch.setattr(snapshot, "_write_payload", blocked)
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("odometry", "scan"))
    try:
        writer.submit("odometry", {"sample": 0})
        assert entered.wait(2)
        writer.submit("scan", {"sample": 1})
        for i in range(1, 100):
            writer.submit("odometry", {"sample": i})
    finally:
        release.set()
        writer.close()
    assert written == [("odometry", 0), ("scan", 1), ("odometry", 99)]


def test_close_is_bounded_with_a_stalled_filesystem(tmp_path, monkeypatch, capsys):
    entered, release = threading.Event(), threading.Event()

    def blocked(*args):
        entered.set()
        assert release.wait(5)

    monkeypatch.setattr(snapshot, "_write_payload", blocked)
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("scan",))
    try:
        writer.submit("scan", {})
        assert entered.wait(2)
        writer.close(timeout=.01)
        assert writer.thread.is_alive()
        assert "shutdown timed out" in capsys.readouterr().err
        assert not writer.submit("scan", {})
    finally:
        release.set()
        writer.close()


def test_invalid_snapshot_preserves_previous_and_names_are_bounded(tmp_path):
    snapshot.write_snapshot(tmp_path, "scan", {"valid": True})
    previous = (tmp_path / "scan.json").read_bytes()
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("scan",))
    try:
        with pytest.raises(ValueError):
            writer.submit("unknown", {})
        with pytest.raises(ValueError):
            writer.submit("scan", {"range": float("nan")})
    finally:
        writer.close()
    assert (tmp_path / "scan.json").read_bytes() == previous


def test_unconfigured_snapshots_start_no_worker():
    writer = snapshot.LatestSnapshotWriter(None, ("scan",))
    assert writer.thread is None
    assert not writer.submit("scan", {})
    writer.close()
