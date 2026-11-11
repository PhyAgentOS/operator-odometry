import subprocess
import sys

from odometry.process import OwnedProcess


def test_stop_is_scoped_to_owned_process_group(tmp_path):
    unrelated = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True
    )
    owned = OwnedProcess()
    try:
        owned.start(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            cwd=tmp_path,
            log_path=tmp_path / "backend.log",
        )
        owned.stop()
        assert owned.process is None
        assert unrelated.poll() is None
        owned.stop()  # idempotent cleanup
    finally:
        owned.stop()
        unrelated.terminate()
        unrelated.wait(timeout=3)
