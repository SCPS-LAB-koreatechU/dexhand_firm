"""Optional session storage outside the SD card; never changes HOME."""
import os
from pathlib import Path


def data_path(kind):
    override=os.environ.get('BHL_HAND_DATA_DIR')
    if override:
        root=Path(override).expanduser().resolve()
        return root / {'logs':'logs','calibration':'calibration','preview':'preview.json'}[kind]
    return {
        'logs':Path.home()/'bhl_logs/hand_gui',
        'calibration':Path.home()/'bhl_calibration/hand_camera',
        'preview':Path.home()/'.local/state/bhl/camera_retarget_preview.json',
    }[kind]
