"""Generated blank video -> real MediaPipe child -> GUI PPM preview, no hardware."""
import argparse
import json
import multiprocessing as mp
from pathlib import Path
import queue
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk'))
from bhl_hand_sdk.camera_source import camera_worker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    args = parser.parse_args()
    import cv2
    import numpy as np
    context = mp.get_context('spawn')
    with tempfile.TemporaryDirectory(prefix='dexhand-preview-test-') as directory:
        video = Path(directory)/'blank.avi'
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
        assert writer.isOpened()
        for _ in range(24):
            writer.write(np.zeros((480, 640, 3), dtype=np.uint8))
        writer.release()
        output, preview, stop = context.Queue(maxsize=1), context.Queue(maxsize=1), context.Event()
        worker = context.Process(target=camera_worker, kwargs=dict(output=output, preview=preview,
            stop=stop, camera=str(video), model=str(args.model), headless=True, auto_arm=False))
        worker.start()
        frames = images = 0
        eof = False
        deadline = time.monotonic()+30
        try:
            while time.monotonic() < deadline:
                try:
                    message = output.get(timeout=.02)
                    if message['kind'] == 'frame':
                        frames += 1
                        assert message['valid'] is False and message['arm_at'] == 0
                    else:
                        eof = message['kind'] == 'error' and 'read failed' in message['message']
                except queue.Empty:
                    pass
                try:
                    image = preview.get_nowait()
                    assert image['ppm'].startswith(b'P6\n480 360\n255\n')
                    assert len(image['ppm']) == len(b'P6\n480 360\n255\n')+480*360*3
                    images += 1
                except queue.Empty:
                    pass
                if not worker.is_alive():
                    break
            worker.join(timeout=1)
            assert not worker.is_alive() and worker.exitcode == 0 and frames and images and eof
            print(json.dumps(dict(passed=True, observed_frames=frames, preview_images=images,
                camera_hardware_opened=False, can_opened=False, physical_human_hand_tested=False)))
        finally:
            stop.set()
            if worker.is_alive():
                worker.terminate()
                worker.join(timeout=2)
            for pipe in (output, preview):
                pipe.close()
                pipe.cancel_join_thread()


if __name__ == '__main__':
    main()
