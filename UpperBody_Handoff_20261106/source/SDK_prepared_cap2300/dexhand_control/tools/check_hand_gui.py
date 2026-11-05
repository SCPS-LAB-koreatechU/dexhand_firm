"""Exercise the real Tk widgets and CAN worker in dry-run only. Needs a display.

Does not open CAN or any camera. Screenshot is optional (requires Pillow).
"""
import argparse
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    import tkinter as tk
    from bhl_hand_sdk.gui import HandGui
    root = tk.Tk()
    failures = []
    root.report_callback_exception = lambda *details: failures.append(details)
    app = HandGui(root, SimpleNamespace(project=ROOT,
        config=ROOT/'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml',
        hand='right', channel='can0', dry_run=True, model='unused'))
    def wait(predicate, timeout=3):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            root.update()
            if failures:
                raise AssertionError(f'Tk callback failed: {failures}')
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('Tk/worker state timed out')
    try:
        root.update()
        assert len(app.row_labels) == 15 and not app.connected
        assert app.worker.control is None
        assert str(app.open_all_button.cget('state')) == 'disabled'
        app.open_all()
        assert app.worker.control is None
        for side in ('left', 'right'):
            app.side.set(side)
            app._populate()
            assert app.cal_table.item('14', 'values')[1] == '18'
        assert app.cal_table.item('6', 'values')[1] == '32'
        assert app.cal_table.item('7', 'values')[1] == '25'
        app.connect()
        assert str(app.side_box.cget('state')) == 'disabled'
        assert str(app.arm_button.cget('state')) == 'disabled'
        wait(lambda: app.connected and app.last_status is not None)
        assert not app.active
        app.initial_open.set(True)
        app.post('arm', initial_open=True)
        wait(lambda: app.active)
        old = app.last_status['goal_us'][:]
        app.slider(0, .35)
        target = round(850+.35*(1775-850))
        wait(lambda: app.last_status['goal_us'][0] == target)
        assert app.last_status['goal_us'][1:] == old[1:]
        # Regression: conventional percentage units and no stale timer snap-back.
        assert float(app.motor_widgets[0].cget('to')) == 100
        app.dragging.add(0)
        app.slider(0, .10)
        old_token = app.move_tokens[0]
        app.slider(0, .20)
        new_token = app.move_tokens[0]
        app._finish_move(0, old_token)
        assert 0 in app.dirty
        app._finish_move(0, new_token)
        assert 0 in app.dirty
        app.dragging.discard(0)
        app._finish_move(0, new_token)
        assert 0 not in app.dirty
        wait(lambda: app.last_status['goal_us'][0] == round(850+.20*(1775-850)))
        app.motor_widgets[0].set(21)
        wait(lambda: app.last_status['goal_us'][0] == round(850+.21*(1775-850)))
        assert '슬라이더 사용 가능' in app.guide_text.get()
        app.slider(1, .15)
        wait(lambda: app.last_status['goal_us'][1] != app.draft.original['motors'][1]['open_us'])
        assert str(app.open_all_button.cget('state')) == 'normal'
        app.open_all()
        wait(lambda: app.last_status['goal_us'] == [m['open_us'] for m in app.draft.original['motors']])
        assert app.active
        assert all(abs(v.get()) < .01 for v in app.scales)
        app.tabs.select(2)
        wait(lambda: app.last_status['mode'] == 'calibration' and not app.active)
        assert str(app.open_all_button.cget('state')) == 'disabled'
        app.cal_table.selection_set('0')
        app.cal_pulse.set('900')
        app.record('open_us')
        assert app.draft.profile['motors'][0]['open_us'] == 900
        assert app.draft.original['motors'][0]['open_us'] == 850
        with tempfile.TemporaryDirectory(prefix='dexhand-gui-files-') as work:
            draft_path = Path(work)/'right_hand.json'
            with patch('bhl_hand_sdk.gui.filedialog.asksaveasfilename', return_value=str(draft_path)):
                app.save_draft()
            assert draft_path.is_file()
            app.draft.record(0, 'open_us', 925)
            with patch('bhl_hand_sdk.gui.filedialog.askopenfilename', return_value=str(draft_path)):
                app.load_draft()
            assert app.draft.profile['motors'][0]['open_us'] == 900
            output = Path(work)/'deployment'
            with patch('bhl_hand_sdk.gui.filedialog.asksaveasfilename', return_value=str(output)):
                app.export()
            wait(lambda: not app.exporting)
            assert (output/'firmware/DexHandRight/DexHandRight.ino').is_file()
            assert (output/'dexhand.yaml').is_file()
        app.tabs.select(1)
        wait(lambda: app.last_status['mode'] == 'camera' and not app.active)
        app.post('arm', initial_open=True)
        wait(lambda: '편 손' in app.log.get('1.0', 'end'))
        assert not app.active
        # Verify Tk can decode the exact PPM transport used by the camera child.
        app.worker.images.put_nowait(dict(captured_at=time.monotonic(),
            ppm=b'P6\n480 360\n255\n'+bytes((23, 37, 49))*(480*360)))
        wait(lambda: app.photo is not None)
        app.tabs.select(0)
        wait(lambda: app.last_status['mode'] == 'manual')
        if args.screenshot:
            from PIL import ImageGrab
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            for index, label in enumerate(('manual', 'camera', 'calibration')):
                app.tabs.select(index)
                wait(lambda: app.last_status['mode'] == ('manual', 'camera', 'calibration')[index])
                root.update()
                if sys.platform == 'win32':
                    import ctypes
                    window = ctypes.windll.user32.GetParent(root.winfo_id())
                    screenshot = ImageGrab.grab(window=window)
                else:
                    root.lift()
                    root.update()
                    screenshot = ImageGrab.grab(bbox=(root.winfo_rootx(), root.winfo_rooty(),
                        root.winfo_rootx()+root.winfo_width(), root.winfo_rooty()+root.winfo_height()))
                screenshot.save(args.screenshot.with_name(label+args.screenshot.suffix))
        print('PASS: real Tk widgets, both hand mappings, single motor isolation, calibration save/load/export, mode HOLD, camera arm gate, embedded PPM preview; no CAN/camera opened')
    finally:
        app.close()
        wait(lambda: not app.worker.thread.is_alive())
        try:
            root.destroy()
        except tk.TclError:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
