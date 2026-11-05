"""Simple direct-CAN hand test: ping, limited open/close cycle, or USB camera.

No ROS required. Only the selected hand node is opened; an existing SDK lease
prevents running over the hand_controller. Camera work runs in a separate process.
"""
import argparse
import importlib.util
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import sys
import time

from .config import load_config
from .controller import HandController
from .demo_session import DemoSession, CameraControl


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("positive finite number required")
    return result


def parser():
    from .cli import default_config
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("hand", choices=("left", "right"))
    result.add_argument("mode", choices=("ping", "cycle", "camera"), nargs="?", default="ping")
    result.add_argument("--config", default=default_config())
    result.add_argument("--channel", help="SocketCAN interface; default from the selected hand config")
    result.add_argument("--dry-run", action="store_true", help="no CAN opened; responses simulated")
    result.add_argument("--initial-open", action="store_true", help="explicit cold-start open target; first PWM cannot be interpolated from unknown physical position")
    result.add_argument("--level", type=positive, default=.25, help="maximum grip pull fraction, default .25, max .5")
    result.add_argument("--speed", type=positive, default=.25, help="maximum pull fraction per second, default .25, max 1")
    result.add_argument("--cycles", type=int, default=2)
    result.add_argument("--dwell", type=positive, default=.7)
    result.add_argument("--camera", default="0", help="USB camera index or /dev/video path")
    result.add_argument("--model", default=str(Path.home()/".cache/bhl/hand_landmarker.task"))
    result.add_argument("--headless", action="store_true", help="no OpenCV window")
    result.add_argument("--enable", action="store_true", help="headless camera: arm once after .5s open hand; no automatic re-arm")
    result.add_argument("--camera-timeout", type=positive, default=.35)
    return result


def cycle(session, cycles, dwell, *, initial_open=False, clock=time.monotonic, sleep=time.sleep, log=print):
    dt = 1/session.hand.config.control_rate_hz
    session.arm(initial_open=initial_open)
    for index in range(cycles):
        for closed in (False, True, False):
            session.set_closed(closed)
            log(f"cycle {index+1}/{cycles}: {'CLOSE' if closed else 'OPEN'} (max pull {session.level:.0%})")
            last = clock()
            end = last + 1/session.hand.config.max_speed + dwell + 2
            settled_at = None
            while True:
                now = clock()
                session.step(max(1e-6, now-last))
                last = now
                if session.hand.settled():
                    settled_at = now if settled_at is None else settled_at
                    if now-settled_at >= dwell:
                        break
                if now >= end:
                    raise RuntimeError("command interpolation timed out; physical arrival cannot be measured")
                sleep(max(0, dt-(clock()-now)))
    session.stop("CYCLE_COMPLETE")


def camera(session, args, *, log=print):
    if not Path(args.model).is_file():
        raise RuntimeError("camera model missing; run bash setup_hand_test.sh --camera first")
    for dependency in ("cv2", "mediapipe"):
        if importlib.util.find_spec(dependency) is None:
            raise RuntimeError("camera dependencies missing; run bash setup_hand_test.sh --camera")
    if not args.headless and sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise RuntimeError("no GUI display; use a local desktop, or --headless --enable")
    from .camera_source import camera_worker
    context = mp.get_context("spawn")
    output, stop = context.Queue(maxsize=1), context.Event()
    activity = context.Value("i", 0)
    worker = context.Process(target=camera_worker, kwargs=dict(output=output, stop=stop,
             camera=args.camera, model=args.model, headless=args.headless, auto_arm=args.enable,
             activity=activity, max_age=args.camera_timeout), daemon=True)
    worker.start()
    control = CameraControl(session, initial_open=args.initial_open, timeout=args.camera_timeout, log=log)
    log("Camera preview only. Show ONE OPEN hand and press SPACE to arm. H=HOLD, Q=quit.")
    if args.headless:
        log("Headless: explicit --enable permits ONE start after an open hand; tracking loss requires restart.")
    dt, next_log = 1/session.hand.config.control_rate_hz, 0.0
    try:
        while True:
            now = time.monotonic()
            sample = None
            for _ in range(4):
                try:
                    sample = output.get_nowait()
                except queue.Empty:
                    break
            if sample and sample.get("kind") == "error":
                raise RuntimeError("camera: " + sample.get("message", "error"))
            if sample and sample.get("kind") == "quit":
                break
            if stop.is_set() or not worker.is_alive():
                session.stop("CAMERA_STOPPED")
                try:
                    final = output.get(timeout=.2)
                except queue.Empty:
                    final = {}
                if final.get("kind") == "error":
                    raise RuntimeError("camera: " + final.get("message", "error"))
                if final.get("kind") == "quit":
                    break
                raise RuntimeError("camera worker stopped; no automatic resume")
            control.tick(sample)
            activity.value = control.activity
            if now >= next_log:
                if session.active:
                    log(f"APPLIED={session.results['APPLIED']} max ACK={session.max_latency_ms:.1f}ms; physical position unknown")
                elif control.gate.sample is None:
                    log("PREVIEW: waiting for a fresh camera frame")
                else:
                    age = max(0, now-control.gate.sample["captured_at"])
                    log(f"{control.gate.state}: hand={control.gate.sample['valid']}, frame age={age:.2f}s; show ONE OPEN hand and explicitly arm")
                next_log = now+1
            time.sleep(max(0, dt-(time.monotonic()-now)))
    except BaseException:
        session.stop("CAMERA_INTERRUPTED_OR_FAILED")
        raise
    finally:
        # Stop CAN streaming before waiting for a blocked camera worker to exit.
        try:
            if session.active:
                session.stop("CAMERA_EXIT")
        finally:
            activity.value = -1
            stop.set()
            worker.join(timeout=1)
            if worker.is_alive():
                worker.terminate()
                worker.join(timeout=1)
            output.close()
            output.cancel_join_thread()


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if args.level > .5 or args.speed > 1 or not 1 <= args.cycles <= 10 or not .1 <= args.camera_timeout <= 1:
        cli.error("level <=.5, speed <=1, cycles 1..10, camera timeout .1..1 seconds required")
    if args.mode == "camera" and args.headless and not args.enable:
        cli.error("headless camera needs explicit --enable; use --dry-run first")
    if args.mode == "camera" and args.enable and not args.headless:
        cli.error("GUI camera uses SPACE; --enable is only for headless mode")
    session = None
    code = 0
    try:
        config = load_config(args.config)
        config.max_speed = min(config.max_speed, args.speed)
        config.reply_timeout = max(config.reply_timeout, .02)
        config.ping_timeout = max(config.ping_timeout, .2)
        if args.channel:
            config.hand(args.hand).channel = args.channel
        hand = HandController(config, args.hand, dry_run=args.dry_run)
        session = DemoSession(hand, level=args.level)
        if hand.spec.feedback:
            pong = session.ping()
            print(f"{'SIMULATED' if args.dry_run else 'CAN'} PONG: {args.hand}, node={hand.spec.node}, protocol={pong.version}, profile={pong.profile_tag}", flush=True)
        else:
            print(f"단방향 CAN 소켓 접속: {hand.spec.channel}, ID {hand.spec.node}. 보드 수신·PWM 적용·프로필 일치는 미확인입니다.",flush=True)
        if args.mode == "cycle":
            cycle(session, args.cycles, args.dwell, initial_open=args.initial_open)
        elif args.mode == "camera":
            camera(session, args)
    except KeyboardInterrupt:
        print("Interrupted: requesting HOLD", flush=True)
        code = 130
    except Exception as error:
        print("ERROR: " + str(error), file=sys.stderr, flush=True)
        code = 1
    finally:
        if session:
            try:
                session.close()
            except Exception as error:
                print("Cleanup/HOLD error: " + str(error), file=sys.stderr)
                code = 1
            print(json.dumps(session.summary(), ensure_ascii=False, allow_nan=False), flush=True)
            if session.last_hold is not None and not session.last_hold.ok:
                print("HOLD was not acknowledged; actual PWM state is unconfirmed", file=sys.stderr)
                code = 1
    return code


if __name__ == "__main__":
    raise SystemExit(main())
