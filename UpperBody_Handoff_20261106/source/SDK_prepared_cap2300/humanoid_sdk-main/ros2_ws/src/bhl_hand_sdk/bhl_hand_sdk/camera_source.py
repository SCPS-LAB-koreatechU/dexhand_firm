"""MediaPipe Tasks camera worker. The child never imports or opens a CAN bus."""
import base64,math
import json
import queue
import time
from pathlib import Path


def angle(a, b, c):
    u, v = [x-y for x, y in zip(a, b)], [x-y for x, y in zip(c, b)]
    length = math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
    if not math.isfinite(length) or length < 1e-10:
        raise ValueError("degenerate landmarks")
    cosine = max(-1.0, min(1.0, sum(x*y for x, y in zip(u, v))/length))
    return math.degrees(math.acos(cosine))


def fist_score(points):
    """Heuristic PIP/DIP flexion of four fingers, not robot joint calibration."""
    if len(points) != 21 or any(len(p) != 3 or any(not math.isfinite(x) for x in p) for p in points):
        raise ValueError("21 finite 3D landmarks required")
    curls = []
    for mcp in (5, 9, 13, 17):
        pip = angle(points[mcp], points[mcp+1], points[mcp+2])
        dip = angle(points[mcp+1], points[mcp+2], points[mcp+3])
        curls.append(max(0.0, min(1.0, (175.0-(pip+dip)/2)/90.0)))
    return sum(curls)/4


def offer_latest(output, value):
    try:
        output.put_nowait(value)
    except queue.Full:
        try:
            output.get_nowait()
        except queue.Empty:
            pass
        try:
            output.put_nowait(value)
        except queue.Full:
            pass


def publish_sample(path, value):
    """Optional RAM side channel: control never waits for Tk or preview drawing."""
    if path is None:return
    target=Path(path);temporary=target.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,allow_nan=False))
    temporary.replace(target)


def publish_observation(path,frame,sample):
    if path is None:return
    import cv2
    ok,encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,92])
    if not ok:raise RuntimeError('카메라 관찰 영상 인코딩 실패')
    publish_sample(path,dict(sample,image_jpeg_base64=base64.b64encode(encoded.tobytes()).decode('ascii')))


def selected_view(camera, requested='auto', sysfs_root='/sys/class/video4linux'):
    if requested not in ('auto','full','left','right'):raise ValueError('Unknown camera view')
    if requested!='auto':return requested
    source=str(camera)
    node='video'+source if source.isdigit() else Path(source).resolve().name
    try:name=(Path(sysfs_root)/node/'name').read_text().strip()
    except OSError:name=''
    return 'left' if name.upper().startswith('ZED') else 'full'


def crop_view(frame, view):
    if view=='full':return frame
    if view not in ('left','right'):raise ValueError('Unknown camera view')
    width=frame.shape[1]
    if width<2 or width%2:raise ValueError('Stereo frame must have an even width')
    return frame[:,:width//2] if view=='left' else frame[:,width//2:]


def cpu_base_options(base_type,model,threads=4):
    """Pinned Tasks 0.10.x exposes XNNPACK threads in its acceleration proto.

    Newer C-API bindings do not expose that proto and keep their native default.
    """
    if not isinstance(threads,int) or not 1<=threads<=8:raise ValueError('CPU thread count must be 1..8')
    base=base_type(model_asset_path=model)
    if not callable(getattr(base,'to_pb2',None)):return base
    class CpuOptions(base_type):
        def to_pb2(self):
            proto=super().to_pb2();proto.acceleration.xnnpack.num_threads=threads;return proto
    return CpuOptions(model_asset_path=model)


def camera_worker(output, stop, *, camera, model, headless=False, auto_arm=False, activity=None, max_age=.35, width=640, height=480, preview=None, include_landmarks=False, stereo_view="auto", inference_threads=4, control_sample_path=None, observation_path=None, input_role='human_control'):
    capture = detector = None
    try:
        if input_role not in ('human_control','robot_observation'):raise ValueError('Unknown camera input role')
        import cv2
        cv2.setNumThreads(1)
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
        options = vision.HandLandmarkerOptions(base_options=cpu_base_options(python.BaseOptions,model,inference_threads),
                    running_mode=vision.RunningMode.VIDEO, num_hands=2,
                    min_hand_detection_confidence=0.65, min_hand_presence_confidence=0.65,
                    min_tracking_confidence=0.65)
        detector = vision.HandLandmarker.create_from_options(options)
        view_mode=selected_view(camera,stereo_view)
        source = int(camera) if str(camera).isdigit() else camera
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open USB camera {camera}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        last_ms, sequence, arm_at = -1, 0, 0.0
        closed = None
        smooth = None
        open_since = None
        auto_sent = False
        while not stop.is_set():
            captured_at = time.monotonic()
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("USB camera read failed")
            read_finished=time.monotonic()
            frame=crop_view(frame,view_mode)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            stamp_ms = max(last_ms+1, int(captured_at*1000))
            last_ms = stamp_ms
            inference_started=time.monotonic()
            result = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), stamp_ms)
            inference_finished=time.monotonic()
            # Two visible hands are ambiguous: do not silently switch people/hands.
            valid = len(result.hand_world_landmarks) == 1
            points=None;source_hand=None;hand_score=0.
            if valid and result.handedness and result.handedness[0]:
                source_hand=result.handedness[0][0].category_name;hand_score=float(result.handedness[0][0].score)
            score = None
            if valid:
                points = [(p.x, p.y, p.z) for p in result.hand_world_landmarks[0]]
                try:
                    score = fist_score(points)
                except ValueError:
                    valid = False
                if valid:
                    smooth = score if smooth is None else .35*score + .65*smooth
                    if smooth < .25:
                        closed = False
                    elif smooth > .60:
                        closed = True
                    valid = closed is not None
            landmark_valid=points is not None and len(points)==21 and all(all(math.isfinite(v) for v in p) for p in points)
            if not valid:
                smooth = closed = None
            now = time.monotonic()
            if valid and closed is False:
                open_since = now if open_since is None else open_since
            else:
                open_since = None
            if (auto_arm and not auto_sent and open_since is not None and now-open_since >= .5
                    and now-captured_at <= max_age):
                arm_at, auto_sent = now, True  # one deliberate headless enable only; never auto-rearm
            if control_sample_path is not None or observation_path is not None:
                direct=dict(kind='frame',frame_id=sequence+1,captured_at=captured_at,input_role=input_role,
                    detection_valid=landmark_valid,landmarks=points,source_hand=source_hand,handedness_score=hand_score,
                    image_landmarks=[(p.x,p.y,p.z) for p in result.hand_landmarks[0]] if landmark_valid and result.hand_landmarks else None,
                    candidate_hand_count=len(result.hand_world_landmarks),frame_size=[frame.shape[1],frame.shape[0]],
                    timing_ms=dict(inference=(inference_finished-inference_started)*1000),published_at=time.monotonic())
                publish_sample(control_sample_path,direct)
                publish_observation(observation_path,frame,direct)
            if preview is not None:
                view = cv2.flip(frame, 1)
                for landmarks in result.hand_landmarks:
                    pixels = [(int((1-p.x)*view.shape[1]), int(p.y*view.shape[0])) for p in landmarks]
                    for chain in ((0, 1, 2, 3, 4), (0, 5, 6, 7, 8), (5, 9, 10, 11, 12),
                                  (9, 13, 14, 15, 16), (13, 17, 18, 19, 20), (0, 17)):
                        for a, b in zip(chain, chain[1:]):
                            cv2.line(view, pixels[a], pixels[b], (70, 220, 80), 2)
                    for point in pixels:
                        cv2.circle(view, point, 3, (0, 180, 255), -1)
                # Preserve aspect ratio after selecting one stereo eye.
                import numpy as np
                scale=min(480/view.shape[1],360/view.shape[0]);vw=max(1,round(view.shape[1]*scale));vh=max(1,round(view.shape[0]*scale))
                canvas=np.zeros((360,480,3),dtype=np.uint8);x=(480-vw)//2;y=(360-vh)//2
                canvas[y:y+vh,x:x+vw]=cv2.resize(view,(vw,vh))
                image = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
                ppm = b'P6\n480 360\n255\n' + image.tobytes()
                offer_latest(preview, dict(ppm=ppm, captured_at=captured_at))
            if not headless:
                view = cv2.flip(frame, 1)
                label = "OPEN" if valid and closed is False else "CLOSED" if valid else "SHOW EXACTLY ONE HAND"
                cv2.putText(view, label, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 220, 0), 2)
                cv2.putText(view, "SPACE: arm/re-arm with open hand | H: HOLD | Q: quit", (12, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
                state = activity.value if activity is not None else 0
                cv2.putText(view, "Robot: " + ("ACTIVE" if state == 1 else "HOLD" if state < 0 else "PREVIEW ONLY"),
                            (15, 105), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 255), 2)
                cv2.imshow("DexHand CAN test - preview until SPACE", view)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) or cv2.getWindowProperty("DexHand CAN test - preview until SPACE", cv2.WND_PROP_VISIBLE) < 1:
                    offer_latest(output, dict(kind="quit"))
                    stop.set()
                    break
                if key == ord("h"):
                    # This control event is carried in every subsequent frame until a new arm key.
                    arm_at = -now
                if key == 32 and valid and closed is False:
                    arm_at = now
            sequence += 1
            offer_latest(output, dict(kind="frame", frame_id=sequence, captured_at=captured_at,
                                      valid=valid, closed=closed, score=score, arm_at=arm_at,
                                      detection_valid=landmark_valid, landmarks=points if include_landmarks else None,
                                      source_hand=source_hand,handedness_score=hand_score,view_mode=view_mode,frame_size=[frame.shape[1],frame.shape[0]],timing_ms=dict(read=(read_finished-captured_at)*1000,inference=(inference_finished-inference_started)*1000,total=(time.monotonic()-captured_at)*1000)))
    except Exception as error:
        if control_sample_path is not None:
            try:publish_sample(control_sample_path,dict(kind='error',message=str(error),captured_at=time.monotonic(),detection_valid=False))
            except OSError:pass
        offer_latest(output, dict(kind="error", message=str(error)))
        stop.set()
    finally:
        if capture is not None:
            capture.release()
        if detector is not None:
            detector.close()
        if not headless:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass
