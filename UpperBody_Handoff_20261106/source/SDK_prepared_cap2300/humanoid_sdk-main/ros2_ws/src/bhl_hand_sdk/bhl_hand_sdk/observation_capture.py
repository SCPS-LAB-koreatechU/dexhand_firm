"""Archive RGB frames with capture-time command history; no CAN or inference."""
import base64,collections,json,math,time
from pathlib import Path


class ObservationRecorder:
    def __init__(self,folder):
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.history=collections.deque(maxlen=300);self.last_frame=None

    def command(self,stamp,pulses):
        self.history.append((stamp,list(pulses)))

    def capture(self,source,now=None):
        now=time.monotonic() if now is None else now
        data=json.loads(Path(source).read_text());stamp=data['captured_at'];ident=data['frame_id']
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=now-stamp<=.8:return None
        if type(ident) is not int or ident<0:raise ValueError('Invalid camera frame ID')
        key=(ident,stamp)
        if key==self.last_frame:return None
        image=base64.b64decode(data.pop('image_jpeg_base64'),validate=True)
        if not image.startswith(b'\xff\xd8'):raise ValueError('Not a JPEG frame')
        path=self.folder/f'frame_{ident:08d}.jpg';path.write_bytes(image)
        preceding=[(t,p) for t,p in self.history if t<=stamp]
        command_stamp,command=(preceding[-1] if preceding else (None,None))
        self.last_frame=key
        return dict(event='observation',camera=data,image_file=str(path),
            command_at_capture_us=command,command_stamp=command_stamp,
            command_is_measured_position=False,model_landmarks_are_estimates=True)
