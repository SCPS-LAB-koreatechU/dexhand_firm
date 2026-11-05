"""Run in a separate process: native-library CPU incompatibility may abort Python."""
import argparse,json,time
from pathlib import Path

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--model',required=True);args=parser.parse_args()
    import mediapipe as mp,numpy as np
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    options=vision.HandLandmarkerOptions(base_options=python.BaseOptions(model_asset_path=str(Path(args.model).resolve())),running_mode=vision.RunningMode.VIDEO,num_hands=2)
    start=time.monotonic()
    with vision.HandLandmarker.create_from_options(options) as detector:
        result=detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,data=np.zeros((480,640,3),dtype=np.uint8)),0)
        assert not result.hand_landmarks
    print(json.dumps(dict(result='PASS',mediapipe=mp.__version__,elapsed_s=time.monotonic()-start,test='blank-image model inference; camera and CAN not opened')))
if __name__=='__main__':main()
