"""Check optional camera dependencies/model with blank frames; opens no camera or CAN."""
import argparse


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--model", required=True)
    args = cli.parse_args()
    import cv2
    import mediapipe as mp
    import numpy as np
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    options = vision.HandLandmarkerOptions(base_options=python.BaseOptions(model_asset_path=args.model),
                                           running_mode=vision.RunningMode.VIDEO, num_hands=2)
    with vision.HandLandmarker.create_from_options(options) as detector:
        blank = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.zeros((480, 640, 3), dtype=np.uint8))
        for timestamp in (1, 34, 67):
            result = detector.detect_for_video(blank, timestamp)
            if result.hand_world_landmarks:
                raise RuntimeError("unexpected hand detection on blank input")
    print(f"PASS: MediaPipe {mp.__version__}, OpenCV {cv2.__version__}, official model, 3 blank frames. Camera/CAN not opened.")


if __name__ == "__main__":
    main()
