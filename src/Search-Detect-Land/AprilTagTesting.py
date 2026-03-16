import cv2
from pupil_apriltags import Detector
from pathlib import Path

detector = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0, quad_sigma=.0, refine_edges=1, decode_sharpening=.25)

script_dir = Path(__file__).resolve().parent
img_path = script_dir.parents[1] / "resources" / "apriltags" / "67.png"

img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)

if img is None:
    raise FileNotFoundError(f"Could not load image at {img_path}")

results = detector.detect(img)

for r in results:
    print("id:", r.tag_id)
    print("center:", r.center)
    print("corners:", r.corners)