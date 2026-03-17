"""

TODO
- need to adjust camera calibration according to pi cam, derive cameraMatrix.pkl and dist.pkl from program from that one guy, also adjust cam res accordingly
- need to adjust the tag_size to my printed tag size, don't got a tape measure or ruler on me rn
- need to adjust cap = cv2.VideoCapture(0) to raspberry pi cam

"""
import math
import cv2
import numpy.core
import numpy as np
from pupil_apriltags import Detector
from pathlib import Path
import pickle
from picamera2 import Picamera2

def rotation_matrix_to_euler_angles(R):
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0

        return np.degrees([roll, pitch, yaw])

class NumpyCompatUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith("numpy._core"):
            module = module.replace("numpy._core", "numpy.core", 1)
        return super().find_class(module, name)

def load_compat_pickle(path):
    with open(path, "rb") as f:
        return NumpyCompatUnpickler(f).load()

project_root = Path(__file__).resolve().parents[2]
calibration_path = project_root / "resources" / "calibration"

camera_matrix = load_compat_pickle(calibration_path / "cameraMatrix.pkl")
dist_coeffs = load_compat_pickle(calibration_path / "dist.pkl")

fx = camera_matrix[0][0]
fy = camera_matrix[1][1]
cx = camera_matrix[0][2]
cy = camera_matrix[1][2]
tag_size = .125

camera_params = [fx, fy, cx, cy]

#raspberry pi cam start
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (1920, 1080)}))
picam2.start()
picam2.set_controls({"ScalerCrop": (0, 0, 4608, 2592)})
picam2.set_controls({"AfMode": 0, "LensPosition": 0.0})

#detector object
detector = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0, quad_sigma=.0, refine_edges=1, decode_sharpening=.25)

"""
#webcam
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise RuntimeError("Could not open camera")
"""

while True:
    #ret, frame = cap.read()
    frame = picam2.capture_array()
    #cv2.imshow("Cam", frame)
    if frame is None:
        print("Could not read frame")
        break

    frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    results = detector.detect(gray,
                              estimate_tag_pose=True,
                              camera_params=camera_params,
                              tag_size=tag_size)


    

    for r in results:
        """
        corners = r.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
        center = tuple(r.center.astype(int))
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        """
        tx, ty, tz = r.pose_t.flatten()
        distance = np.linalg.norm([tx, ty, tz])
        """
        cv2.putText(frame,
                    f"ID{r.tag_id} X={tx:.2f} Y={ty:.2f} Z={tz:.2f}m D={distance:.2f}m",
                    (center[0] + 10, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    .6,
                    (255, 0, 0),
                    2)
        """
        roll, pitch, yaw = rotation_matrix_to_euler_angles(r.pose_R)
        """
        cv2.putText(frame,
                    f"Roll: {roll:.1f} Pitch: {pitch:.1f} Yaw: {yaw:.1f}",
                    (center[0], center[1] + 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2)
        """
        print(f"ID{r.tag_id} X={tx:.2f} Y={ty:.2f} Z={tz:.2f}m D={distance:.2f}m")
        print(f"Roll: {roll:.1f} Pitch: {pitch:.1f} Yaw: {yaw:.1f}\n\n")
    """
    cv2.imshow("AprilTag Webcam Pose", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    """
    

picam2.stop()
#cv2.destroyAllWindows()