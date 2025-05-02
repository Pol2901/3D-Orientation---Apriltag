# Import necessary libraries
import mycamera, apriltag, cv2, math, json
from picamera2 import Picamera2
import dt_apriltags
import cv2
import os
import numpy as np
from transforms3d.euler import mat2euler
from transforms3d.quaternions import mat2quat

# Basic 1D Kalman filter for smoothing one value (e.g., one component of a quaternion or an angle)
class SimpleKalman:
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.error_estimate = 1.0

    def update(self, measurement):
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error_estimate = (1 - kalman_gain) * self.error_estimate + abs(self.estimate) * self.process_variance
        return self.estimate

# Main class to handle camera input, AprilTag detection, and orientation extraction
class Detector_mira220:
    def __init__(self, tag_size):
        # Initialize and configure the Raspberry Pi camera
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_still_configuration(main={"size": (1600, 1400)})
        self.picam2.configure(camera_config)
        self.picam2.start()
        self.img = None
        self.tag_size = tag_size

        # Initialize AprilTag detector
        self.detector = dt_apriltags.Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

        self.camera_obj = self.init_camera()  # Store camera parameters
        self.tag_positions = []  # Will hold information on detected tags
        self.kalman_quat = [SimpleKalman() for _ in range(4)]  # One Kalman filter per quaternion component
        self.kalman_angles = {
            "Yaw": SimpleKalman(),
            "Pitch": SimpleKalman(),
            "Roll": SimpleKalman()
        }  # Kalman filters for Euler angles

    def filter_angle(self, current_value, previous_value, alpha=0.8):
        if previous_value is None:
            return current_value
        return alpha * previous_value + (1 - alpha) * current_value

    # Camera intrinsic parameters (fx, fy, cx, cy)
    def init_camera(self):
        fx, fy, cx, cy = 1450, 1450, 960, 540
        return type('Camera', (object,), {'camera_params': [fx, fy, cx, cy]})

    # Capture an image from the camera
    def capture_Camera(self):
        self.img = self.picam2.capture_array()
        return self.img

    # Stop camera when done
    def release_Camera(self):
        self.picam2.stop()

    # Detect tags and estimate pose
    def getPose(self):
        self.tag_positions = []  # Clear previous detections
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale
        self.dt_results = self.detector.detect(
            self.gray,
            estimate_tag_pose=True,
            camera_params=self.camera_obj.camera_params,
            tag_size=self.tag_size
        )

        if not self.dt_results:
            return False  # No tags found

        # Process each detected tag
        for result in self.dt_results:
            pose_t = np.array(result.pose_t)
            pose_R = np.array(result.pose_R)

            # Construct transformation matrix (4x4) and invert it to get camera pose
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = pose_R
            transformation_matrix[:3, 3] = pose_t.flatten()
            camera_pose = np.linalg.inv(transformation_matrix)

            # Extract position (converted to mm)
            X = camera_pose[0, 3] * 1000
            Y = camera_pose[1, 3] * 1000
            Z = camera_pose[2, 3] * 1000

            # Convert rotation matrix to Euler angles and quaternion
            radian, degree = self.get_Euler(camera_pose[:3, :3])
            Roll, Pitch, Yaw = degree
            Quaternion = self.get_Quaternion(camera_pose[:3, :3])

            # Apply Kalman filter to each Euler angle
            Yaw = self.kalman_angles["Yaw"].update(Yaw)
            Pitch = self.kalman_angles["Pitch"].update(Pitch)
            Roll = self.kalman_angles["Roll"].update(Roll)

            # Apply Kalman filter to each quaternion component
            Quaternion_filtered = tuple(kf.update(q) for kf, q in zip(self.kalman_quat, Quaternion))

            # Store tag information including filtered values
            self.tag_positions.append({
                "id": result.tag_id,
                "X": X, "Y": Y, "Z": Z,
                "Yaw": Yaw, "Pitch": Pitch, "Roll": Roll,
                "Quaternion": Quaternion_filtered,
                "transformation_matrix": transformation_matrix
            })

        return True  # At least one tag found

    # Convert rotation matrix to Euler angles (radians and degrees)
    def get_Euler(self, Rmatrix):
        radian = np.array(mat2euler(Rmatrix[0:3][0:3], 'sxyz'))
        degree = np.rad2deg(radian)
        return radian, degree

    # Convert rotation matrix to quaternion (w, x, y, z)
    def get_Quaternion(self, Rmatrix):
        if Rmatrix is None:
            quaternion = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            quaternion = mat2quat(Rmatrix[0:3][0:3])
        return quaternion

    # Show the image (scaled) using OpenCV if display is available
    def showImage(self, image, length, scale=0.5):
        DISPLAY_AVAILABLE = os.getenv('DISPLAY') is not None

        if not DISPLAY_AVAILABLE:
            return ord('*')  # No display available (headless mode)
        try:
            max_width = 1000
            max_height = 875
            height, width = image.shape[:2]
            scaling_factor = min(max_width / width, max_height / height)
            new_width = int(width * scaling_factor)
            new_height = int(height * scaling_factor)
            resize_picture = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            cv2.imshow('img', resize_picture)
        except:
            pass
        keypress = cv2.waitKey(length)
        return keypress

    # Close all OpenCV windows
    def destroyAllWindows(self):
        cv2.destroyAllWindows()

print('done')
