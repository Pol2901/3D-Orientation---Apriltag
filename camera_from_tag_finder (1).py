import mycamera, apriltag, cv2, math, json
from picamera2 import Picamera2
import dt_apriltags
import cv2
import os
import numpy as np
from transforms3d.euler import mat2euler
from transforms3d.quaternions import mat2quat

class Detector_mira220:
    def __init__(self, tag_size):
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_still_configuration(main={"size": (1600, 1400)})  # Force raw mode at 1600x1400
        self.picam2.configure(camera_config)
        # print("Applied configuration:", self.picam2.camera_configuration())
        self.picam2.start()
        self.img = None
        self.tag_size = tag_size

        # Initialization of the AprilTag detector
        self.detector = dt_apriltags.Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

        self.camera_obj = self.init_camera()
        self.tag_positions = []  # Stores info for all detected tags

    def filter_angle(self, current_value, previous_value, alpha=0.8):
        """
        Applies an exponential filter to smooth variations.
        :param current_value: New value.
        :param previous_value: Previous value.
        :param alpha: Smoothing factor (0 < alpha <= 1).
        :return: Smoothed value.
        """
        if previous_value is None:
            return current_value  # First value
        return alpha * previous_value + (1 - alpha) * current_value

    def init_camera(self):
        fx, fy, cx, cy = 1450, 1450, 960, 540
        return type('Camera', (object,), {'camera_params': [fx, fy, cx, cy]})

    def capture_Camera(self):
        self.img = self.picam2.capture_array()
        return self.img

    def release_Camera(self):
        self.picam2.stop()

    def getPose(self):
        self.tag_positions = []  # Reset detected tags
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.dt_results = self.detector.detect(
            self.gray,
            estimate_tag_pose=True,
            camera_params=self.camera_obj.camera_params,
            tag_size=self.tag_size
        )

        if not self.dt_results:
            return False

        # Loop through all detected tags
        for result in self.dt_results:
            pose_t = np.array(result.pose_t)
            pose_R = np.array(result.pose_R)

            # Homogeneous transformation matrix (4x4)
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = pose_R
            transformation_matrix[:3, 3] = pose_t.flatten()

            # Invert to get camera pose relative to the tag
            camera_pose = np.linalg.inv(transformation_matrix)

            # Extract positions
            X = camera_pose[0, 3] * 1000
            Y = camera_pose[1, 3] * 1000
            Z = camera_pose[2, 3] * 1000

            # Extract angles
            radian, degree = self.get_Euler(camera_pose[:3, :3])
            Roll, Pitch, Pitch = degree
            Quaternion = self.get_Quaternion(camera_pose[:3, :3])

            # Store detected tag information
            self.tag_positions.append({
                "id": result.tag_id,
                "X": X, "Y": Y, "Z": Z,
                "Yaw": Yaw, "Pitch": Pitch, "Roll": Roll,
                "Quaternion": Quaternion,
                "transformation_matrix": transformation_matrix
            })

        return True  # At least one tag found

    def get_Euler(self, Rmatrix):
        radian = np.array(mat2euler(Rmatrix[0:3][0:3], 'sxyz'))
        degree = np.rad2deg(radian)
        return radian, degree

    def get_Quaternion(self, Rmatrix):
        """
        Converts 3x3 rotation matrix into a quaternion (w, x, y, z).
        """
        if Rmatrix is None:
            quaternion = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            quaternion = mat2quat(Rmatrix[0:3][0:3])  # Conversion matrix → quaternion
        return quaternion  # Returns (w, x, y, z)

    def showImage(self, image, length, scale=0.5):
        # Check if we have access to a display
        DISPLAY_AVAILABLE = os.getenv('DISPLAY') is not None

        if not DISPLAY_AVAILABLE:
            # print("[INFO] Headless mode detected: no display with cv2.imshow()")
            return ord('*')  # Return arbitrary value to avoid blocking
        try:
            # Set a maximum display size (adjust according to your screen)
            max_width = 1000  # Maximum width in pixels
            max_height = 875  # Maximum height in pixels

            # Get current image size
            height, width = image.shape[:2]

            # Compute scaling factor to maintain proportions
            scaling_factor = min(max_width / width, max_height / height)

            # Compute new size
            new_width = int(width * scaling_factor)
            new_height = int(height * scaling_factor)

            # Resize image while keeping proportions
            resize_picture = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            cv2.imshow('img', resize_picture)
        except:
            pass
        keypress = cv2.waitKey(length)
        return keypress

    def destroyAllWindows(self):
        cv2.destroyAllWindows()

print('done')
