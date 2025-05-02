import cv2, math
import numpy as np

# --------------------------------------------------------
class Draw:

    def __init__(self, tagobj):
        self.tagobj = tagobj
        # self.img = None

    # --------------------------------------------------------
    def annotate_Image(self, loc_x, loc_y, X, Y, Z, yaw, pitch, roll, quaternion, color=(255, 0, 0)):
        # Text for coordinates X, Y, Z
        text_translation = f'(X,Y,Z) in mm: ({X:.0f}, {Y:.0f}, {Z:.0f})'
        location_translation = (loc_x, loc_y)

        # Text for angles (Pitch, Yaw, Roll)
        text_rotation = f'(Pitch,Yaw,Roll): ({pitch:.1f}, {yaw:.1f}, {roll:.1f})'
        location_rotation = (loc_x, loc_y + 50)

        # Text for quaternion
        text_quaternion = f'(Quaternion): ({quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f})'
        location_quaternion = (loc_x, loc_y + 100)

        # Font and style
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        thickness = 2

        # Add annotations to the image
        cv2.putText(self.tagobj.img, text_translation, location_translation, font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(self.tagobj.img, text_rotation, location_rotation, font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(self.tagobj.img, text_quaternion, location_quaternion, font, font_scale, color, thickness, cv2.LINE_AA)

    # --------------------------------------------------------
    def get_Destination(self, loc, X, Y):
        print("Destination: ", loc)
        dx = loc[0] - X 
        dy = loc[1] - Y
        radian = math.atan2(dx, dy)
        self.draw_Arrow(radian)

    # --------------------------------------------------------
    def draw_Arrow(self, radian):
        arrow_length = self.tagobj.camera_width / 5
        center = (int(self.tagobj.camera_width - arrow_length * 0.6), int(self.tagobj.camera_height - arrow_length * 0.6))
        x1 = int(-arrow_length * math.sin(radian) / 2 + center[0])
        x2 = int(arrow_length * math.sin(radian) / 2 + center[0])
        y1 = int(arrow_length * math.cos(radian) / 2 + center[1])
        y2 = int(-arrow_length * math.cos(radian) / 2 + center[1])
        image = cv2.arrowedLine(self.tagobj.img, (x1, y1), (x2, y2), (0, 255, 0), 15, tipLength=0.5)
        image = cv2.circle(self.tagobj.img, center, int(arrow_length * 0.6), (0, 255, 255), 3)

    # --------------------------------------------------------
    def draw_Bounding_box(self):
        # self.tagobj.getPose()
        for result in self.tagobj.dt_results:
            (ptA, ptB, ptC, ptD) = result.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # Draw the bounding box of the AprilTag detection
            cv2.line(self.tagobj.img, ptA, ptB, (0, 255, 0), 2)
            cv2.line(self.tagobj.img, ptB, ptC, (0, 255, 0), 2)
            cv2.line(self.tagobj.img, ptC, ptD, (0, 255, 0), 2)
            cv2.line(self.tagobj.img, ptD, ptA, (0, 255, 0), 2)

    # --------------------------------------------------------
    def draw_Cube(self, tag_size=0.097, z_sign=1):
        for result in self.tagobj.dt_results:
            pose_t = result.pose_t  # Translation
            pose_R = result.pose_R  # Rotation

            # 3D points of the cube
            opoints = np.array([
                [-0.5, -0.5, 0],
                [0.5, -0.5, 0],
                [0.5, 0.5, 0],
                [-0.5, 0.5, 0],
                [-0.5, -0.5, -z_sign],
                [0.5, -0.5, -z_sign],
                [0.5, 0.5, -z_sign],
                [-0.5, 0.5, -z_sign]
            ]) * tag_size

            # Intrinsic matrix
            fx, fy, cx, cy = self.tagobj.camera_obj.camera_params
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

            # Convert rotation to rotation vector
            rvec, _ = cv2.Rodrigues(pose_R)
            tvec = pose_t.flatten()

            # Disable distortion for testing
            dcoeffs = np.zeros(5)  # Disable distortion

            # Project 3D points onto the image
            ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
            ipoints = np.round(ipoints).astype(int).reshape(-1, 2)

            # Display projected points
            for point in ipoints:
                cv2.circle(self.tagobj.img, tuple(point), 5, (0, 255, 0), -1)

            # List of cube edges
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                (0, 4), (1, 5), (2, 6), (3, 7),
                (4, 5), (5, 6), (6, 7), (7, 4)
            ]

            # Draw the edges
            for i, j in edges:
                pt1, pt2 = tuple(ipoints[i]), tuple(ipoints[j])
                cv2.line(self.tagobj.img, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

    # --------------------------------------------------------
    def showImage(self, image, length, scale=0.5):
        try:
            resize_picture = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))  # Resize image
            cv2.imshow('img', resize_picture)
        except:
            pass
        keypress = cv2.waitKey(length)
        return keypress

    # --------------------------------------------------------
    def destroyAllWindows(self):
        cv2.destroyAllWindows()

# ======================== End of Camera class =========================
