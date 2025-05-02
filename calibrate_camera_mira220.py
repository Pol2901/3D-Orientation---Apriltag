import cv2
import math
import numpy as np
from picamera2 import Picamera2

# ================ Camera Calibration =============

class Camera_Calibration:

    def __init__(self):
        # Initialisation de la caméra Mira220
        self.picam2 = Picamera2()
        # Créer une configuration avec la résolution souhaitée
        camera_config = self.picam2.create_still_configuration(main={"size": (1600, 1400)})
        self.picam2.configure(camera_config)
        self.picam2.start()

        # Critères pour l'algorithme de recherche des coins d'échiquier
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Préparation des points d'objet (échiquier 6x5)
        self.objp = np.zeros((5 * 6, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:6, 0:5].T.reshape(-1, 2)

        # Stockage des points 3D et 2D pour la calibration
        self.objpoints = []  # Points d'objet dans le monde réel (3D)
        self.imgpoints = []  # Points d'image dans le plan image (2D)

    # ------------------------------------------------------------
    def capture_chessboard(self):
        # Capture d'une image
        self.img = self.picam2.capture_array()

        # Conversion en niveaux de gris
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        # Recherche des coins de l'échiquier
        ret, corners = cv2.findChessboardCorners(self.gray, (6, 5), None)

        if ret:
            self.objpoints.append(self.objp)
            corners2 = cv2.cornerSubPix(self.gray, corners, (11, 11), (-1, -1), self.criteria)
            self.imgpoints.append(corners2)
            cv2.drawChessboardCorners(self.img, (6, 5), corners2, ret)
            print("Coins d'échiquier détectés.")
        else:
            print("Échec de la détection des coins d'échiquier. Ajustez la caméra ou l'éclairage.")

        # Affichage de l'image avec les coins annotés
        cv2.imshow('Chessboard', self.img)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()

    # ------------------------------------------------------------
    def calibrate(self):
        # Calibration de la caméra
        ret, self.mtx, self.distort, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None
        )
        print("Calibration terminée.")
        print("Matrice intrinsèque :\n", self.mtx)
        print("Coefficients de distorsion :\n", self.distort)

        # Sauvegarde des résultats de calibration
        np.savez('calibration_mira220.npz', mtx=self.mtx, dist=self.distort, rvecs=self.rvecs, tvecs=self.tvecs)

    # ------------------------------------------------------------
    def undistort(self):
        # Correction de distorsion sur l'image capturée
        h, w = self.img.shape[:2]
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.distort, (w, h), 1, (w, h))
        undst = cv2.undistort(self.img, self.mtx, self.distort, None, self.newcameramtx)

        # Recadrage de l'image
        x, y, w, h = self.roi
        undst = undst[y:y + h, x:x + w]

        # Affichage de l'image corrigée
        cv2.imshow('Undistorted Image', undst)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()

    # ------------------------------------------------------------
    def remapping(self):
        # Génération des cartes pour le remapping
        h, w = self.img.shape[:2]
        mapx, mapy = cv2.initUndistortRectifyMap(self.mtx, self.distort, None, self.newcameramtx, (w, h), 5)
        dst = cv2.remap(self.img, mapx, mapy, cv2.INTER_LINEAR)

        # Affichage de l'image remappée
        cv2.imshow('Remapped Image', dst)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()

    # ------------------------------------------------------------
    def projection_Error(self):
        # Calcul de l'erreur de projection moyenne
        mean_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.distort)
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print(f"Erreur de reprojection totale : {mean_error / len(self.objpoints)}")

# ================ Camera Calibration =============

calib_obj = Camera_Calibration()
calib_obj.capture_chessboard()
calib_obj.calibrate()
calib_obj.undistort()
calib_obj.remapping()
calib_obj.projection_Error()

import os
print(f"Fichier de calibration sauvegardé dans : {os.getcwd()}")
