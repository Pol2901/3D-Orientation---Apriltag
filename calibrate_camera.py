import apriltag, cv2, imutils, math
import numpy as np

# ================ Camera Calibration =============

class Camera_Calibration:

   def __init__(self):
      self.camera = cv2.VideoCapture(0)
      # termination criteria
      self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
      # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
      self.objp = np.zeros((5*7,3), np.float32)
      self.objp[:,:2] = np.mgrid[0:5,0:7].T.reshape(-1,2)
#      print(self.objp)
      # Arrays to store object points and image points from all the images.
      self.objpoints = [] # 3d point in real world space
      self.imgpoints = [] # 2d points in image plane.
      
   # ------------------------------------------------------------
   def capture_chessboard(self):
      ret, self.img = self.camera.read()
#      img = cv2.imread('left01.jpg')
      if not ret:
        print("Erreur : Impossible de capturer une image depuis la caméra.")
        return
      self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
      # Find the chess board corners
      cret, corners = cv2.findChessboardCorners(self.gray, (5,7), None)

      if cret == True:
         self.objpoints.append(self.objp)
         corners2 = cv2.cornerSubPix(self.gray,corners, (11,11), (-1,-1), self.criteria)
         self.imgpoints.append(corners)
         cv2.drawChessboardCorners(self.img, (6,5), corners2, cret)
         print("Chessboard corners detected.")
      else:
         print("Failed to detect chessboard corners. Please adjust the camera or lighting.")
      cv2.imshow('img', self.img)
      cv2.waitKey(1000)
      cv2.destroyAllWindows()
#      print("ret: ", cret, corners)

   # ------------------------------------------------------------
   def calibrate(self):
      ret, self.mtx, self.distort, self.rvecs, self.tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None)
      print ('ret: ', ret)
      print ('mtx:\n', self.mtx)
      print ('distort: ', self.distort)
      print ('rvecs: ', self.rvecs)
      print ('tvecs: ', self.tvecs)
      print ('Save the calibration data to calibration_savez.npz')
      np.savez('calibration_savez', mtx=self.mtx, dist=self.distort,rvecs=self.rvecs,tvecs=self.tvecs)

   # ------------------------------------------------------------
   def undistort(self):
      h, w = self.img.shape[:2]
      self.newcameramtx, self.roi=cv2.getOptimalNewCameraMatrix(self.mtx, self.distort, (w,h), 1, (w,h))
      # undistort
      undst = cv2.undistort(self.img, self.mtx, self.distort, None, self.newcameramtx)
      # crop the image
      x, y, self.w, self.h = self.roi
      undst = undst[y:y+self.h, x:x+self.w]
      # cv2.imwrite('calibresult.png', dst)
      cv2.imshow('undistorted', undst)
      cv2.waitKey(1000)
      cv2.destroyAllWindows()

   # ------------------------------------------------------------
   def remapping(self):
    # Vérification que les matrices nécessaires sont bien définies
      if not hasattr(self, 'newcameramtx') or not hasattr(self, 'roi'):
          print("Erreur : Les paramètres de calibration ne sont pas initialisés.")
          return

    # Génération des cartes pour le remapping
      h, w = self.img.shape[:2]
      mapx, mapy = cv2.initUndistortRectifyMap(
          self.mtx, self.distort, None, self.newcameramtx, (w, h), cv2.CV_16SC2
      )

    # Application du remapping
      dstort = cv2.remap(self.img, mapx, mapy, cv2.INTER_LINEAR)

    # Vérification des dimensions de la ROI (region of interest)
      x, y, roi_w, roi_h = self.roi
      if roi_w > 0 and roi_h > 0:
          dst = dstort[y:y + roi_h, x:x + roi_w]
      else:
          print("Avertissement : La ROI est invalide, l'image ne sera pas recadrée.")
          dst = dstort

    # Affichage du résultat
      if dst.size > 0:
          cv2.imshow('Remapping', dst)
          cv2.waitKey(0)
          cv2.destroyAllWindows()
      else:
          print("Erreur : L'image après remapping est vide.")


   # ------------------------------------------------------------
   def projection_Error(self):
      mean_error = 0
      for i in range(len(self.objpoints)):
         imgpoints2, _ = cv2.projectPoints(self.objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.distort)
         error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
         mean_error += error
      print( "total error: {}".format(mean_error/len(self.objpoints)) )



# ================ Camera Calibration =============

calib_obj = Camera_Calibration()
calib_obj.capture_chessboard()
calib_obj.calibrate()
calib_obj.undistort()
calib_obj.remapping()
calib_obj.projection_Error()
import os
print(f"Fichier sauvegardé dans : {os.getcwd()}")

