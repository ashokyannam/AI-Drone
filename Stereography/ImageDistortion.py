import cv2
import os

def undistortImage(img, mtx, dist, crop = True, save = None):
	h,w = img.shape[:2]
	newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
	x, y, w, h = roi
	
	# undistort
	undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)
	
	# crop the image
	if crop: undistorted = undistorted[y:y+h, x:x+w]
	
	# save the image
	if save: cv2.imwrite(res, undistorted)
	
	return undistorted