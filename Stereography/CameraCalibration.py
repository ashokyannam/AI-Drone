import numpy
import cv2
import glob
import os

default_chessboard = numpy.zeros((7*7,3), numpy.float32)
default_chessboard[:,:2] = numpy.mgrid[0:7,0:7].T.reshape(-1,2)

def calibrateCamera(chessboard = default_chessboard, imgpath = 'calibrate', imgext = 'jpg'):
	# termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	
	dir = imgpath + ('' if imgpath[-1] in ['\\', '/'] else '/')
	
	if os.path.isdir(dir):
		path = dir + "*." + imgext

		images = glob.glob(path)

		for fname in images:
			img = cv2.imread(fname)
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

			# Find the chess board corners
			ret, corners = cv2.findChessboardCorners(gray, (7,7), None)

			# If found, add object points, image points (after refining them)
			if ret == True:
				objpoints.append(chessboard)

				corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
				imgpoints.append(corners2)

				# Draw and display the corners
				img = cv2.drawChessboardCorners(img, (7, 7), corners2, ret)
				img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
				cv2.imshow("img", img)
				cv2.waitKey(5000)

				cv2.destroyAllWindows()

				ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
				
				return (True, (mtx, dist, (rvecs, tvecs)))
	
	return (False, None)