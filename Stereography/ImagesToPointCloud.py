import numpy
import cv2
from CameraCalibration import calibrateCamera
from ImageDistortion import undistortImage
from FeatureDetection import *
import codecs, json
from numpy.linalg import svd as svd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math

# cache of camera calibration
calibration = (numpy.array([[2.64581993e+03, 0.00000000e+00, 1.77182865e+03],
       [0.00000000e+00, 2.70952979e+03, 1.04783171e+03],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
	   numpy.array([[-0.30015665,  0.35530861,  0.01702952,  0.02882908,  0.19575338]]))
#calibration = (None, None)
K, distortion = calibration

# if calibration not cached, calibrate camera
if K is None or distortion is None:
	print("Calibrating...")
	success, calibration = calibrateCamera()
	if not success: exit("Failed calibration. Please try again.")
	K, distortion, _ = calibration

print("Calibrated:")
print((K, distortion))

left = cv2.imread("left4.jpg")
right = cv2.imread("right4.jpg")

left = undistortImage(left, K, distortion)
right = undistortImage(right, K, distortion)

left = cv2.resize(left, (0, 0), fx=0.5, fy=0.5)
right = cv2.resize(right, (0, 0), fx=0.5, fy=0.5)

kp1, des1 = sift.detectAndCompute(left, None)
kp2, des2 = sift.detectAndCompute(right, None)

pts1, pts2 = matchDescriptions(kp1, des1, kp2, des2)

F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_LMEDS)

# We select only inlier points
pts1 = pts1[mask.ravel()==1]
pts2 = pts2[mask.ravel()==1]

K_inv = numpy.linalg.inv(K)

# calculate E as K'^T * F * K
E = K_inv.T.dot(F).dot(K)

# decompose E to get U, S, Vt
U, S, Vt = svd(E)
W = numpy.array([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]).reshape(3, 3)

first_inliers = []
second_inliers = []
for i in range(len(pts1)):
	# normalize and homogenize the image coordinates
	first_inliers.append(K_inv.dot([pts1[i][0], pts1[i][1], 1.0]))
	second_inliers.append(K_inv.dot([pts2[i][0], pts2[i][1], 1.0]))

def in_front_of_both_cameras(first_points, second_points, rot, trans):
	# check if the point correspondences are in front of both images
	rot_inv = rot
	for first, second in zip(first_points, second_points):
		first_z = numpy.dot(rot[0, :] - second[0]*rot[2, :], trans) / numpy.dot(rot[0, :] - second[0]*rot[2, :], second)
		first_3d_point = numpy.array([first[0] * first_z, second[0] * first_z, first_z])
		second_3d_point = numpy.dot(rot.T, first_3d_point) - numpy.dot(rot.T, trans)
		
		if first_3d_point[2] < 0 or second_3d_point[2] < 0:
			return False
	
	return True

R = U.dot(W).dot(Vt)
T = U[:, 2]
if not in_front_of_both_cameras(first_inliers, second_inliers, R, T):
	# Second choice: R = U * W * Vt, T = -u_3
	T = - U[:, 2]
	if not in_front_of_both_cameras(first_inliers, second_inliers, R, T):
		
		# Third choice: R = U * Wt * Vt, T = u_3
		R = U.dot(W.T).dot(Vt)
		T = U[:, 2]
		
		if not in_front_of_both_cameras(first_inliers, second_inliers, R, T):
			
			# Fourth choice: R = U * Wt * Vt, T = -u_3
			T = - U[:, 2]

'''def drawlines(img1,img2,lines,pts1,pts2):
	# img1 - image on which we draw the epilines for the points in img1
	#	lines - corresponding epilines
	pts1 = numpy.int32(pts1)
	pts2 = numpy.int32(pts2)
	r,c,h = img1.shape
	for r,pt1,pt2 in zip(lines,pts1,pts2):
		color = tuple(numpy.random.randint(0,255,3).tolist())
		x0,y0 = map(int, [0, -r[2]/r[1] ])
		x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
		cv2.line(img1, (x0,y0), (x1,y1), color,1)
		cv2.circle(img1,tuple(pt1), 10, color, -1)
		cv2.circle(img2,tuple(pt2), 10,color,-1)
	return img1,img2

# Find epilines corresponding to points in right image (second image)
lines1 = cv2.computeCorrespondEpilines(pts2.reshape(-1,1,2), 2, F)
lines1 = lines1.reshape(-1,3)
img5,img6 = drawlines(left,right,lines1,pts1,pts2)

# Find epilines corresponding to points in left image (first image)
lines2 = cv2.computeCorrespondEpilines(pts1.reshape(-1,1,2), 1, F)
lines2 = lines2.reshape(-1,3)
img3,img4 = drawlines(right,left,lines2,pts2,pts1)

plt.subplot(121),plt.imshow(img5)
plt.subplot(122),plt.imshow(img3)
plt.show()'''

R1 = numpy.zeros((3,3))
R2 = numpy.zeros((3,3))
P1 = numpy.zeros((3,4))
P2 = numpy.zeros((3,4))
Q = numpy.zeros((4,4))

cv2.stereoRectify(K, distortion, K, distortion, right.shape[0:2], R, T, R1, R2, P1, P2, Q)

def create_matchers(imgL, imgR, window_size = 15):
	left_matcher = cv2.StereoSGBM_create(
		minDisparity=0,
		numDisparities=256,             # max_disp has to be dividable by 16 f. E. HH 192, 256
		blockSize=5,
		P1=8 * 3 * window_size ** 2,
		P2=32 * 3 * window_size ** 2,
		disp12MaxDiff=1,
		uniquenessRatio=15,
		speckleWindowSize=0,
		speckleRange=2,
		preFilterCap=63,
		mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
	)

	right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
	
	displ = left_matcher.compute(imgL, imgR)
	dispr = right_matcher.compute(imgR, imgL)
	displ = numpy.int16(displ)
	dispr = numpy.int16(dispr)
	
	return left_matcher, right_matcher, displ, dispr

def wls_disparity_filter(displ, dispr, left_matcher = None, right_matcher = None, imgL = None, imgR = None, lmbda = 80000, sigma = 1.2, visual_multiplier = 1.0):
	wls_filter = None
	disparity = None
	
	if imgL is not None and left_matcher is not None:
		wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
	elif imgR is not None and right_matcher is not None:
		wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_right=right_matcher)
	else:
		return None
	
	wls_filter.setLambda(lmbda)
	wls_filter.setSigmaColor(sigma)
	
	if imgL is not None and left_matcher is not None:
		disparity = wls_filter.filter(displ, imgL, None, dispr)
	else:
		disparity = wls_filter.filter(displ, imgR, None, dispr)
	
	disparity = cv2.normalize(src=disparity, dst=disparity, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
	disparity = numpy.uint8(disparity)
	
	return disparity

def calc_point_cloud(image, disp, q):
	points = cv2.reprojectImageTo3D(disp, q).reshape(-1, 3)
	colors = image.reshape(-1, 3)
	da = disp.reshape(-1)
	mask = (
		(da > da.min()) & 
		numpy.all(~numpy.isnan(points), axis=1) & 
		numpy.all(~numpy.isinf(points), axis=1)
	)
	return points[mask], colors[mask]

def project_points(points, colors, r, t, k, dist_coeff, width, height):
	projection, _ = cv2.projectPoints(points, r, t, k, dist_coeff)
	xy = projection.reshape(-1, 2).astype(numpy.int)
	mask = (
		(0 <= xy[:, 0]) & (xy[:, 0] < width) &
		(0 <= xy[:, 1]) & (xy[:, 1] < height)
	)
	return xy[mask], colors[mask]

def calc_projected_image(points, colors, r, t, k, dist_coeff, width, height):
	xy, cm = project_points(points, colors, r, t, k, dist_coeff, width, height)
	image = numpy.zeros((height, width, 3), dtype=colors.dtype)
	image[xy[:, 1], xy[:, 0]] = cm
	return image

def rotate(arr, anglex, anglez):
	return numpy.array([
		[1, 0, 0],
		[0, numpy.cos(anglex), -numpy.sin(anglex)],
		[0, numpy.sin(anglex), numpy.cos(anglex)]
	]).dot(numpy.array([
		[numpy.cos(anglez), 0, numpy.sin(anglez)],
		[0, 1, 0],
		[-numpy.sin(anglez), 0, numpy.cos(anglez)]
	])).dot(arr)

height, width = left.shape[0:2]

def view(r, t):
	cv2.imshow('projected', calc_projected_image(
		points, colors, r, t, K, distortion, width, height
	))

matchl, matchr, displ, dispr = create_matchers(left, right)
disparity = wls_disparity_filter(displ, dispr, left_matcher=matchl, imgL=left)
points, colors = calc_point_cloud(left, disparity, Q)

cv2.imshow('Disparity Map', disparity)
cv2.waitKey()
cv2.destroyAllWindows()

'''
points = points[::10]
colors = colors[::10]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xs=points[:, 0], ys=points[:, 1], zs=points[:, 2], c=colors/255.0)
plt.show()

def basic_rotation(axis, angle, degrees=False):
	mat = numpy.eye(3,3)
	
	if axis in ['x', 'y', 'z']:
		if degrees: angle *= math.pi/180.0
		
		if axis == 'x':
			mat = numpy.array([
				[1, 0, 0],
				[0, numpy.cos(angle), -numpy.sin(angle)],
				[0, numpy.sin(angle), numpy.cos(angle)]
			])
		elif axis == 'y':
			mat = numpy.array([
				[numpy.cos(angle), 0, numpy.sin(angle)],
				[0, 1, 0],
				[-numpy.sin(angle), 0, numpy.cos(angle)]
			])
		else:
			mat = numpy.array([
				[numpy.cos(angle), -numpy.sin(angle), 0],
				[numpy.sin(angle), numpy.cos(angle), 0],
				[0, 0, 1]
			])
	else:
		print('Cannot rotate about unknown axis ' + str(axis) + '.')
	
	return mat

r = R2
t = T

view(r, t)

angles = { # x, z
	'w': (-numpy.pi/16, 0),
	's': (numpy.pi/16, 0),
	'a': (0, numpy.pi/16),
	'd': (0, -numpy.pi/16)
}

alt = {
	'y': (('x', 10, True), ('x', -20, True)),
	'h': (('x', -10, True), ('x', 20, True))
}

while 1:
	key = cv2.waitKey(0)
	
	if key not in range(256):
		continue
	
	ch = chr(key)
	if ch in angles:
		ax, az = angles[ch]
		r = rotate(r, -ax, -az)
		t = rotate(t, ax, az)
		view(r, t)
	
	elif ch in alt:
		dr, dt = alt[ch]
		if dr is not None:
			axis, angle, degrees = dr
			r = basic_rotation(axis, angle, degrees).dot(r)
		if dt is not None:
			axis, angle, degress = dt
			t = basic_rotation(axis, angle, degrees).dot(t)
		view(r, t)
	
	elif ch == '\x1b': # esc
		cv2.destroyAllWindows()
		break'''