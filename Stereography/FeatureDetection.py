import cv2
import os
import numpy

sift = cv2.xfeatures2d.SIFT_create()

def matchDescriptions(kp1, des1, kp2, des2):
	# FLANN parameters
	FLANN_INDEX_KDTREE = 1
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks = 50)
	flann = cv2.FlannBasedMatcher(index_params, search_params)
	matches = flann.knnMatch(des1, des2, k = 2)
	good = []
	pts1 = []
	pts2 = []
	
	# ratio test as per Lowe's paper
	for i,(m,n) in enumerate(matches):
		if m.distance < 0.8*n.distance:
			good.append(m)
			pts2.append(kp2[m.trainIdx].pt)
			pts1.append(kp1[m.queryIdx].pt)

	pts1 = numpy.int32(pts1)
	pts2 = numpy.int32(pts2)
	
	return (pts1, pts2)