"""
import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('custom_cmap.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
# noise removal
kernel = np.ones((3,3),np.uint8)
opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

# sure background area
sure_bg = cv2.dilate(opening,kernel,iterations=3)

# Finding sure foreground area
dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)

# Finding unknown region
sure_fg = np.uint8(sure_fg)
unknown = cv2.subtract(sure_bg,sure_fg)
# Marker labelling
ret, markers = cv2.connectedComponents(sure_fg)

# Add one to all labels so that sure background is not 0, but 1
markers = markers+1

# Now, mark the region of unknown with zero
markers[unknown==255] = 0

markers = cv2.watershed(img,markers)
img[markers == -1] = [255,0,0]

"""
from skimage.morphology import watershed
from skimage.feature import peak_local_max
import cv2
import numpy as np

from skimage.segmentation import slic
from skimage.data import astronaut
from skimage.segmentation import felzenszwalb

from sklearn.cluster import MiniBatchKMeans, KMeans
from sklearn.metrics.pairwise import pairwise_distances_argmin
from sklearn.datasets.samples_generator import make_blobs



import cv2
# cap = cv2.VideoCapture("Off-Road Car Action From the Desert _ Dakar Rally 2016-ZpMVhhXAy98.mkv")
cap = cv2.VideoCapture("/home/bowen/Desktop/Sample Videos/2016-03-01 14-18-38-542378_Partition_0.avi")


while True:
	if cap.grab():
		flag, frame = cap.retrieve()
		if not flag:
			continue
		else:
			# segments = felzenszwalb(frame, scale=10.0, sigma=0.95, min_size=50)
			# print (np.unique(segments))
			
			cv2.imshow('video', frame)
	if cv2.waitKey(10) == 27:
	    break



np.random.seed(0)

batch_size = 45
centers = [[1, 1], [-1, -1], [1, -1]]
n_clusters = len(centers)
X, labels_true = make_blobs(n_samples=3000, centers=centers, cluster_std=0.7)
Compute clustering with Means
k_means = KMeans(init='k-means++', n_clusters=3, n_init=10)
t0 = time.time()
k_means.fit(X)
t_batch = time.time() - t0
Compute clustering with MiniBatchKMeans
mbk = MiniBatchKMeans(init='k-means++', n_clusters=3, batch_size=batch_size,
                      n_init=10, max_no_improvement=10, verbose=0)
t0 = time.time()
mbk.fit(X)
t_mini_batch = time.time() - t0