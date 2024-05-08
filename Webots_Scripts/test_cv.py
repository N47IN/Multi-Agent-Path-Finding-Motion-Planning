import cv2
import numpy as np

# Load image
image = cv2.imread('Binary_Mask.png', cv2.IMREAD_GRAYSCALE)
params = cv2.SimpleBlobDetector_Params()
params.minThreshold = 10
params.maxThreshold = 150
params.filterByArea = True
params.minArea = 200
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False
detector = cv2.SimpleBlobDetector_create(params)

keypoints = detector.detect(image)
blank = np.zeros((1, 1))
blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255),
                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
for keyPoint in keypoints:
    x = int(keyPoint.pt[0])
    y = int(keyPoint.pt[1])
    s = int(keyPoint.size)
    print(x,y,s)
    cv2.circle(image, (x,y), s//2, (0,255,0), 2)

cv2.imshow("Detected Circles", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
