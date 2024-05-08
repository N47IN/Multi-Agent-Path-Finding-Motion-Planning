import cv2
import numpy as np

# Load image
image = cv2.imread('Binary_Mask.png', cv2.IMREAD_GRAYSCALE)

# Invert the image
inverted_image = cv2.bitwise_not(image)

# Threshold the inverted image to get binary image
_, binary_image = cv2.threshold(inverted_image, 127, 255, cv2.THRESH_BINARY)

# Find contours
contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter contours based on area and non-convexity
min_area_threshold = 100  # Adjust as needed
# detected_circles = []
# for contour in contours:
#     area = cv2.contourArea(contour)
#     if area > min_area_threshold and not cv2.isContourConvex(contour):
#         # Fit a circle to the contour
#         (x, y), radius = cv2.minEnclosingCircle(contour)
#         center = (int(x), int(y))
#         radius = int(radius)
#         detected_circles.append((center, radius))

detected_contours = []
for contour in contours:
    area = cv2.contourArea(contour)
    if area > min_area_threshold and not cv2.isContourConvex(contour):
        if cv2.arcLength(contour, True) > 150:
            detected_contours.append(contour[:len(contour)//2])
        else:
            detected_contours.append(contour)

# Draw contours directly on the original image
result = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
cv2.drawContours(result, detected_contours, -1, (0, 255, 0), 2)

# Draw circles around detected patches
# result = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
# for circle in detected_circles:
#     center, radius = circle
#     cv2.circle(result, center, radius, (0, 255, 0), 2)

# Display result
cv2.imshow("Detected Circles", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
