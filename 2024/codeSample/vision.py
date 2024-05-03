import cv2 as cv
import numpy as np

img_1 = cv.imread('1.png')
img_2 = cv.imread('2.png')

# Convert the images to grayscale
gray_1 = cv.cvtColor(img_1, cv.COLOR_BGR2GRAY)
gray_2 = cv.cvtColor(img_2, cv.COLOR_BGR2GRAY)

# Apply threshold to detect white squares
_, thresh_1 = cv.threshold(gray_1, 200, 255, cv.THRESH_BINARY)
_, thresh_2 = cv.threshold(gray_2, 200, 255, cv.THRESH_BINARY)

# Find contours
contours_1, _ = cv.findContours(thresh_1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
contours_2, _ = cv.findContours(thresh_2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# Draw contours
cv.drawContours(img_1, contours_1, -1, (0, 255, 0), 3)
cv.drawContours(img_2, contours_2, -1, (0, 255, 0), 3)

# Display the thresholded images
cv.imshow('Thresholded Image 1', img_2)

cv.waitKey(0)
cv.destroyAllWindows()

