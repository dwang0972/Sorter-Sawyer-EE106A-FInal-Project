from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import numpy as np
import sys
from matplotlib import pyplot as plt

def color_filter(img, lower, upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_range = np.array(lower, dtype=np.uint8)
    upper_range = np.array(upper, dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_range, upper_range)
    return cv2.bitwise_and(hsv, hsv, mask=mask)

def init_results():
	return {'rectangle': [],
			'circle': [],
			'square': []}

def categorize(shape, contour):
	results = init_results()
	if shape not in results.keys():
		return
	results[shape].append(contour)
	return results

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to the input image")

ap.add_argument("-s", "--shape", required=True,
	help="type of shape [rectangle, circle, square]")

ap.add_argument("-c", "--color", required=True,
	help="color of object [red, green, blue, yellow]")

args = vars(ap.parse_args())

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread(args["image"])
color = args["color"]
min_limit = [0,0,0]
max_limit = [255,255,255]

if color == "blue":
	# min_limit = [103, 50, 180]
	min_limit = [100, 0, 0]
	# max_limit = [130, 255, 255]
	max_limit = [255, 255, 255]
elif color == "yellow":
	min_limit = [20, 0, 0]
	max_limit = [40, 200, 255]

image = color_filter(image, min_limit, max_limit)
# cv2.imwrite("test.jpg", image)

resized = imutils.resize(image, width=300)
ratio = image.shape[0] / float(resized.shape[0])

# convert the resized image to grayscale, blur it slightly,
# and threshold it
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

# find contours in the thresholded image and initialize the
# shape detector
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if imutils.is_cv2() else cnts[1]
sd = ShapeDetector()

# loop over the contours
for c in cnts:
	# compute the center of the contour, then detect the name of the
	# shape using only the contour
	M = cv2.moments(c)
	if not M["m00"]:
		continue
	cX = int((M["m10"] / M["m00"]) * ratio)
	cY = int((M["m01"] / M["m00"]) * ratio)
	shape = sd.detect(c)

	# multiply the contour (x, y)-coordinates by the resize ratio,
	# then draw the contours and the name of the shape on the image
	c = c.astype("float")
	c *= ratio
	c = c.astype("int")
	results = categorize(shape, c)

	cv2.drawContours(image, [c], -1, (255, 255, 255), 20)
	cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
		10, (255, 255, 255), 2)

requested_shape = args["shape"]

if len(results[shape]) == 0:
	print("No requested shape found")
	sys.exit()

cx = 0
cy = 0

if requested_shape == "rectangle" or requested_shape == "square":
	if len(results[requested_shape]) != 0:
		contour = results[requested_shape][0] # set contour
		for x in contour:
			cx += x[0][0]
			cy += x[0][1]
		cx = int(cx / len(contour))
		cy = int(cy/len(contour))
		cv2.circle(image, (cx, cy), 10, (255, 255, 255), -1)

if requested_shape == "circle":
	




# show the output image
# plt.imshow(image)
cv2.imwrite("test.jpg", image)
cv2.waitKey(0)
