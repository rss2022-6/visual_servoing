import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	xx, yy, xw, yh =0,0,0,0

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	kernel = np.ones((3,3),np.uint8)
	# median = cv2.medianBlur(dilation,11)
	frame_threshold = cv2.inRange(hsv, (0, 140, 150), (30, 255, 255))
	hsv_filter = cv2.bitwise_and(img, img, mask=frame_threshold)
	# image_print(hsv_filter)
	erosion = cv2. erode(hsv_filter,kernel,iterations = 1)
	dilation = cv2.dilate(erosion,kernel,iterations = 1)
	grey = cv2.cvtColor(dilation, cv2.COLOR_BGR2GRAY)
	im2, contours, hierarchy = cv2.findContours(grey, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
	x,y,w,h = 0,0,0,0
	area = 0
	for contour in contours:
		x_,y_,w_,h_ = cv2.boundingRect(contour)
		if w_*h_>w*h:
			x,y,w,h=x_,y_,w_,h_


	bounding_box = ((x,y),(x+w,y+h))
	cv2.rectangle(img,(x, y),(x+w,y+h),(0,0,255),3)
	#image_print(img)
	xx,yy,xw,yh=x,y,x+w,y+h

	# x_save = int(x*0.9)
	# y_save = int(y*0.9)
	# hsv2 = hsv[int(y*0.9):int((y+h)*1.1),int(x*0.9):int((x+w)*1.1)]
	# img2 = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

	# frame_threshold = cv2.inRange(hsv2, (0, 150, 150), (45, 255, 255))
	# hsv_filter = cv2.bitwise_and(img2, img2, mask=frame_threshold)
	# # image_print(hsv_filter)
	# erosion = cv2. erode(hsv_filter,kernel,iterations = 1)
	# dilation = cv2.dilate(erosion,kernel,iterations = 1)
	# grey = cv2.cvtColor(dilation, cv2.COLOR_BGR2GRAY)
	# im2, contours, hierarchy = cv2.findContours(grey, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
	# x,y,w,h = 0,0,0,0
	# for contour in contours:
	# 	x_,y_,w_,h_ = cv2.boundingRect(contour)
	# 	if w_*h_>w*h:
	# 		x,y,w,h=x_,y_,w_,h_

	# xx = min(xx, x+x_save)
	# yy = min(yy, y+y_save)
	# xw = max(xw, x+w+x_save)
	# yh = max(yh, y+h+y_save)

	# cv2.rectangle(hsv,(xx, yy),(xw, yh),(0,0,255),3)

	# image_print(hsv)
	# bounding_box = ((x+x_save,y+y_save),(x+w+x_save,y+h+y_save))
	bounding_box = ((xx, yy),(xw, yh))

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

def lf_color_segmentation(img, template):
	"""
	Implement the line following detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a line to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the line, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	img = img[230:260]
	xx, yy, xw, yh =0,0,0,0

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	kernel = np.ones((3,3),np.uint8)
	frame_threshold = cv2.inRange(hsv, (0, 140, 150), (30, 255, 255))
	hsv_filter = cv2.bitwise_and(img, img, mask=frame_threshold)
	erosion = cv2.erode(hsv_filter,kernel,iterations = 1)
	dilation = cv2.dilate(erosion,kernel,iterations = 1)
	grey = cv2.cvtColor(dilation, cv2.COLOR_BGR2GRAY)
	im2, contours, hierarchy = cv2.findContours(grey, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
	x,y,w,h = 0,0,0,0
	area = 0
	for contour in contours:
		x_,y_,w_,h_ = cv2.boundingRect(contour)
		if w_*h_>w*h:
			x,y,w,h=x_,y_,w_,h_

	y = y + 220
	bounding_box = ((x,y),(x+w,y+h))
	cv2.rectangle(img,(x, y),(x+w,y+h),(0,0,255),3)
	xx,yy,xw,yh=x,y,x+w,y+h

	bounding_box = ((xx, yy),(xw, yh))

	# cv2.rectangle(img,(x, y),(x+w,y+h),(0,0,255),3)
	# image_print(img)
	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
