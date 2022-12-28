import cv2

class BallTracking:
	def __init__(self, frame_dimension, low_color, high_color, low_plate, high_plate):
		self.frame_dimension = frame_dimension
		self.low_color = low_color
		self.high_color = high_color
		self.low_plate = low_plate
		self.high_plate = high_plate

	def changeColor(self, low, high):
		self.low_color = low
		self.high_color = high

	def cropFrame(self, frame, width, height):
		(x,y,w,h) = (width, height, self.frame_dimension, self.frame_dimension)
		frame = frame[y:y+h, x:x+w]
		return frame

	def getMask(self, frame, plateFlag):
		# blur
		blurred_frame = cv2.GaussianBlur(frame, (11, 11), 0)
			
		#cv2.imshow("Blur", blurred_frame)

		# bgr to hsv
		hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

		#cv2.imshow("HSV", hsv_frame)

		# mask for the ball
		blue_mask = cv2.inRange(hsv_frame, self.low_color, self.high_color)
		mask = cv2.erode(blue_mask, None, iterations=2) # remove small blobs

		# to remove hands
		if plateFlag:
			mask_plate = cv2.inRange(hsv_frame, self.low_plate, self.high_plate)
			mask = cv2.bitwise_and(mask, mask_plate)

		# dilate remained blobs
		mask = cv2.dilate(mask, None, iterations=2)

		#cv2.imshow("Finale", mask)
	
		return mask

	def getBallCenter(self, contours):
		ball_center = None
		
		# only if one contour was found
		if len(contours) > 0:
			c = contours[0]
			
			# minimum enclosing circle
			(x, y), radius = cv2.minEnclosingCircle(c)
			
			ball_center = (int(x), int(y))
		return ball_center

	def drawBlueLine(self, frame, points):
		for i in range(1, len(points)):
			# if either of the tracked points are None, ignore them
			if points[i - 1] is None or points[i] is None:
				continue
			cv2.line(frame, points[i - 1], points[i], (255,0,0), 10)
