from threading import Thread
import cv2

class Camera:
	def __init__(self, src=0):
		self.cam = cv2.VideoCapture(src)
		_, self.frame = self.cam.read()
		self.stopped = False

	def start(self):
		# start the thread to read frames
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping until the thread is stopped
		while 1:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				self.cam.release()
				cv2.destroyAllWindows()
				return
			# otherwise, read the next frame from the stream
			_, self.frame = self.cam.read()

	def read(self):
		# return the frame most recently read
		return self.frame
	
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
