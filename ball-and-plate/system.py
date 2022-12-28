from math import sin, cos
from collections import deque
import cv2
from imutils import grab_contours
from simple_pid import PID
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# our modules
from camera import Camera
from balltracking import BallTracking

# graph and time
from matplotlib import pyplot as plt
import time

# camera
frame_dimension = 360 # square frame
low_red = (0,6,0)
high_red = (13,255,255)
low_blue = (100,150,0)
high_blue = (140,255,255)
red = True
blue = False
low_plate = (0,42,133)
high_plate = (53,255,217)
centerFlag = True
ballContourFlag = True
maskFlag = False
windowFlag = True
blueLineFlag = False
resultsFlag = True
points = deque(maxlen=100) # list of tracked points

# pids
center_x = int(frame_dimension/2)
center_y = int(frame_dimension/2)
setpoint_x = center_x
setpoint_y = center_y
sample_time = 0.08

Tc_x = 4.5 # 5.5
Kc_x = 0.38 # 0.22 , con 0.4 periodo di circa 4

if red:
	Kp_y = (Kc_x + 0.06) 		#(Kc_x*0.6) # +0.35
	Kd_y = (Kp_y*Tc_x*0.125)*2 	# *2
	Ki_y = (2*Kp_y/Tc_x)/5 		# /25 per il cerchio o senza

if blue:
	Kp_y = Kc_x + 0.05  		#(Kc_x*0.6) # +0.35
	Kd_y = (Kp_y*Tc_x*0.125)*2 	# *2
	Ki_y = (2*Kp_y/Tc_x)/5 		# /25 per il cerchio o senza

Kp_x = Kp_y
Kd_x = Kd_y
Ki_x = Ki_y

max_angle = 90

PIDx = PID(Kp=Kp_x, Ki=Ki_x, Kd=Kd_x, setpoint=setpoint_x)
PIDx.output_limits = (-max_angle,max_angle)
PIDx.sample_time = sample_time

PIDy = PID(Kp=Kp_y, Ki=Ki_y, Kd=Kd_y, setpoint=setpoint_y)
PIDy.output_limits = (-max_angle,max_angle)
PIDy.sample_time = sample_time

# square
square_mode = False
current_vertex = 0
if blue:
	half_side = int(frame_dimension/5)
if red:
	half_side = int(frame_dimension/4)

square_setpoints = [(center_x+half_side, center_y+half_side), (center_x+half_side, center_y-half_side) , (center_x-half_side, center_y-half_side), (center_x-half_side, center_y+half_side)]
square_error_range = 50 # mm

# servo
factory = PiGPIOFactory()
servo_x = Servo(18, pin_factory=factory)
servo_y = Servo(12, pin_factory=factory)

cam = Camera(0).start()
balltracking = BallTracking(frame_dimension, low_red, high_red, low_plate, high_plate)

times = []
errors_y = []
errors_x = []
duty_cycles_x = []
duty_cycles_y = []

ps_x = []
ds_x = []
is_x = []
ps_y = []
ds_y = []
is_y = []

count_x = 0
count_y = 0
count = 50

start = time.time()

while 1:
	frame = cam.read()

  	# crop the frame
	frame = balltracking.cropFrame(frame, 150, 106)

	# draw center of the plate/frame
	if centerFlag:
		cv2.circle(frame, (center_x, center_y), 2, (0,0,0), 2)
  
	# get the mask
	mask = balltracking.getMask(frame, red)
	
	if maskFlag:
		cv2.imshow("Mask", mask)
	
  	# find contours in the mask
	contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours = grab_contours(contours)
	#cv2.drawContours(frame,contours,-1,(0,0,0),3)
  
  	# find the center of the ball
	ball_center = balltracking.getBallCenter(contours)

	# square mode
	if square_mode:
		cv2.rectangle(frame, (center_x+half_side, center_y+half_side), (center_x-half_side, center_y-half_side), (0,0,255), 2)

		PIDx.setpoint = square_setpoints[current_vertex][0]
		PIDy.setpoint = square_setpoints[current_vertex][1]

		cv2.circle(frame, (PIDx.setpoint, PIDy.setpoint), 5, (0,255,0), 2)
		
		PIDx.Ki = 0
		PIDy.Ki = 0

		if red:
			PIDx.Kp = Kp_x+0.07
			PIDy.Kp = Kp_y+0.07
			PIDx.Kd = Kd_x+0.2
			PIDy.Kd = Kd_y+0.2
		if blue:
			PIDx.Kp = Kp_x+0.17
			PIDy.Kp = Kp_y+0.17
		
	else:
		PIDx.Kp = Kp_x
		PIDy.Kp = Kp_y
		PIDx.Kd = Kd_x
		PIDy.Kd = Kd_y
		PIDx.Ki = Ki_x
		PIDy.Ki = Ki_y

  	# only if ball is on the plate
	if (ball_center is not None):
		if ballContourFlag:
			cv2.circle(frame, ball_center, 20, (255,255,255), 2)
  
		if blueLineFlag:
			# update the list of tracked points
			points.appendleft(ball_center)
			balltracking.drawBlueLine(frame, points)

		times.append(time.time()-start)

		# if len(times)>2:
		# 	print(times[len(times)-1]-times[len(times)-2])

		# get duty cycle X
		current_value_x = ball_center[0]
		error_x = PIDx.setpoint-current_value_x
		errors_x.append(error_x/10) # mm->cm
		
		duty_cycle_x = PIDx(current_value_x)
		#print(f"X Err: {error_x/10}, Duty: {duty_cycle_x}")

		# get duty cycle Y
		current_value_y = ball_center[1]
		error_y = PIDy.setpoint-current_value_y
		errors_y.append(error_y/10) # mm->cm
		
		duty_cycle_y = PIDy(current_value_y)
		#print(f"Y Err: {error_y/10}, Duty: {duty_cycle_y}")

		# anti-windup
		# if (duty_cycle_x == max_angle or duty_cycle_x == -max_angle): PIDx.Ki = 0
		# else: PIDx.Ki = Ki_x
		
		# if (duty_cycle_y == max_angle): PIDy.Ki = 0
		# else: PIDy.Ki = Ki_y

		# change vertex
		if square_mode and error_x < square_error_range and error_x > -square_error_range and error_y < square_error_range and error_y > -square_error_range:
			if current_vertex == 3:
				current_vertex = 0
			else: current_vertex+=1

		# graph
		p, i, d = PIDx.components
		ps_x.append(p)
		is_x.append(i)
		ds_x.append(d)

		p, i, d = PIDy.components
		ps_y.append(p)
		is_y.append(i)
		ds_y.append(d)

		duty_cycles_y.append(duty_cycle_y)
		duty_cycles_x.append(duty_cycle_x)

	else:
		duty_cycle_x = 0
		duty_cycle_y = 0

	servo_x.value = duty_cycle_x/max_angle
	servo_y.value = duty_cycle_y/max_angle

  	# show the frame
	if windowFlag:
		cv2.imshow("Webcam", frame)
  	
	# if the 'q' key is pressed, stop the loop
	key = cv2.waitKey(1)
	if key == ord("s"):
		if square_mode:
			square_mode = False
			PIDx.setpoint = center_x
			PIDy.setpoint = center_y
		else: square_mode = True
		blueLineFlag = square_mode

	if key == ord("b"):
		balltracking.changeColor(low_blue, high_blue)
		red = False
		blue = True
	
	if key == ord("r"):
		balltracking.changeColor(low_red, high_red)
		red = True
		blue = False

	if key == ord("q"):
		servo_x.value = 0
		servo_y.value = 0
		time.sleep(1)
		break

# stop servos, camera and close all windows
servo_x.close()
servo_y.close()
cam.stop()

# graphic results
if resultsFlag:
	fig, axs = plt.subplots(3, 2)
	axs[0,0].plot(times, errors_x, 'b')
	axs[0,0].set_title('Error X')
	# axs[0,0].set_xlabel("Time [s]")
	axs[0,0].set_ylabel("Error [cm]")
	axs[0,0].grid()

	axs[0,1].plot(times, errors_y, 'r')
	# axs[0,1].set_xticks(times)
	axs[0,1].set_title('Error Y')
	# axs[0,1].set_xlabel("Time [s]")
	axs[0,1].set_ylabel("Error [cm]")
	axs[0,1].grid()

	axs[1,0].plot(times, duty_cycles_x, 'b')
	axs[1,0].set_title('Duty X')
	# axs[1,0].set_xlabel("Time [s]")
	axs[1,0].set_ylabel("Angle [°]")
	axs[1,0].grid()

	axs[1,1].plot(times, duty_cycles_y, 'r')
	axs[1,1].set_title('Duty Y')
	# axs[1,1].set_xlabel("Time [s]")
	axs[1,1].set_ylabel("Angle [°]")
	axs[1,1].grid()

	axs[2,0].plot(times, ps_x, 'r')
	axs[2,0].plot(times, is_x, 'g')
	axs[2,0].plot(times, ds_x, 'b')
	axs[2,0].legend(["P", "I", "D"])
	axs[2,0].set_title('PID X')
	axs[2,0].set_xlabel("Time [s]")
	axs[2,0].set_ylabel("PID")
	axs[2,0].grid()

	axs[2,1].plot(times, ps_y, 'r')
	axs[2,1].plot(times, is_y, 'g')
	axs[2,1].plot(times, ds_y, 'b')
	axs[2,1].legend(["P", "I", "D"])
	axs[2,1].set_title('PID Y')
	axs[2,1].set_xlabel("Time [s]")
	axs[2,1].set_ylabel("PID")
	axs[2,1].grid()

	plt.show()
	plt.close()
