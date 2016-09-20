#!/usr/bin/env python

# Import objects from custom files
from HandEngine import HandEngine
from MovingArm import MovingArm

# Import other necessary packages
import rospy
import baxter_external_devices
import thread
import sys
import time
from scipy.signal import butter, lfilter
from sensor_msgs.msg import (
    Imu, Image,
)    
import matplotlib.pyplot as plt

# Set up simulated "hand contacts", plotting, and system shutdown using keypresses
def handle_keyboard(engine, arm):
	while not rospy.is_shutdown():
		c = baxter_external_devices.getch()
		if c:
			if c == 'p':
				# Keyboard input "p" registers a pretend handclap
				engine._clapDetected()
			if c == 'd':
				# Keyboard input "d" creates a plot
				plt.figure()
				plt.plot(arm._time, arm._data)
				plt.show()
			if c == 'f':
				# Keyboard input "f" needed to successfully quit code
				sys.exit(0)

# Create global variables to allow us to track all the things we want to track
global accelerations		# buffer of accelerations that we filter to see if a hand contact has occurred
accelerations = []
global motion_time			# definition of how long each cycle will be, updated as subsequent hand contacts occur
global blocking				# time duration after each hand contact that the contact detection logicengine ignores new acceleration readings, to prevent false positives
blocking = False
global last_contact_time	# time that previous hand contact occurred
global start_time			# find current time 
start_time = time.time()
global engine 				# object generated using the HandEngine.py code
global rep_num				# counter for how many cycles of hand clapping have occurred
rep_num = 0

def callback(msg):
	# Get global vars
	global accelerations
	global motion_time
	global blocking
	global last_contact_time
	global start_time

	# Name file for saving acceleration readings and save them
	file_name = 'test_rec.csv' 
	f = open(file_name, "a")
	f.write(str(time.time() - start_time) + ',' + str(msg.linear_acceleration.x) + ',' + str(msg.linear_acceleration.y) + ',' + str(msg.linear_acceleration.z) + str("\n"))
	f.close()

	# Handles accelerometer feedback and searches for handclap signals
	if not blocking:

		# If not in timeout period, listen to accelerometer readings and look for hand contact occurrence
		accelerations.append(msg.linear_acceleration.x)		# append x acceleration readings
		b = []
		a = []
		b, a = butter(1, 0.5, 'highpass')					# create filter
		x_filt = lfilter(b, a, accelerations)				# filter signal

		# Generate threshold based on hand-clapping tempo based on first study; lower for slower tempos
		threshold = 1.080028754 * (1 / motion_time) + 2.919995325

		# Check if max filtered signal is above identified threshold
		if max(x_filt) > threshold:

			# If it is, reset some variables and print out to the terminal
			engine._clapDetected()
			blocking = True
			accelerations = []
			last_contact_time = time.time()

	else:
		# If we are still in the "blocking" time, ignore accelerometer until blocking time has passed
		if time.time() > last_contact_time + .3:
			blocking = False

def main():
	# Get global vars
	global motion_time
	global engine
	global rep_num

	# Intitialize a node for this overall Baxter use
	print("Initializing node... ")
	rospy.init_node("baxter_arm_motion")

	# Set important constants for Baxter motion cycles
	motion_time = 3.0				# cycle time for first segment of Baxter "motion"
	tempM = 3.0						# future cycle time interval estimate
	factor = .1667 * motion_time	# calculate a "factor" that makes it possible to slow Baxter down if desired
	engine = HandEngine()			# object generated using the HandEngine.py code
	arm = MovingArm(1000.0)			# object generated using the MovingArm.py code
	limit = 25						# identify number of clapping cycles that user should complete

	# Compute frequency (1/T) for first robot motion cycle
	engine.frequency = 1 / (motion_time + factor)	
	
	# Define robot start pose
	start_pos = {'right_s0': -0.8620972018066407, 'right_s1': 0.35665053277587894, 'right_w0': 1.1696603494262696, 'right_w1': 1.6193157223693849, 'right_w2': -0.02070874061279297, 'right_e0': 1.5132720455200197, 'right_e1': 1.9381847232788088}

	# Compute appropriate amplitude of hand motion using eqn fit from human motion
	for_amp = engine.calculateAmplitude(motion_time + (factor))

	# Move Baxter to start pose
	arm._right_limb.move_to_joint_positions(start_pos)
	
	# Start a new thread for Baxter arm motion
	thread.start_new_thread(handle_keyboard, (engine,arm))

	# Initialize motion of Baxter's one active join in this interaction
	arm.beginSinusoidalMotion("right_w1")
	engine.resetStartTime()
	shift = False

	# Start subscriber that listens to accelerometer values
	rospy.Subscriber('/robot/accelerometer/right_accelerometer/state', Imu, callback)

	# Wait for first few contacts before moving robot
	while rep_num < 2:
		if engine.ping():
			rep_num = rep_num + 1

	# Record the time that the experiment trial is starting
	trial_start_time = time.time()

	# Do this loop during experiment trial time...
	while rep_num < limit and time.time() - trial_start_time < 40:

		# Initialize sinusoidal arm motion
		arm.updateSinusoidalMotion(for_amp, 1 / (motion_time))

		# If engine is pinged (in case of sensed hand contact)...
		if engine.ping():

			# Compute estimated appropriate next motion cycle
			tempM = engine.estimateInterval(2)

			# Flag that we should update the parameters in the sinusoidal motion generator
			#shift = True

			# Print predicted appropriate next cycle time, capping it at 0.3 sec minimum
			if tempM < 0.3:
				tempM = 0.3
			print 'Cycle time is' 
			print tempM

			# Update motion parameters if a contact was sensed in the last motion rep
			motion_time = tempM											# update cycle time
			factor = .1667 * motion_time								# update padding factor that allows for slowing down
			engine.frequency = 1 / (motion_time + factor)				# update cycle frequency
			for_amp = engine.calculateAmplitude(motion_time + factor)	# update motion amplitude
			arm.beginSinusoidalMotion("right_w1")						# start new sinusoidal motion cycle

			# Increment hand contact count
			rep_num = rep_num + 1

	# Define robot end pose
	end_pos = {'right_s0': -0.8620972018066407, 'right_s1': 0.35665053277587894, 'right_w0': 1.1696603494262696, 'right_w1': 1.8593157223693849, 'right_w2': -0.02070874061279297, 'right_e0': 1.5132720455200197, 'right_e1': 1.9381847232788088}

	# Move Baxter to end pose
	arm._right_limb.move_to_joint_positions(end_pos)

	plt.figure()
	plt.plot(arm._time, arm._data)
	plt.show()

if __name__ == "__main__":
	main()