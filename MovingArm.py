import rospy

import time
import baxter_interface
from baxter_interface import CHECK_VERSION

import numpy as np

class MovingArm(object):

	def __init__(self, rate):

		self._rate = rate  									# Pause between force updates
		self._missed_cmds = 20.0  							# Number of missed cycles before triggering timeout

		self._right_limb = baxter_interface.Limb('right')	# Right arm baxter sdk accessor
		self._left_limb = baxter_interface.Limb('left')		# Left arm bexter sdk accessor

		self._Kp = dict()									# Proportional gains constant
		self._Kd = dict()									# Derivative gains constant
		self._Kf = dict()									# Feedforward "gains" constant
		self._w = dict()									# Velocity smoothness constant

		self._coeffs = dict()								# Coefficients for force calculations
		self._start_time = 0.0								# Beginning of motion
		self._motion_time = 2.0								# Time to complete motion
		self._start_pos = dict()							# Initial position -- determined from arm locations
		self._end_pos = dict()								# Desired end position

		self._ovrl_disp = dict()							# Overall displacement used in force calculations
		self._vel_0 = dict()								# Detected velocity used in force calculations
		self._pos_0 = {'right_w1': 1.4593157223693849}

		self._cubic = False									# True for cubic motion, but false for sinsoidal motion

		self._amplitude = 1.0								# Amplitude used for sinusoidal motion
		self._frequency = 1/3.0								# Frequency in hz for sinusoidal motion
		self._timeElapsed = 0								# Time since the beginning of motion
		self._isSet = False 								# True if the sinusoidal motion has been set up
		self._joint = ""									# Joint on which sinusoidal motion is to be performed

		self._gtime = 0.0
		self._gst = time.time()

		self._data = []
		self._time = []

		ln = self._left_limb.joint_names()
		rn = self._right_limb.joint_names()

		self._joint_names = ln + rn 						# List of joint names for both limbs

		for joint in self._joint_names:
			self._ovrl_disp[joint] = 0.0
			self._vel_0[joint] = 0.0

		self._update_parameters()

		self._right_limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
		self._left_limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

		self._control_rate = rospy.Rate(self._rate)



		# Verify robot is enabled
		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()
		print("Running. Ctrl-c to quit")

	def move(self, right_position, left_position, length):
		""" Moves the robot to desired arm positions in the given length of time"""

		if right_position == None:
			right_position = self._right_limb.joint_angles()
		if left_position == None:
			left_position = self._left_limb.joint_angles()

		self._start_time = time.time()
		self._motion_time = length
		self._right_start_pos = self._right_limb.joint_angles()
		self._right_end_pos = right_position
		self._left_start_pos = self._left_limb.joint_angles()
		self._left_end_pos = left_position
		self._start_pos = self._right_start_pos.copy()
		self._start_pos.update(self._left_start_pos)
		self._end_pos = self._right_end_pos.copy()
		self._end_pos.update(self._left_end_pos)

		self._calculate_coeffs()
		

		while not self._update_forces():
			self._control_rate.sleep()

		#self._reset_forces()

	def moveWithStopping(self, right_position, left_position, length, interrupt):
		"""Moves the robot to the desired end positions in the given length of time. If interrupt return true, the motion ends and
		callback is called"""
		if right_position == None:
			right_position = self._right_limb.joint_angles()
		if left_position == None:
			left_position = self._left_limb.joint_angles()

		self._start_time = time.time()
		self._motion_time = length
		self._right_start_pos = self._right_limb.joint_angles()
		self._right_end_pos = right_position
		self._left_start_pos = self._left_limb.joint_angles()
		self._left_end_pos = left_position
		self._start_pos = self._right_start_pos.copy()
		self._start_pos.update(self._left_start_pos)
		self._end_pos = self._right_end_pos.copy()
		self._end_pos.update(self._left_end_pos)

		self._calculate_coeffs()

		while not self._update_forces():
			self._control_rate.sleep()
			if interrupt():
				#self._reset_forces()
				return True

		#self._reset_forces()
		return False

	def _calculate_coeffs(self):
		"""Helper function that calculates 			# Store variables to plot later
			self._data.append(des_pos)
			self._time.append(time.time())the cubic coefficients for the given motion"""
		for joint in self._joint_names:
			self._ovrl_disp[joint] = self._start_pos[joint] - self._end_pos[joint]
			self._coeffs[joint] = [(2*self._ovrl_disp[joint])/(self._motion_time ** 3), (3*-self._ovrl_disp[joint])/(self._motion_time ** 2)]

	def _update_parameters(self):
		"""Helper function that sets the control parameters for the robot"""
		Kp_vals = [40, 40, 58, 56, 27, 42, 31, 40, 40, 58, 56, 27, 42, 31]
		Kd_vals = [2, 2, 2, 2, 1, 1, 2, 2, 2, 2, 2, 1, 1, 2]
		Kf_vals = [0.2, 0.2, 0.5, 0.9, 0.02, 0.12, 0.08, 0.2, 0.2, 0.5, 0.9, 0.02, 0.12, 0.08]
		w_vals = [0.065, 0.065, 0.16, 0.085, 0.075, 0.065, 0.085, 0.065, 0.065, 0.16, 0.085, 0.075, 0.065, 0.085]

		ii = 0
		for joint in self._joint_names:
			self._Kp[joint] = Kp_vals[ii]
			self._Kd[joint] = Kd_vals[ii]
			self._Kf[joint] = Kf_vals[ii]
			self._w[joint] = w_vals[ii]
			ii = ii + 1

	def beginSinusoidalMotion(self, joint):
		self._isSet = True
		self._cubic = False
		self._timeElapsed = 0
		self._start_time = time.time()
		self._joint = joint

		self._start_pos = {'right_s0': -0.8620972018066407, 'right_s1': 0.35665053277587894, 'right_w0': 1.1696603494262696, 'right_w1': 1.6193157223693849, 'right_w2': -0.02070874061279297, 'right_e0': 1.5132720455200197, 'right_e1': 1.9381847232788088}
		cur_pos = self._pos_0
		self._start_pos[joint] = cur_pos[joint]
		self._start_pos.update({'left_s0': -0.8620972018066407, 'left_s1': 0.35665053277587894, 'left_w0': 1.1696603494262696, 'left_w1': 1.6193157223693849, 'left_w2': -0.02070874061279297, 'left_e0': 1.5132720455200197, 'left_e1': 1.9381847232788088})

	def gravityComp(self):
		grav_comp = {'right_s0': 0.0000000000, 'right_s1': -11.5113408, 'right_w0': 0.0000000000, 'right_w1': -2.21, 'right_w2': 0.0000000000, 'right_e0': 20.9088208, 'right_e1': -11.5113408}
		grav_comp_l = {'left_s0': 0.0000000000, 'left_s1': -11.5113408, 'left_w0': 0.0000000000, 'left_w1': -2.21, 'left_w2': 0.0000000000, 'left_e0': 20.9088208, 'left_e1': -11.5113408}

		self._right_limb.set_joint_torques(grav_comp)
		self._left_limb.set_joint_torques(grav_comp_l)

	def updateSinusoidalMotion(self, amp, freq):
		self._timeElapsed = time.time() - self._start_time
		self._gtime = time.time() - self._gst
		self._amplitude = amp
		self._frequency = freq

		self._update_forces()
		self._control_rate.sleep()

		a = self._right_limb.joint_angle(self._joint) - self._start_pos[self._joint]
		b = self._right_limb.joint_effort(self._joint)
		if not b == 0:
			b = b / abs(b)
		return (abs(a), b)

	# Calculate the current angular difference between the desired and the current 
	# joint positions applying a torque accordingly using PD control.
	def _update_forces(self):
		"""Helper function that calculates arm torques based on the difference between the desired and current joint positions and PD control"""

		cmd = dict()

		# Record current joint angles/velocities
		tmp_pos = self._right_limb.joint_angles()
		cur_pos = tmp_pos.copy()
		cur_pos.update(self._left_limb.joint_angles())
		tmp_vel = self._right_limb.joint_velocities()
		cur_vel = tmp_vel.copy()
		cur_vel.update(self._left_limb.joint_velocities())

		des_disp = dict()
		des_vel = dict()
		des_acc = dict()
		des_pos = dict()
		smooth_vel = dict()

		deltaT = time.time() - self._start_time

		if self._cubic:
			for joint in self._joint_names:
				# Update values for robot retreating slowly case if still within calculated reaction time
				# We can use a cubic for connecting data points, here assuming velocity is ~0 at start and end of this period
				des_disp[joint] = self._coeffs[joint][0]*(deltaT ** 3) + self._coeffs[joint][1]*(deltaT ** 2)
				des_vel[joint] = self._coeffs[joint][0]*3*(deltaT ** 2) + self._coeffs[joint][1]*2*deltaT
				des_acc[joint] = self._coeffs[joint][0]*6*deltaT + self._coeffs[joint][1]*2

					
				# Define desired robot joint angles (adding calculated displacement to centered start pose of robot)
				des_pos[joint] = self._start_pos[joint] + des_disp[joint]

				

				# Compute torque to apply calculated with PD coltrol
				#cmd[joint] = self._Kp[joint]*(desired_pose[joint]-cur_pos[joint]) + self._Kd[joint]*(desired_velocity[joint]-smooth_vel[joint])
				

				# We should add in gravity compensation, too, if we can find a good way
		else:
			des_disp = dict(zip(self._joint_names, [0.0] * 14))
			des_vel = des_disp.copy()
			des_acc = des_disp.copy()

			des_disp[self._joint] = -self._amplitude * np.sin(2 * np.pi * self._frequency * self._timeElapsed + np.pi)
			des_vel[self._joint] = -self._amplitude * 2 * np.pi * np.cos(2 * np.pi * self._frequency * self._timeElapsed + np.pi)
			des_acc[self._joint] = -self._amplitude * 4 * (np.pi ** 2) * np.sin(2 * np.pi * self._frequency * self._timeElapsed + np.pi)

			des_pos = self._start_pos.copy()
			des_pos[self._joint] = self._start_pos[self._joint] + des_disp[self._joint]

			smooth_vel[self._joint] = self._w[self._joint] * cur_vel[self._joint] + (1 - self._w[self._joint]) * self._vel_0[self._joint]

			# Save current smoothed velocity for use in the next iteration
			self._vel_0[self._joint] = smooth_vel[self._joint]
			self._pos_0[self._joint] = des_pos[self._joint]

			# Store variables to plot later
			self._data.append(des_pos[self._joint])
			self._time.append(time.time()-self._gst)

		for joint in self._joint_names:
			grav_comp = {'right_s0': 0.0000000000, 'right_s1': -11.5113408, 'right_w0': 0.0000000000, 'right_w1': -2.21, 'right_w2': 0.0000000000, 'right_e0': 20.9088208, 'right_e1': -11.5113408}
			grav_comp.update({'left_s0': 0.0000000000, 'left_s1': -11.5113408, 'left_w0': 0.0000000000, 'left_w1': -2.21, 'left_w2': 0.0000000000, 'left_e0': 20.9088208, 'left_e1': -11.5113408})

			# Compute smoothed version of current velocity offered by Rethink's SDK (limited because of coarse encoder spacing, but at least doesn't make motion jerky after smoothed)
			smooth_vel[joint] = self._w[joint] * cur_vel[joint] + (1 - self._w[joint]) * self._vel_0[joint]

			# Save current smoothed velocity for use in the next iteration
			self._vel_0[joint] = smooth_vel[joint]

			cmd[joint] = self._Kf[joint] * des_acc[joint] + self._Kp[joint]*(des_pos[joint]-cur_pos[joint]) + self._Kd[joint]*(des_vel[joint]-smooth_vel[joint])
			if not self._cubic:
				cmd[joint] += 0.14 * grav_comp[joint]

		right_torques = dict()
		left_torques = dict()
		for joint in self._right_limb.joint_names():
			right_torques[joint] = cmd[joint]

		self._right_limb.set_joint_torques(right_torques)

		f = open("ippolito_position_slow1.csv", "a")
		f.write(str(self._gtime) + ',' + str(cur_pos[self._joint]) + '\n')
		f.close()

		f = open("ippolito_desposition_slow1.csv", "a")
		f.write(str(self._gtime) + ',' + str(des_disp[self._joint]) + '\n')
		f.close()

	def _reset_forces(self):
		"""Helper function that zeroes arm torques after motion is complete"""

		vals = [0.0] * len(self._joint_names)
		cmd = dict(zip(self._joint_names, vals))

		right_torques = dict()
		left_torques = dict()
		for joint in self._right_limb.joint_names():
			right_torques[joint] = cmd[joint]
		for joint in self._left_limb.joint_names():
			left_torques[joint] = cmd[joint]

		self._right_limb.set_joint_torques(right_torques)
		self._left_limb.set_joint_torques(left_torques)
		self._right_limb.move_to_joint_positions(self._right_limb.joint_angles())
		self._left_limb.move_to_joint_positions(self._left_limb.joint_angles())
