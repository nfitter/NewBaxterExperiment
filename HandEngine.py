import time
import numpy as np
from scipy.signal import butter, lfilter

class HandEngine(object):

	def __init__(self):
		# Declarations
		self._intervals = []		# List of clap time intervals
		self._accelerations = []	# List of accelerations detected
		self._startTime = None		# Time at the beginning of a clap
		self._clap = False			# Indicates whether a clap has been detected since last ping
		self._blocking = False		# Indicates whether accelerometer data is being blocked
		self._delayTime = .05		# The amount of time that the accelerometer is blocked after a clap is detected

		self.frequency = 1 / 3.0	# The frequency of arm motion

		

		self._init_time = time.time()

		self.resetStartTime()

	def ping(self):
		"""Returns true if a clap was detected since last ping"""
		if self._clap:
			self._clap = False
			return True
		return False

	def resetStartTime(self):
		self._startTime = time.time()
		self._clap = False

	def _clapDetected(self):
		"""Helper function that handles a hand clap"""

		print "--- Hand clap detected ---"

		if self._startTime == None:
			self._startTime = time.time()
		else:
			curTime = time.time()
			self._intervals.append(curTime - self._startTime)
			self._startTime = curTime

		self._clap = True
		self._blocking = True

	def calculateAmplitude(self, interval):
		"""Calculates the desired displacement of clapping motion for a given time interval"""
		return 0.15*np.exp(-0.4286*(1/interval))

	def estimateInterval(self, algorithm=2):
		"""Estimates the next clapping time interval from the list of previous intervals"""
			# algorithm = 0 -> Simple Averaging
			# algorithm = 1 -> Difference Learning
			# algorithm = 2 -> Fading Memory Difference Learning

		# Grab all the contact time readings except for the first, since first time interval may not hold any meaning
		temp_list = self._intervals[1::]
		print temp_list
		length = len(temp_list)

		if length >= 1:

			if length == 1:
				return temp_list[length - 1]

			# Shorter names used for convenience

			Li = temp_list[length - 1]
			Tc = self._startTime
			Tm = Tc - Li

			if algorithm == 0:
				return (Li + temp_list[length - 2]) / 2.0
			elif algorithm == 1:
				Iall = np.median(temp_list)
				temp = Tm + Iall + Li - (Li % Iall)

				print Iall, temp, Li, Tm

				if temp >= ((2.0/3.0) * Iall) + Tc:
					return temp - Tc
				else:
					return temp + Iall - Tc
			else:
				if length < 3:
					return Li
				I3 = np.median(temp_list[length - 3:length])
				temp = Tm + I3 + Li - (Li % I3)

				if temp >= ((2.0/3.0) * I3) + Tc:
					return temp - Tc
				else:
					return temp + I3 - Tc
