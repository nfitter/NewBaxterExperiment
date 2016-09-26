#!/usr/bin/env python

# Import objects from custom files
from HandEngine import HandEngine
from MovingArm import MovingArm
from Head import Head

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
import argparse                             # package for allowing user input

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
                plt.plot(accelerations)
                plt.show()
            if c == 'f':
                # Keyboard input "f" needed to successfully quit code
                sys.exit(0)

# Create global variables to allow us to track all the things we want to track
global accelerations        # buffer of accelerations that we filter to see if a hand contact has occurred
accelerations = []
global motion_time            # definition of how long each cycle will be, updated as subsequent hand contacts occur
global blocking                # time duration after each hand contact that the contact detection logicengine ignores new acceleration readings, to prevent false positives
blocking = False
global last_contact_time    # time that previous hand contact occurred
global start_time            # find current time 
start_time = time.time()
global engine                 # object generated using the HandEngine.py code
global rep_num                # counter for how many cycles of hand clapping have occurred
rep_num = 0

############################ Accelerometer callback function ###############################

def callback(msg):
    # Get global vars
    global accelerations
    global motion_time
    global blocking
    global last_contact_time
    global start_time
    global engine

    # Name file for saving acceleration readings and save them
    file_name = 'test_rec.csv' 
    f = open(file_name, "a")
    f.write(str(time.time() - start_time) + ',' + str(msg.linear_acceleration.x) + ',' + str(msg.linear_acceleration.y) + ',' + str(msg.linear_acceleration.z) + str("\n"))
    f.close()

    # Handles accelerometer feedback and searches for handclap signals
    if not blocking:

        # If not in timeout period, listen to accelerometer readings and look for hand contact occurrence
        accelerations.append(msg.linear_acceleration.x)        # append x acceleration readings
        b, a = butter(1, 0.5, 'highpass')                    # create filter
        x_filt = lfilter(b, a, accelerations)                # filter signal

        # Generate threshold based on hand-clapping tempo based on first study; lower for slower tempos
        threshold = (1 / motion_time) + 2.0

        # Check if max filtered signal is above identified threshold for moving case
        if max(x_filt) > threshold:

            # If it is, reset some variables and print out to the terminal
            engine._clapDetected()
            blocking = True
            accelerations = []
            last_contact_time = time.time()

    else:
        # If we are still in the "blocking" time, ignore accelerometer until blocking time has passed
        if time.time() > last_contact_time + 0.3:
            blocking = False

############################# Main trial routine function #################################

def main():
    # Get global vars
    global motion_time
    global engine
    global rep_num
    global last_contact_time

    # Let user input experiment trial number (1-25, defined in dictionary below)
    parser = argparse.ArgumentParser(description = 'Input stimulus number.')
    parser.add_argument('integers', metavar = 'N', type = int, nargs = '+', help = 'an integer representing exp conds')
    foo = parser.parse_args()
    trial_cond = (foo.integers[0])

    # Access lookup table of conditions based on this input, assign appropriate values to global variables controlling trial conditions
    trial_stimuli = {1:[True,True,1.333], 2:[True,True,1.833],
            3:[True,False,1.333], 4:[True,False,1.833],
            5:[False,True,1.333], 6:[False,True,1.833],
            7:[False,False,1.333], 8:[False,False,1.833]}

    # Break out selected dictionary entries into experiment trial variables
    robot_lead = trial_stimuli[trial_cond][0] # set Boolean for turning face animation on or off
    follower_coop = trial_stimuli[trial_cond][1] # set Boolean for turning physical response on or off
    freq = trial_stimuli[trial_cond][2] # Hz, set robot clapping frequency

    # Define uncooperative robot time intervals
    dummy_ints = [0.8, 0.4, 0.7, 0.5, 0.45, 0.65, 0.55, 0.7, 0.6, 0.5, 0.6, 0.55, 0.45, 0.7, 0.75, 0.6, 0.45, 0.8, 0.7, 0.55, 0.45, 0.4, 0.45, 0.65, 0.7, 0.45]

    # Intitialize a node for this overall Baxter use
    print("Initializing node... ")
    rospy.init_node("baxter_arm_motion")

    # Set important constants for Baxter motion cycles
    motion_time = 1.0/freq                # cycle time for first segment of Baxter "motion"
    tempM = 1.0/freq                    # future cycle time interval estimate
    mult_factor = 0.0                    # factor for multiplication mentioned in next line
    factor = mult_factor * motion_time    # calculate a "factor" that makes it possible to slow Baxter down if desired
    engine = HandEngine()                # object generated using the HandEngine.py code
    arm = MovingArm(1000.0)                # object generated using the MovingArm.py code
    face = Head("faces/")      # object generated using the Head.py code
    limit = 25                            # identify number of clapping cycles that user should complete
    elapsed_time = dummy_ints[0]        # keep track of elapsed time into trial

    # Load face image on Baxter's screen
    thread.start_new_thread(face.run,(10,))
    face.changeEmotion('Happy')

    # Make robot look to the right
    face.changeGaze('E')

    # Wait to make sure face engine has time to start up
    #rospy.sleep(5.)

    # Define robot start pose
    start_pos = {'right_s0': -0.8620972018066407, 'right_s1': 0.35665053277587894, 'right_w0': 1.1696603494262696, 'right_w1': 1.3593157223693849, 'right_w2': -0.02070874061279297, 'right_e0': 1.5132720455200197, 'right_e1': 1.9381847232788088}

    # Move Baxter to start pose
    arm._right_limb.move_to_joint_positions(start_pos)

    # Initialize motion of Baxter's one active join in this interaction
    arm.beginSinusoidalMotion("right_w1")
    engine.resetStartTime()
    shift = False

    # Start a new thread for keyboard control
    thread.start_new_thread(handle_keyboard, (engine,arm))

    # Start subscriber that listens to accelerometer values
    rospy.Subscriber('/robot/accelerometer/right_accelerometer/state', Imu, callback)

    # Set control rate
    control_rate = rospy.Rate(1000)

    # Print cue so operator knows robot is ready
    print 'Ready to start trial'

    ################################ Robot Leader Case ##########################################

    # Part of code for robot lead condition, either follower cooperation case
    if robot_lead:

        # Compute frequency (1/T) for robot motion cycles
        engine.frequency = 1 / (motion_time)

        # Compute appropriate amplitude of hand motion using eqn fit from human motion
        for_amp = engine.calculateAmplitude(motion_time)

        # Record the time that the experiment trial is starting
        trial_start_time = time.time()

        # Do this loop during experiment trial time...
        while rep_num < limit and time.time() - trial_start_time < 40:

            # Initialize sinusoidal arm motion
            arm.updateSinusoidalMotion(for_amp, engine.frequency)

            # If engine is pinged (in case of sensed hand contact)...
            if engine.ping():

                # Increment hand contact count
                rep_num = rep_num + 1

                # Set reactive facial expression
                face.switchEmotion('Reactive','Blink','Happy','NW',0.15)

    ################################ Robot Follower Case #########################################

    # Part of code for robot follow condition
    else:

        # Compute frequency (1/T) for first robot motion cycle
        engine.frequency = 1 / (motion_time + factor)    
        
        # Compute appropriate amplitude of hand motion using eqn fit from human motion
        for_amp = 0

        # Record the time that the experiment trial is starting
        trial_start_time = time.time()

        # Do this loop during experiment trial time...
        while rep_num < limit and time.time() - trial_start_time < 40:

            # Initialize sinusoidal arm motion
            arm.updateSinusoidalMotion(for_amp, engine.frequency)

            # Wait for first few contacts before moving robot
            if rep_num < 3:

                # If engine is pinged (in case of sensed hand contact)...
                if engine.ping():

                    # For first few contacts, use dummy 0-amplitude curve
                    if rep_num < 2:

                        # Command 0-amplitude curve
                        arm.updateSinusoidalMotion(for_amp, engine.frequency)
                        trial_start_time = time.time()
                    
                    # Then get ready with the right sinusoid parameters
                    else:
                        tempM = engine.estimateInterval(1)                              # pred next motion cycle period
                        motion_time = tempM                                            # update cycle time
                        engine.frequency = 1 / (motion_time + factor)                   # update cycle frequency
                        for_amp = engine.calculateAmplitude(motion_time + (factor))        # update motion amplitude
                        
                    # Increment hand contact count
                    rep_num = rep_num + 1

            # Then start serious motion
            else:

                # Initialize sinusoidal arm motion
                arm.updateSinusoidalMotion(for_amp, engine.frequency)

                # Part of code for cooperative robot follower condition
                if follower_coop:

                    # If engine is pinged (in case of sensed hand contact)...
                    if engine.ping():

                        # Compute estimated appropriate next motion cycle
                        tempM = engine.estimateInterval(1)

                        # Print predicted appropriate next cycle time, capping it at 0.3 sec minimum
                        if tempM < 0.3:
                            tempM = 0.3
                        print 'Cycle time is' 
                        print tempM

                        # Update motion parameters if a contact was sensed in the last motion rep
                        motion_time = tempM                                            # update cycle time
                        factor = mult_factor * motion_time                            # update padding factor that allows for slowing down
                        engine.frequency = 1 / (motion_time + factor)                # update cycle frequency
                        for_amp = engine.calculateAmplitude(motion_time + factor)    # update motion amplitude
                        arm.beginSinusoidalMotion("right_w1")                        # start new sinusoidal motion cycle

                        # Increment hand contact count
                        rep_num = rep_num + 1

                # Part of code for uncooperative robot follower condition
                else:

                    # See if enough time has elapsed to change robot motion pattern
                    if time.time() - trial_start_time > elapsed_time:

                        # Increment hand contact count
                        rep_num = rep_num + 1

                        # Add elapsed period to the total elapsed time
                        elapsed_time += dummy_ints[rep_num]

                        # Update motion parameters if a contact was sensed in the last motion rep
                        motion_time = dummy_ints[rep_num]                            # update cycle time
                        factor = mult_factor * motion_time                            # update padding factor that allows for slowing down
                        engine.frequency = 1 / (motion_time + factor)                # update cycle frequency
                        for_amp = engine.calculateAmplitude(motion_time + factor)    # update motion amplitude
                        arm.beginSinusoidalMotion("right_w1")                        # start new sinusoidal motion cycle

    # Define robot end pose
    end_pos = {'right_s0': -0.8620972018066407, 'right_s1': 0.35665053277587894, 'right_w0': 1.1696603494262696, 'right_w1': 1.8593157223693849, 'right_w2': -0.02070874061279297, 'right_e0': 1.5132720455200197, 'right_e1': 1.9381847232788088}

    # Load happy ending image onto Baxter's face
    face.changeEmotion('Joy')

    # Move Baxter to end pose
    arm._right_limb.move_to_joint_positions(end_pos)

    plt.figure()
    plt.plot(arm._time, arm._data1, arm._time, arm._data2)
    plt.show()

if __name__ == "__main__":
    main()