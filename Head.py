import baxter_interface
import cv2
import cv_bridge
import random
import rospy

from sensor_msgs.msg import Image

class Head(object):
    def __init__(self, directory):
        self._head = baxter_interface.Head()

        self.directory = directory

        self.colors = ["Blue", "Gray", "Green", "Orange", "Purple", "Red", "White", "Yellow"]
        self.emotions = ["Afraid", "Confused", "Happy", "Joy", "Neutral", "Sad", "Sassy", "Silly", "SleepOpen", "SleepClosed", "Surpise", "Worried"]
        self.gazes = ["Blink", "SW", "SE", "NW", "NE"]

        self.color = "Blue"
        self.emotion = "Happy"
        self.gaze = "NW"

        self.faces = dict()

        self.start = rospy.get_rostime().to_sec()
        self.blinkTime = 0.0
        self.blinkLength = 0.0
        self.blinkWait = 0.0
        self.blinking = False

        self.headWait = 10
        self.headTime = rospy.get_rostime().to_sec()

        self._setupBlink()

        self.trackLeft = False
        self.trackRight = True
        self.leftPos = dict()
        self.rightPos = dict()

        self.switchTicks = 0.0
        self.switchTime = 0.0
        self.switchBack = ""

        self.cswitchTicks = 0.0
        self.cswitchTime = 0.0
        self.cSwitchBack = ""

        self.faceset = False

    def _setFace(self, imgpath):
        """Changes the robot's face to the specified image"""
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        if not imgpath in self.faces.keys():
            img = cv2.imread(imgpath)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
            self.faces[imgpath] = msg
            pub.publish(msg)
        else:
            msg = self.faces[imgpath]
            pub.publish(msg)

    def setAngle(self, angle):
        """Causes the head to move to the desired angle"""
        self._head.set_pan(angle)

    def nod(self):
        """Causes the head to nod"""
        self._head.command_nod()

    def sleep(self):
        self._setFace(self.directory + 'SleepClosed' + self.color + '.jpg')

    def sleepOpen(self):
        self._setFace(self.directory + 'SleepOpen' + self.color + '.jpg')

    def updateFace(self):
        self.faceset = True
        # if self.blinking:
        #     fn = self.directory + self.emotion + "Blink" + self.color + ".jpg"
        #     self._setFace(fn)
        # else:
        fn = self.directory + self.emotion + self.gaze + self.color + ".jpg"
        self._setFace(fn)

    def changeColor(self, color):
        if not self.color == color:
            self.faceset = False
            color = color.capitalize()
            if color in self.colors:
                self.color = color
                return True
        return False

    def switchColor(self, nextCo, default, ticks):
        self.changeColor(nextCo)
        self.cswitchTime = rospy.get_rostime().to_sec()
        self.cswitchTicks = ticks
        self.cswitchBack = default

    def changeEmotion(self, emotion):
        if not self.emotion == emotion:
            self.faceset = False
            emotion = emotion.capitalize()
            if emotion in self.emotions:
                self.emotion = emotion
                return True
        return False

    def switchEmotion(self, nextEm, default, ticks):
        self.changeEmotion(nextEm)
        self.switchTime = rospy.get_rostime().to_sec()
        self.switchTicks = ticks
        self.switchBack = default

    def changeGaze(self, gaze):
        '''
        Use this function to update Baxter gaze
        '''
        self.faceset = False
        gaze = gaze.capitalize()
        if gaze in self.gazes:
            self.gaze = gaze
            return True
        return False

    def _setupBlink(self):
        self.blinkTime = rospy.get_rostime().to_sec()
        self.blinkLength = random.uniform(0.1, 0.4) * 2
        self.blinkWait = random.uniform(1.8, 3.8) * 2

    def setLeftPos(self, leftpos):
        self.leftPos = leftpos
        self._updateGaze()

    def setRightPos(self, rightpos):
        self.rightPos = rightpos 
        self._updateGaze()

    def _updateGaze(self):
        if not (self.trackRight and self.trackLeft):
            if self.trackRight:
                if self.rightPos['right_s1'] < 0.0:
                    self.gaze = 'NW'
                else:
                    self.gaze = 'SW'
            elif self.trackLeft:
                if self.leftPos['left_s1'] < 0.0:
                    self.gaze = 'NE'
                else:
                    self.gaze = 'SE'
        else:
            total = (self.rightPos['right_s1'] + self.leftPos['left_s1']) / 2.0
            if total < 0.0:
                self.gaze = "N"
            else:
                self.gaze = "S"
            if random.randint(0, 1) == 0:
                self.gaze += "E"
            else:
                self.gaze += "W"

    def run(self, rate):
        pause = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.update()
            pause.sleep()

    def update(self):
        now = rospy.get_rostime().to_sec()
        # Handle gaze
        if self.blinking:
            if now - self.blinkTime > self.blinkLength:
                self.blinking = False
                self.blinkTime = now
                self._setupBlink()
                self.updateFace()
        else:
            if now - self.blinkTime > self.blinkWait:
                self.blinking = True
                self.blinkTime = now
                self.updateFace()
                
        if not self.switchTicks == 0.0:
            if now - self.switchTime > self.switchTicks:
                self.changeEmotion(self.switchBack)
                self.switchTicks = 0.0
                self.updateFace()

        if not self.cswitchTicks == 0.0:
            if now - self.cswitchTime > self.cswitchTicks:
                self.changeColor(self.cswitchBack)
                self.cswitchTicks = 0.0
                self.updateFace()


        if not self.faceset:
            self.updateFace()

        # Handle head motion
        if now - self.headTime > self.headWait:
            motion = 0.0
            if self.trackLeft and self.trackRight:
                motion = random.uniform(-0.5, 0.5)
            elif self.trackLeft:
                motion = random.uniform(0.0, 0.5)
            elif self.trackRight:
                motion = random.uniform(-0.5, 0.0)
            # Set Baxter head angle
            self.setAngle(-0.25)
            self.headTime = now
            self.headWait = random.uniform(20, 80)
        

