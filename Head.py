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
        self.emotions = ["Afraid", "Confused", "Happy", "Joy", "Neutral", "Reactive", "Sad", "Sassy", "Silly", "SleepOpen", "SleepClosed", "Surpise", "Worried"]
        self.gazes = ["Blink", "SW", "SE", "NW", "NE"]
        self.name = ""

        self.color = "Blue"
        self.emotion = "Happy"
        self.gaze = "NW"
        self.name = "BlueNWHappy"

        self.faces = dict()

        self.start = rospy.get_rostime().to_sec()
        self.blinkTime = 0.0
        self.blinkLength = 0.0
        self.blinkWait = 0.0
        self.blinking = False

        self.headWait = 10
        self.headTime = rospy.get_rostime().to_sec()

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

    def updateFace(self):
        self.faceset = True
        fn = self.directory + self.name + ".jpg"
        self._setFace(fn)

    def changeEmotion(self, newname):
        self.faceset = False
        self.switchTicks = 0.0
        self.name = newname

    def switchEmotion(self, newname, defaultname, ticks):
        self.name = newname
        self.faceset = False
        self.switchTime = rospy.get_rostime().to_sec()
        self.switchTicks = ticks
        self.switchBack = defaultname

    def run(self, rate):
        pause = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.update()
            pause.sleep()

    def update(self):
        now = rospy.get_rostime().to_sec()
                
        if not self.switchTicks == 0.0:
            if now - self.switchTime > self.switchTicks:
                self.name = self.switchBack
                self.switchTicks = 0.0
                self.updateFace()
                print 'yup'

        if not self.faceset:
            self.updateFace()
            print 'yup'

        # Run the following if the head is in the wrong position
        #self.setAngle(-0.3)
        

