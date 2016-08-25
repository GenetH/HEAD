#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image

from room_luminance.msg import Luminance
from cv_bridge import CvBridge, CvBridgeError
import roslib
import sys
import rospy
import cv2
import  numpy as np
import math

roslib.load_manifest('room_luminance')

'''
This Class contains the states and behaviours required to get the amount of light in the captured frame.
 '''
class ROIluminance:

 # initialize publishers, subscribers and static members in the class constructor
  def __init__(self):
    self.pub = rospy.Publisher('/opencog/room_luminance', Luminance, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.Visibility)
    self.count = 10
    self.ref = " "
    self.RefArea = 0
    self.tolerableCnts = 6
    self.totalArea = 0
    self.covUp = 75
    self.covDown = 95

  ''' BGR based: could be used for further requirements. i.e hsv would be more than enough for the current req.'''

  # def luminance_BGR(self, image_raw_bgr):
  #     #split into channels
  #     b, g, r = cv2.split(image_raw_bgr)
  #     size = np.size(image_raw_bgr)
  #
  #     #Get average of Blue, Green and Red pixels
  #     B = float(np.sum(b)) / size
  #     G = float(np.sum(g)) / size
  #     R = float(np.sum(r)) / size
  #
  #     # Photometric Luminance
  #     Y1 = 0.2126*R + 0.7152*G + 0.0722*B
  #
  #     # Perceived Luminance
  #     Y2 = 0.299*R + 0.587*G + 0.114*B
  #     return [Y1, Y2]


  ''' HSV based Room Luminance Detection '''
  def luminance_HSV(self, image_raw_hsv):
      h, s, v = cv2.split(image_raw_hsv)
      size = np.size(image_raw_hsv)

      #  optional: for other feature extration purpose
      # H = float(np.sum(h)) / size #range  0-360
      # S = float(np.sum(s)) / size #range  0- 100

      V = float(np.sum(v)) / size #range  0- 100
      return float(V)


  def objectBlock(self, image_raw):
    h, w = image_raw.shape
    self.totalArea = float(h * w)

    thresh = cv2.threshold(image_raw, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=10)

    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    TArea = 0
    for c in cnts:
        # Discard tiny objects/contours that usually does not have relevant if it is created by close enough object
        if cv2.contourArea(c) <= 100:
            pass
        self.RefArea = self.RefArea + cv2.contourArea(c)
    return math.ceil(((self.RefArea / self.totalArea) * 100))

  def classify(self, lumene, coverage):
    if lumene <= 25:
        return "Dark"
    elif lumene <= 55:
        if self.covUp <= coverage <=self.covDown:

            return "Dark"
        else:
            return "Nominal"
    else:
        return "Bright"


  def Visibility(self, data):

    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        h, s, v = cv2.split(hsv)


        if self.count % 10 == 0:
            self.ref = gray
            self.count = 1
        self.count += 1
        diff = cv2.absdiff(self.ref, gray)

        #get the average light on HSV image
        lumene = self.luminance_HSV(hsv)
        # get the percent of coverage by an object
        coverage = self.objectBlock(diff)

        self.RefArea = 0

        msg = Luminance()
        msg.brightness = self.classify(lumene, coverage)
        msg.coverage = coverage
        self.pub.publish(msg)
        # cv2.imshow("Room", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit(0)

    except CvBridgeError as e:
      print(e)


def main(args):
    rospy.init_node('perceived_luminance', anonymous=True)
    ROIluminance()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Luminance Detector Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)