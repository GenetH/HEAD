#!/usr/bin/env python
from sensor_msgs.msg import Image

from pi_face_tracker.msg import FaceEvent,Faces
from room_luminance.msg import Luminance
from cv_bridge import CvBridge, CvBridgeError
import roslib
import rospy
import cv2
import sys
import  numpy as np
import math
import time


roslib.load_manifest('room_luminance')

'''
This Class contains the states and behaviours required to get the amount of light in the captured frame and detects object blocks above N% of coverage.
 '''

class ROIluminance:

 # initialize publishers, subscribers and static members inside class constructor.
  def __init__(self):
    self.pub = rospy.Publisher('/opencog/room_luminance', Luminance, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.Visibility)
    self.face_event = rospy.Subscriber("/camera/face_locations", Faces, self.count_faces) # event = new_face: informs released/uncovered screen

    self.count = 25
    self.ref = ""
    self.RefArea = 0
    self.totalArea = 0
    self.covUp = 65
    self.covDown = 95
    self.face = 0



  '''
  BGR based: could be used for further requirements. i.e hsv would be more than enough for the current req.
  '''
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


  def objectBlock(self, image_raw, a):
    h, w = image_raw.shape
    self.totalArea = float(h * w)

    thresh = cv2.threshold(image_raw, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=10)

    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        # Discard tiny objects/contours that usually does not have relevance if it is created by close enough/blocking objects
        if cv2.contourArea(c) <= 100:
            pass
        self.RefArea = self.RefArea + cv2.contourArea(c)
    return math.ceil(((self.RefArea / self.totalArea) * 100))


  def classify(self, lumene):
    if lumene <= 25:
        return "Dark"
    elif lumene <= 40:
        return "Nominal"
    else:
        return "Bright"

  # callback
  def count_faces(self, face_state):
    self.face = len(face_state.faces)
      # self.Face_Event = face_state.face_event


  #callback
  def Visibility(self, data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if self.count % 25 == 0:
            self.ref = gray
            self.count = 1
        self.count += 1
        diff = cv2.absdiff(self.ref, gray)

        # get the luminance of HSV frame
        lumene = self.luminance_HSV(hsv)

        # get the percent of coverage by an object
        coverage = self.objectBlock(diff, self.ref)

        # msg.covered = iscovered(coverage)






        self.RefArea = 0
        msg = Luminance()


        msg.covered = 0
        msg.perc_covered = coverage

        msg.value = lumene
        msg.room_light = self.classify(lumene)

        self.pub.publish(msg)

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