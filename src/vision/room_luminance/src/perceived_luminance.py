#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import roslib
import sys
import rospy
import cv2
import  numpy as np

roslib.load_manifest('room_luminance')

'''
This Class contains the states and behaviours required to get the amount of light in ROI.
 '''
class ROIluminance:
 #put publishers and subscribers in the ROIluminance constractor
  def __init__(self):
    self.pub = rospy.Publisher('/opencog/room_luminance', String, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.Visibility)

  '''
  # BGR based: for optionl use
  def luminance_BGR(self, image_raw_bgr):
      #split into channels
      b, g, r = cv2.split(image_raw_bgr)
      size = np.size(image_raw_bgr)

      #Get average of Blue, Green and Red pixels
      B = float(np.sum(b)) / size
      G = float(np.sum(g)) / size
      R = float(np.sum(r)) / size

      # Photometric Luminance
      Y1 = 0.2126*R + 0.7152*G + 0.0722*B

      # Perceived Luminance
      Y2 = 0.299*R + 0.587*G + 0.114*B
      return [Y1, Y2]
  '''
  # HSV based: especially Luminance (V) dependent
  def luminance_HSV(self, image_raw_hsv):
      h, s, v = cv2.split(image_raw_hsv)
      size = np.size(image_raw_hsv)
      #  optional: for other feature ext
      # H = float(np.sum(h)) / size #range  0-360
      # S = float(np.sum(s)) / size #range  0- 100

      V = float(np.sum(v)) / size #range  0- 100
      # print (V)
      return float(V)


  def classify(self, lumene):
    if lumene <= 25:
        return "Dark"
    elif lumene <= 55:
        return "Nominal"
    else:
        return "Bright"

  def HandbBock(self, image_raw):
    gray = cv2.cvtColor(image_raw, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5,5), 0)
    edged = cv2.Canny(gray, 35, 125)

    (cnts, _) =cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c =max(cnts, key = cv2.contourArea)
    print (c)
    cv2.imshow("Aread", edged)


  def Visibility(self, data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        # print (self.luminance_BGR(cv_image))

        lumene = self.luminance_HSV(hsv)
        self.HandbBock(cv_image)



        self.pub.publish(self.classify(lumene))


        # cv2.imshow("Room",cv_image)
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
