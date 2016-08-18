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

  def __init__(self):
    self.pub = rospy.Publisher('/opencog/room_luminance', String, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.getLightstate)



  # other lumene extractor
  def luminancewbgr(self, data2):
      b, g, r = cv2.split(data2)
      B = float(np.sum(b)) / np.size(data2)
      G = float(np.sum(g)) / np.size(data2)
      R = float(np.sum(r)) / np.size(data2)

      # relative luminance
      Y = 0.2126*R + 0.7152*G + 0.0722*B   # photometric /digital ITU BT. 709
      print (Y)
      # perceived luminance
      Y = 0.299*R + 0.587*G + 0.114*B
      print(Y, "--")


  #hsv
  def luminancewhsv(self, data2):
      h, s, v = cv2.split(data2)
      H = float(np.sum(h)) / np.size(data2)
      S = float(np.sum(s)) / np.size(data2)
      V = float(np.sum(v)) / np.size(data2)
      print( "-----------------", V)


      # relative luminance
      # Y = 0.2126*H + 0.7152*G + 0.0722*B   # photometric /digital ITU BT. 709
      # print (Y)
      # perceived luminance
      # Y = 0.299*R + 0.587*G + 0.114*B
      # print(Y, "--")


  # Callback
  def getLightstate(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)


        # will be used to check for side blocks; L and R blocks
        h, w = cv_image.shape[:2]
        gray2 = gray[0:h,0:w/2]
        gray3 = gray[0:h, w/2:w]

        decision = "IsDark? "
        self.luminancewbgr(cv_image)

        #HSV checkuhp

        self.luminancewhsv(hsv)


        self.pub.publish(decision)

        cv2.imshow("Room",cv_image)

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
