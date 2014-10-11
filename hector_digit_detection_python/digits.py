#!/usr/bin/env python

import cv2
import numpy
import operator
import rospy
import subprocess

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from hector_digit_detection_msgs.srv import *

pub = rospy.Publisher('super_image', Image, queue_size=10)
target_height = 32.0
fp = ''

image1 = numpy.zeros((1,1,3), numpy.uint8)
image3 = numpy.zeros((1,1,3), numpy.uint8)

def image2digit_service(data):
    #print 'Executing image2digit service ...'

    bridge = CvBridge()
    image2 = bridge.imgmsg_to_cv2(data.data)
    if len(image2.shape) > 2 and image2.shape[2] > 2:
        image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

    h2,w2 = image2.shape[:2]
    image2 = cv2.resize(image2, (int(w2 * (target_height / h2)), int(target_height)))

    h2,w2 = image2.shape[:2]
    image2 = cv2.adaptiveThreshold(image2, 255, 1, cv2.THRESH_BINARY, 11, 2)

    image2_contours = cv2.bitwise_not(image2)
    contours, _ = cv2.findContours(image2_contours, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    moments = map(cv2.moments, contours)
    pairs = filter(lambda (mmnt,_): mmnt['m00'] != 0, zip(moments, contours))
    if len(pairs) > 0:
        moments,contours = zip(*pairs)
    else:
        moments,contours = [],[]

    centroids = map(lambda mmnt: (mmnt['m10']/mmnt['m00'], mmnt['m01']/mmnt['m00']), moments)

    hy,hx = image2.shape[:2]
    cx = hx / 2
    cy = hy / 2

    distances = map(lambda ctr: numpy.linalg.norm((cx-ctr[0],cy-ctr[1])), centroids)
    if len(distances) > 0:
        mini, mind = min(enumerate(distances), key = operator.itemgetter(1))
        # print 'Connected component with min distance %d %f' %  (mini, mind)
        # print contours[mini]
        x,y,w,h = cv2.boundingRect(contours[mini])
        image2 = image2[y:y+h,x:x+w]

    h2,w2 = image2.shape[:2]
    image2_whitemargin = numpy.zeros((h2+6,w2+5,1), numpy.uint8)
    image2_whitemargin = cv2.bitwise_not(image2_whitemargin)
    image2_whitemargin[2:h2+2,:w2] = image2

    image2 = cv2.resize(image2_whitemargin, (int(w2 * (target_height / h2)), int(target_height)))

    h1,w1 = image1.shape[:2]
    h2,w2 = image2.shape[:2]
    h3,w3 = image3.shape[:2]

    image_full = numpy.zeros((h2,w1+w2+w3,3), numpy.uint8)

    image_full[:h1, :w1, :3] = image1
    image_full[:h2,w1:w1+w2, 0] = image2
    image_full[:h2,w1:w1+w2, 1] = image2
    image_full[:h2,w1:w1+w2, 2] = image2
    image_full[:h3,w1+w2:w1+w2+w3, :3] = image3

    cv2.imwrite(fp + '/img.jpg', image_full)
    args = ('tesseract', fp + '/img.jpg', fp + '/tessout', '-l', 'eng', '-psm', '7')

    #print 'Launching tesseract ...'
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()

    #print 'Determing result ...'
    data = ''
    with open (fp + '/tessout.txt', 'r') as myfile:
        data = myfile.read()

    digit = -1
    if len(data) >= 7:
        try:
            digit = int(data[6])
        except ValueError:
            #print 'ord(%s) = %d' % (data[6], ord(data[6]))
		if data[6] == 'O':
			digit = 0
                elif len(data) >= 8:
			try:
				digit = int(data[7])
			except ValueError:
				digit = -1

    # digit threshold in [1,4]
    if digit > 4 or digit < 1:
        digit = -1

    #print 'Data = %sDigit = %d' % (data, digit)
    return digit, 0.0

if __name__ == '__main__':
    try:
        rospy.init_node('digit_detection_python', anonymous=True)
        fp = rospy.get_param('~img_path')

        image1 = cv2.imread(fp + '/hello.jpg', 1)
        h,w = image1.shape[:2]
        image1 = cv2.resize(image1, (int(w * (target_height / h)), int(target_height)))

        image3 = cv2.imread(fp + '/world.jpg', 1)
        h,w = image3.shape[:2]
        image3 = cv2.resize(image3, (int(w * (target_height / h)), int(target_height)))

        s = rospy.Service('image2digit', Image2Digit, image2digit_service)

        rospy.spin()
    except rospy.ROSInterruptException: pass
