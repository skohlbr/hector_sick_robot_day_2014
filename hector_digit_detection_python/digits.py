#!/usr/bin/env python

# roslaunch hector_digit_detection_python digit_detection.launch

'''
SVM and KNearest digit recognition.

Sample loads a dataset of handwritten digits from 'digits.png'.
Then it trains a SVM and KNearest classifiers on it and evaluates
their accuracy.

Following preprocessing is applied to the dataset:
 - Moment-based image deskew (see deskew())
 - Digit images are split into 4 10x10 cells and 16-bin
   histogram of oriented gradients is computed for each
   cell
 - Transform histograms to space with Hellinger metric (see [1] (RootSIFT))


[1] R. Arandjelovic, A. Zisserman
    "Three things everyone should know to improve object retrieval"
    http://www.robots.ox.ac.uk/~vgg/publications/2012/Arandjelovic12/arandjelovic12.pdf

Usage:
   digits.py
'''

from functools import partial

# built-in modules
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from hector_digit_detection_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError
from collections import Counter

from multiprocessing.pool import ThreadPool

import cv2
import numpy as np
from numpy.linalg import norm

# local modules
from common import clock, mosaic

SZ = 20 # size of each digit is SZ x SZ
CLASS_N = 10
DIGITS_FN = 'data/digits.png'

def load_print_digits(fp):
	labels = np.repeat(np.arange(9), 9) + 1
	digits = []
	for i in range(1,10):
		for j in range(0,9):
                        digit_img = cv2.imread('%s/%d_%d.jpg' % (fp,i,j) , 0)
			digit_img2 = cv2.bitwise_not(digit_img)
			digits.append(digit_img2)
	return digits, labels


def split2d(img, cell_size, flatten=True):
    h, w = img.shape[:2]
    sx, sy = cell_size
    cells = [np.hsplit(row, w//sx) for row in np.vsplit(img, h//sy)]
    cells = np.array(cells)
    if flatten:
        cells = cells.reshape(-1, sy, sx)
    return cells

def load_digits(fn):
    print 'loading "%s" ...' % fn
    digits_img = cv2.imread(fn, 0)
    digits = split2d(digits_img, (SZ, SZ))
    labels = np.repeat(np.arange(CLASS_N), len(digits)/CLASS_N)
    return digits, labels

def deskew(img):
    m = cv2.moments(img)
    if abs(m['mu02']) < 1e-2:
        return img.copy()
    skew = m['mu11']/m['mu02']
    M = np.float32([[1, skew, -0.5*SZ*skew], [0, 1, 0]])
    img = cv2.warpAffine(img, M, (SZ, SZ), flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR)
    return img

class StatModel(object):
    def load(self, fn):
        self.model.load(fn)
    def save(self, fn):
        self.model.save(fn)

class KNearest(StatModel):
    def __init__(self, k = 3):
        self.k = k
        self.model = cv2.KNearest()

    def train(self, samples, responses):
        self.model = cv2.KNearest()
        self.model.train(samples, responses)

    def predict(self, samples):
        retval, results, neigh_resp, dists = self.model.find_nearest(samples, self.k)
        return results.ravel()

class SVM(StatModel):
    def __init__(self, C = 1, gamma = 0.5):
        self.params = dict( kernel_type = cv2.SVM_RBF,
                            svm_type = cv2.SVM_C_SVC,
                            C = C,
                            gamma = gamma )
        self.model = cv2.SVM()

    def train(self, samples, responses):
        self.model = cv2.SVM()
        self.model.train(samples, responses, params = self.params)

    def predict(self, samples):
        return self.model.predict_all(samples).ravel()

def evaluate_model(model, digits, samples, labels):
    resp = model.predict(samples)
    err = (labels != resp).mean()
    print 'error: %.2f %%' % (err*100)

    confusion = np.zeros((10, 10), np.int32)
    for i, j in zip(labels, resp):
        confusion[i, j] += 1
    print 'confusion matrix:'
    print confusion
    print

    vis = []
    for img, flag in zip(digits, resp == labels):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if not flag:
            img[...,:2] = 0
        vis.append(img)
    return mosaic(25, vis)

def preprocess_simple(digits):
    return np.float32(digits).reshape(-1, SZ*SZ) / 255.0

def preprocess_hog(digits):
    samples = []
    for img in digits:
        gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
        mag, ang = cv2.cartToPolar(gx, gy)
        bin_n = 16
        bin = np.int32(bin_n*ang/(2*np.pi))
        bin_cells = bin[:10,:10], bin[10:,:10], bin[:10,10:], bin[10:,10:]
        mag_cells = mag[:10,:10], mag[10:,:10], mag[:10,10:], mag[10:,10:]
        hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in zip(bin_cells, mag_cells)]
        hist = np.hstack(hists)

        # transform to Hellinger kernel
        eps = 1e-7
        hist /= hist.sum() + eps
        hist = np.sqrt(hist)
        hist /= norm(hist) + eps

        samples.append(hist)
    return np.float32(samples)

def imageCallback(svmmodel,data):
	print 'in callback! \n'
        #print 'received image of type: "%s"' % data.format

        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(data.data, np.uint8)
        #image_np = cv2.imdecode(data.data, cv2.CV_LOAD_IMAGE_GRAYSCALE)

        print '---'
        print type(data)
        print '---'
        print type(data.data)
        print '---'
        print data.data.encoding
        print '---'

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data.data)
        #cv2.imshow('input image', cv_image)
        #cv2.waitKey(0)

        print type(cv_image)
        print '---'
        #print "size = %d,%d" % (cv_image.size().height, cv_image.size().width)

        image_np = cv_image

        image_np = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        image_np2 = cv2.bitwise_not(image_np)
	image_np3 = cv2.adaptiveThreshold(image_np2, 255, 1, 1, 11, 2)
	image_np4 = image_np3
        contours, hierarchy = cv2.findContours(image_np4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        values = []

	for cnt in contours:
		if cv2.contourArea(cnt) > 100:
			x,y,w,h = cv2.boundingRect(cnt)
			if h > 50 and w > 30:
				cv2.rectangle(image_np, (x,y), (x+w, y+h), (0,255,0), 2)
				hi,wi = image_np.shape[:2]
				#print '%d %d | %d %d at size %d %d' % (x, x+w, y, y+w,wi,hi)
				#cv2.imshow('test', image_np)
				#cv2.waitKey(0)


				roi = image_np2[y:y+h,x:x+w]
				#roi = cv2.bitwise_not(roi)

				cv2.imshow('test', roi)
				cv2.waitKey(0)
				digit2 = deskew(roi)
				samples = preprocess_hog([digit2])
				resp = model.predict(samples)
				print resp
                                values.append(resp)

				#cv2.imshow('all', roi)
				#cv2.imshow('single', digit2)
				#cv2.waitKey(0)

        return values[0], 0.0

def image2digit_service(svmmodel, data):
	print 'in service! \n'
        #print 'received image of type: "%s"' % data.format
        return imageCallback(svmmodel, data)


if __name__ == '__main__':
	try:
		print __doc__

                #digits, labels = load_digits(DIGITS_FN)

                #print 'preprocessing...'
                #shuffle digits
                #rand = np.random.RandomState(321)
                #shuffle = rand.permutation(len(digits))
                #digits, labels = digits[shuffle], labels[shuffle]

                #digits2 = map(deskew, digits)
                #samples = preprocess_hog(digits2)

                #train_n = int(0.9*len(samples))
                #cv2.imshow('test set', mosaic(25, digits[train_n:]))
                #digits_train, digits_test = np.split(digits2, [train_n])
                #samples_train, samples_test = np.split(samples, [train_n])
                #labels_train, labels_test = np.split(labels, [train_n])

                #print 'training KNearest...'
                #model = KNearest(k=4)
                #model.train(samples_train, labels_train)
                #vis = evaluate_model(model, digits_test, samples_test, labels_test)
                #cv2.imshow('KNearest test', vis)

                #print 'training SVM...'
                #model = SVM(C=2.67, gamma=5.383)
                #model.train(samples_train, labels_train)
                #vis = evaluate_model(model, digits_test, samples_test, labels_test)
                #cv2.imshow('SVM test', vis)
                #print 'saving SVM as "digits_svm.dat"...'
                #model.save('digits_svm.dat')
                #cv2.waitKey(0)

                rospy.init_node('digit_detection_python', anonymous=True)

                fp = rospy.get_param('~img_path')
                print fp

                print 'loading Data ...'
                digits, labels = load_print_digits(fp)
		digits2 = map(deskew, digits)
		samples = preprocess_hog(digits2)
		model = SVM(C=2.67, gamma=5.383)

                print 'training SVM ...'
		model.train(samples, labels)

		bound_svmmodel_2_imageCallback = partial(imageCallback, model)

                # rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, bound_svmmodel_2_imageCallback, queue_size = 1)

		bound_svmmodel_2_image2digit_service = partial(image2digit_service, model)

		s = rospy.Service('image2digit', Image2Digit, bound_svmmodel_2_image2digit_service)

		rospy.spin()

	except rospy.ROSInterruptException: pass
