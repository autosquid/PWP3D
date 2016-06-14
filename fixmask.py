# coding:utf-8

import hammer

datum = ["SJ21", "SJ31", "SJ41", "SJ51", "SJ22", "SJ32", "SJ23", "SJ33", "SJ53", "SJ34", "SJ54"]

srcroot = "C:/Users/Justin/workspace/PWP3D/Files/fan"

# HistMask
# HistSrc
# Images
# Others

import cv2
import os
from shutil import copyfile


for d in datum:
	srcfiles = hammer.listImages(os.path.join(srcroot, "HistMask", d))
	print "=========="
	
	for f in srcfiles:
		print (f)
		im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
		ret, nim = cv2.threshold(im, 1, 1, cv2.THRESH_BINARY)
		cv2.imwrite(f, nim)




