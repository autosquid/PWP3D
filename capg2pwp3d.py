# coding:utf-8

import hammer


srcroot = "C:/Users/Justin/workspace/remincline/data/fan/HuaweiData/data/mask"

datum = ["SJ31", "SJ41", "SJ51", "SJ22", "SJ32", "SJ23", "SJ33", "SJ53", "SJ34", "SJ54"]

dstroot = "C:/Users/Justin/workspace/PWP3D/Files/fan"


# HistMask
# HistSrc
# Images
# Others

import cv2
import os
from shutil import copyfile


for d in datum:
	srcfiles = hammer.listImages(os.path.join(srcroot, d))
	dires = ["HistSrc", "Images", "HistMask", "Others"]
	for dd in dires:
		hammer.makedir_p(os.path.join(dstroot, dd, d))

	dstfiles1 = [os.path.join(dstroot, "HistSrc", d, os.path.basename(f)) for f in srcfiles]
	dstfiles2 = [os.path.join(dstroot, "Images", d, os.path.basename(f)) for f in srcfiles]
	dstmaskfiles = [os.path.join(dstroot, "HistMask", d, os.path.basename(f)) for f in srcfiles]
	for f, df1, df2, df3 in zip(srcfiles, dstfiles1, dstfiles2, dstmaskfiles):
		print "=========="
		print (f)
		im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
		im3 = cv2.merge([im, im, im])
		cv2.imwrite(df1, im3)
		cv2.imwrite(df2, im3)
		cv2.imwrite(df3, im)

	srctargetmask =  os.path.join(dstroot, "Others", "SJ21", "targetmask.bmp")
	targetfile =  os.path.join(dstroot, "Others", d, "targetmask.bmp")

	copyfile(srctargetmask, targetfile)





