#!/usr/bin/env python
import numpy as np
import cv2 as cv
face_cascade = cv.CascadeClassifier('cascade.xml')
eye_cascade = cv.CascadeClassifier('cascade.xml')
img = cv.imread('p3.jpeg')
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
faces = face_cascade.detectMultiScale(gray, 1.3, 5)
while(true):
	for (x,y,w,h) in faces:  
	    roi_gray = gray[y:y+h, x:x+w]
	    roi_color = img[y:y+h, x:x+w]
	    eyes = eye_cascade.detectMultiScale(roi_gray)
	    for (ex,ey,ew,eh) in eyes:
	        cv.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
	print(x)
	print(y)
	cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
	cv.imshow('img',img)
	cv.waitKey(0)
cv.destroyAllWindows()