# -*- coding: utf-8 -*-
import socket
import time
import numpy as np
import cv2
 #setup socket
s = socket.socket()
host ='192.168.1.100'
port = 8006
s.bind((host,port))
s.listen(5)
#funciones
t = np.linspace(0,2*np.pi,100)
y0 = np.float16(np.sin(t))
y1 = np.float16(0.5*np.sin(t)*np.sin(2*t))
#cargamos la plantilla e inicializamos la webcam:
face_cascade = cv2.CascadeClassifier('cascade.xml')
cap = cv2.VideoCapture(1)
i=0 
while(True):
    #leemos un frame y lo guardamos
    ret, img = cap.read()
 
    #convertimos la imagen a blanco y negro
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
    #buscamos las coordenadas de los rostros (si los hay) y
    #guardamos su posicion
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    #Dibujamos un rectangulo en las coordenadas de cada rostro
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(125,255,0),2)
        print(x)
        print(y)
         
    #Mostramos la imagen
    cv2.imshow('img',img)
     
    #con la tecla 'q' salimos del programa
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    s.send("q")
    s.send('CH1'+y+'\n')
    i += 1
    if i>=99:
        i = 0
    time.sleep(0.01)      
cap.release()
cv2.destroyAllWindows()