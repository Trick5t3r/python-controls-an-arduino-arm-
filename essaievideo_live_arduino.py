# -*- coding: utf-8 -*-
"""
Created on Thu Feb 11 21:06:55 2021

@author: Spyder
"""
import numpy as np
import cv2
from time import time, sleep
import serial
vM1 = 0
vM2 = 90
vM3 = 90
vM4 = 90
vM5 = 90
vM6 = 90

ser = serial.Serial('COM4', 2000000, timeout=1)
lasttime = time()
while time()-lasttime < 30:
    data = ser.read()
    if data:
        break

print("Connected")

def traiteimg(img):
    """ traite l'image pour qu elle puisse etre annalysee"""
    answer = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return answer

def getRedimage(img):
    # extract red channel
    red_channel = img[:,:,2]
    #answer = cv2.cvtColor(red_channel, cv2.COLOR_BGR2GRAY)
    return red_channel

def dessinecerclesurimage(output, circles, stay=True):
    """ dessine le cercle(x, y, raidus) sur l'image"""
    if circles is not None and circles.all() != None:
        circles = np.round(circles).astype("int")[0]
    	# loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            #print([x, y, r])
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 1, y - 1), (x + 1, y + 1), (0, 128, 255), -1)
            # show the output image
            height = int(output.shape[0])
            put_arduino_to_centerballe(x, height-y)
        if stay:
            cv2.imshow("output", np.hstack([output]))
            #cv2.waitKey(0)

def envoie_message_arduino(message):
    my_str_as_bytes = message.encode("utf-8")
    ser.write(my_str_as_bytes)
    
def positionne_arduino(vM1, vM2, vM3, vM4, vM5, vM6):
    my_str = str(vM1) + ":" + str(vM4) + ":" + str(vM5) + "\n"
    
    envoie_message_arduino(my_str)
    sleep(1)
    
    
def put_arduino_to_centerballe(coordX, coordY, coordZ=1):
    angleabscisse, angleordonne = calculeanglescentral(coordX, coordY, coordZ, 640, 0, 360)
    shot = 0
    vM1 = (angleabscisse)/np.pi * 180 +90
    vM2 = 90
    vM3 = 90 #ou 180
    vM4 = (angleordonne)/np.pi * 180 + 90
    vM5 = shot
    vM6 = 0
    #print(vM1, vM4)
    positionne_arduino(vM1, vM2, vM3, vM4, vM5, vM6)

def calculeanglescentral(coordX, coordY, coordZ, largeurdelimage, profondeurimage, hauteurdelimage):
    newcoordX = coordX - largeurdelimage/2
    coordonneeReelleX = newcoordX/largeurdelimage * 0.3
    newcoordZ = coordZ + profondeurimage
    newcoordY = coordY - hauteurdelimage/2
    coordonneeReelleY = newcoordY/hauteurdelimage * 0.7
    #print(newcoordX, newcoordY)
    #deltaangle = np.arctan(largeurdelimage/(2*newcoordZ))
    angleabscisse = np.arctan(coordonneeReelleX/newcoordZ)
    angleordonne = np.arctan(coordonneeReelleY/np.sqrt(newcoordZ**2+coordonneeReelleX**2))
    
    #angleabscisse /= deltaangle
    return angleabscisse, angleordonne

#################

def getcenterpingpong(img):
    """ retourne le centre du premier cercle trouve (il faut donc qu il n y est qu un seul cercle"""
    # detect circles in the image
    circles = cv2.HoughCircles(traiteimg(img), cv2.HOUGH_GRADIENT, 2, 5000)
    #print(circles)
    # ensure at least some circles were found
    if circles is not None:
        #print(circles)
        dessinecerclesurimage(img, circles)

cap = cv2.VideoCapture(2)
while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape)
    # Our operations on the frame come here
    #frame = getRedimage(frame)
    getcenterpingpong(frame)
    
    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
ser.close()