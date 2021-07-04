# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 22:48:51 2021

@author: Spyder
"""
import numpy as np
import cv2
from time import time, sleep
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
from math import sqrt

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


starttime = time()
#tempsmesure = 0.03#0.05 # secondes a attendre entre chaque mesure
#tempsavantpremieremesure = 0.01#0.05 #secondes temps a attendre avant la premiere mesure
nbmesures = 3
tempspreshot = 1 #en secondes = au temps que la machine met pour tourner

### pour rendre agreable
def remetalabonnetaille(imag):
    """ resize l image pour quelle puisse rentree sur mon ecran """
    #percent by which the image is resized
    scale_percent = 50
    
    #calculate the 50 percent of original dimensions
    width = int(imag.shape[1] * scale_percent / 100)
    height = int(imag.shape[0] * scale_percent / 100)
    
    # dsize
    dsize = (width, height)
    
    # resize image
    return cv2.resize(imag, dsize)
###


def dessineparabole(output, cercles, ltimes, name="output"):
    """ dessine la parabole sur l image de sorti"""
    abscisse, parabolev1 = modeliseparabole(cercles, ltimes)
    curve = np.column_stack((abscisse.astype(np.int32), parabolev1.astype(np.int32)))
    cv2.polylines(output, [curve], False, (0,255,255))
    dessinecerclesurimage(output, cercles, name=name)

def dessinecerclesurimage(output, circles, stay=True, name="output"):
    """ dessine le cercle(x, y, raidus) sur l'image"""
    if circles is not None and circles.all() != None:
        circles = np.round(circles).astype("int")
    	# loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            #print([x, y, r])
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 1, y - 1), (x + 1, y + 1), (0, 128, 255), -1)
            # show the output image
        if stay:
            cv2.imshow(name, np.hstack([output]))
            #cv2.waitKey(0)
    
#################

def getcenterpingpong(img):
    """ retourne le centre du premier cercle trouve (il faut donc qu il n y est qu un seul cercle"""
    # detect circles in the image
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 2, 5000)
    # ensure at least some circles were found
    height = int(img.shape[0])
    if circles is not None:
        return  np.abs(np.array([0, height, 0]) - circles[0]) #permet de mettre la parabole dans le bon sens #circles[0] #si on veut que ca corresponde a limage quand on dessine sur limage
    return np.array([None])

def traiteimg(img):
    """ traite l'image pour qu elle puisse etre annalysee"""
    # extract red channel
    #img = remetalabonnetaille(img)
    #red_channel = img[:,:,2]
    answer = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return answer#red_channel #answer

def calculcentreballdelimg(img):
    """ calcul le centre de la balle de pingpong en traitant l image et en recuperant le centre"""
    tempimg = traiteimg(img)
    return getcenterpingpong(tempimg)




def modeliseparabole(cercles, ltimes):
    """ modelise la parabole basee sur les point des centres des cercles et du temps pouyr les atteindre"""
    
    X = cercles[:, 0]
    Y = cercles[:, 1]
    
    #fitx = np.polyfit(ltimes, X, 2)
    #fity = np.polyfit(ltimes, Y, 2)
    
    fitx = np.polyfit(X, Y, 2)
    
    t = np.arange(X.min(), X.max())
    abscisse = t #X
    parabolev1 = fitx[0] * abscisse**2 + fitx[1] * abscisse + fitx[2]
    
    return abscisse, parabolev1


def modeliseparabole3d(cerclesx, ltimesx, cerclesz, ltimesz, angleEntre2plans = np.pi/4):
    """ modelise la parabole basee sur les point des centres des cercles et du temps pouyr les atteindre"""
    X = cerclesx[:, 0]
    Yx = cerclesx[:, 1]
    Z = cerclesz[:, 0] * np.sin(angleEntre2plans)
    
    #fitx = np.polyfit(ltimes, X, 2)
    #fity = np.polyfit(ltimes, Y, 2)
    
    fitx = np.polyfit(ltimesx, X, 2)
    fity = np.polyfit(ltimesx, Yx, 2)
    fitz = np.polyfit(ltimesz, Z, 2)
    
    t = np.linspace(ltimesx.min(), ltimesx.max(), 30)
    abscisse = t #X
    parabolevx = fitx[0] * abscisse**2 + fitx[1] * abscisse + fitx[2]
    parabolevy = fity[0] * abscisse**2 + fity[1] * abscisse + fity[2]
    parabolevz = fitz[0] * abscisse**2 + fitz[1] * abscisse + fitz[2]
    
    return abscisse, parabolevx, parabolevy, parabolevz

def calculpreshot(cerclesx, ltimesx, cerclesz, ltimesz, tempspreshot, angleEntre2plans = np.pi/6):
    """ modelise la parabole basee sur les point des centres des cercles et du temps pouyr les atteindre"""
    X = cerclesx[:, 0]
    Yx = cerclesx[:, 1]
    Z = cerclesz[:, 0] * np.sin(angleEntre2plans)
        
    fitx = np.polyfit(ltimesx, X, 1)
    fity = np.polyfit(ltimesx, Yx, 2)
    fitz = np.polyfit(ltimesz, Z, 1)
    
    abscisse = tempspreshot
    coordX = fitx[0] * abscisse + fitx[1]
    coordY = fity[0] * abscisse**2 + fity[1] * abscisse + fity[2]
    coordZ = fitz[0] * abscisse + fitz[1]
    
    return coordX, coordY, coordZ

##########



def faitunemesure(cap, cercles, ltimes):
    ret, frame = cap.read()
    currentCercle = calculcentreballdelimg(frame)
    while currentCercle.any() == None:
        ret, frame = cap.read()
        currentCercle = calculcentreballdelimg(frame)
    if cercles.all() != None:
        cercles = np.concatenate((cercles, currentCercle))
        ltimes = np.append(ltimes, time()-starttime)
    else:
        cercles = currentCercle
        ltimes = np.array(time()-starttime)
    #cv2.imshow('frame' + str(time()),frame)
    return cercles, ltimes

def videodraw(cap, cercles, ltimes, name="output"):
    """ dessine les trois cercles plus la parabole sur une nouvelle image"""
    ret, frame = cap.read()
    #dessinecerclesurimage(frame, cercles)
    timetemporaire = time()
    while (not ret) and (time()-timetemporaire < 5):
        ret, frame = cap.read()
    dessineparabole(frame, cercles, ltimes, name)





class MonFluxVideo(threading.Thread):
    def __init__(self, threadID, name, tempsmesure, tempsavantpremieremesure=0):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.tempsmesure = tempsmesure
        self.tempsavantpremieremesure = tempsavantpremieremesure
        self.cercles = np.array([None])
        self.ltimes = np.array([None])
        self.lock = threading.Lock()
        self.lastime = 0
        self.nbmesures = 3
        self.counter = 1
        
    def run(self):
        self.cap = cv2.VideoCapture(int(self.name))
        self.lastime = time()
        print("shot")
        while True:
            if time() - self.lastime >= self.tempsavantpremieremesure:
                ret, frame = self.cap.read()
                if ret:            
                    #on attend quelques secondes avant la premiere mesure
                    #self.lock.acquire()
                    self.cercles, self.ltimes = faitunemesure(self.cap, self.cercles, self.ltimes)
                    #self.lock.release()
                    break
            
        self.lastime = time()
        while self.counter < self.nbmesures:
            if time() - self.lastime >= self.tempsmesure:
                ret, frame = self.cap.read()
                if ret:     
                    print("shot")
                    #self.lock.acquire()
                    self.lastime = time()
                    self.cercles, self.ltimes = faitunemesure(self.cap, self.cercles, self.ltimes)
                    self.counter += 1
                    #self.lock.release()
        
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

        

fluxFrontale = MonFluxVideo(0, 0, 0.1, 1)
fluxCote = MonFluxVideo(2, 2, 0.1, 1)
#fluxFrontale = MonFluxVideo(1, 'videofrontale.mp4', 0.01, 0.03)
#fluxCote = MonFluxVideo(2, 'video45degre.mp4', 0.02, 0.04)

# on lance l analyse en parallele des videos
fluxFrontale.start()
fluxCote.start()

# on attend que les deux annalyses soient termines
fluxFrontale.join()
fluxCote.join()

cap = cv2.VideoCapture(0)
videodraw(cap, fluxFrontale.cercles, fluxFrontale.ltimes, name="0")
cap.release()
cap = cv2.VideoCapture(2)
# Deuxieme camera a besoin de 3 prises dimages pour sactiver
videodraw(cap, fluxCote.cercles, fluxCote.ltimes, name="1")
cap.release()



abscisse, parabolevx, parabolevy, parabolevz = modeliseparabole3d(fluxFrontale.cercles, fluxFrontale.ltimes, fluxCote.cercles, fluxCote.ltimes)

time_start_calcul_preshot = time()-starttime

coordX, coordY, coordZ = calculpreshot(fluxFrontale.cercles, fluxFrontale.ltimes, fluxCote.cercles, fluxCote.ltimes, time_start_calcul_preshot+ tempspreshot)

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
# Test
put_arduino_to_centerballe(coordX, coordY, coordZ)


ser.close()

#### matplotlib
fig = plt.figure()
ax = fig.gca(projection='3d')  # Affichage en 3D
ax.plot(parabolevx, parabolevz, parabolevy, label='Courbe')  # Tracé de la courbe 3D
plt.title("Courbe 3D")
ax.set_xlabel('X')
ax.set_ylabel('Z')
ax.set_zlabel('Y')
plt.tight_layout()
plt.show()

cv2.waitKey(0)


cv2.destroyAllWindows()

print("frontale", fluxFrontale.ltimes)
print("cote", fluxCote.ltimes)
