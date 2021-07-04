# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 21:24:48 2021

@author: Spyder
https://pythonforundergradengineers.com/python-arduino-LED.html
"""
import serial
from time import sleep, time
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
print("enter exit to exit")
continu_de_control = True
while continu_de_control:
    command = input("Quelles valeurs donnees :")
    if command == "exit":
        break
    lCommand = command.split(":")
    vM1 = lCommand[0]
    vM2 = lCommand[1]
    vM3 = lCommand[2]
    vM4 = lCommand[3]
    vM5 = lCommand[4]
    #vM6 = lCommand[5]
    my_str = str(vM1) + ":" + str(vM2) + ":" + str(vM3) + ":" + str(vM4) + ":" + str(vM5) + "\n"
    my_str_as_bytes = my_str.encode("utf-8")
    ser.write(my_str_as_bytes)
ser.close()