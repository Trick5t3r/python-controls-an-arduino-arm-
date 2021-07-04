# python-controls-an-arduino-arm-
Recently, for my TIPE, I created a python script that can communicate with an arduino arm and can recognize a ping pong ball and make the arm follow the ball with a laser. 


__1. Arduino code__
```robotv1.ino``` is the program to put in the Braccio Arduino

__2. Python code__
* ```testpythonarduino.py``` is a simple code I found on the internet freely to test to connect the arduino with pyhton code
* ```essaievideo_live_arduino.py``` is the program to follow the ball with the camera
* ```essaievideodoublecamerathread_live_arduino.py``` this is the goal of my tipe, the displacement of the arm towards a predictable position using a calculation of trajectory and parabola. 
