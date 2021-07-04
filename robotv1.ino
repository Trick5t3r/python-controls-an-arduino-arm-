#include <Braccio.h>
#include <Servo.h>


Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

void setup() {
  Serial.begin(2000000);
  //Serial.print("Start...\n");
  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:
  //Base (M1):90 degrees
  //Shoulder (M2): 45 degrees
  //Elbow (M3): 180 degrees
  //Wrist vertical (M4): 180 degrees
  //Wrist rotation (M5): 90 degrees
  //gripper (M6): 10 degrees
  Braccio.begin();
  
  int vM1 = 90;
  int vM2 = 45;
  int vM3 = 180;
  int vM4 = 180;
  int vM5 = 90;
  int vM6 = 0;
  //Braccio.ServoMovement(40, vM1, vM2, vM3, vM4, vM5, vM6); 
  Serial.print("Had Set Up");
}

void loop() {
   /*
   https://herve-troadec.developpez.com/tutoriels/arduino/initiation-boa/
   https://pythonforundergradengineers.com/python-arduino-LED.html
   Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
   M1=base degrees. Allowed values from 0 to 180 degrees
   M2=shoulder degrees. Allowed values from 15 to 165 degrees
   M3=elbow degrees. Allowed values from 0 to 180 degrees
   M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
   M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */
  
  //(step delay, M1, M2, M3, M4, M5, M6);
  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    String myString = Serial.readString();
    String val1 = getValue(myString, ':', 0);
    //String val2 = getValue(myString, ':', 1);
    //String val3 = getValue(myString, ':', 2);
    String val4 = getValue(myString, ':', 1);
    String val5 = getValue(myString, ':', 2);
    //String val6 = getValue(myString, ':', 5);

    int vM1 = val1.toInt();
    //int vM2 = val2.toInt();
    //int vM3 = val3.toInt();
    int vM4 = val4.toInt();
    int vM5 = val5.toInt();
    //int vM6 = val6.toInt();
    vM1 = constraint(vM1, 0, 180);
    //vM2 = constraint(vM2, 15, 165);
    //vM3 = constraint(vM3, 0, 180);
    vM4 = constraint(vM4, 0, 180);
    vM5 = constraint(vM5, 0, 180);
    //vM6 = constraint(vM6, 10, 73);
    Braccio.ServoMovement(0, vM1, 90, 15, vM4, vM5, 0); //vM6);   
  }
}

int constraint(int value, int min, int max)
{
  if (value<= min){
    return min;
  }else if (value >= max){
    return max;
  }else{
    return value;
  }
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
