
/*
  simpleMovements.ino

 This  sketch simpleMovements shows how they move each servo motor of Braccio

 Created on 18 Nov 2015
 by Andrea Martino

 This example is in the public domain.
 */

#include <Braccio.h>
#include <Servo.h>
#include <math.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

volatile int angle1[11] = {3756,3618,3551,3503,3462,3427,3395,3365,3338,3314,3290};
volatile int angle2[11] = {5999,5713,5573,5473,5385,5311,5243,5178,5119,5068,5016};
volatile int angle3[11] = {1256,1404,1477,1530,1576,1615,1651,1686,1717,1745,1773};
volatile double length[115] = {20.2,20.1,20.0,19.9,19.8,19.7,19.6,19.5,19.4,19.3,19.2}; 
//tan_value from -89 to 89, step = 0.5
volatile double tan_value[11] = {-1.7321,-1.7182,-1.7045,-1.6909,-1.6775,-1.6643,-1.6512,-1.6383,-1.6255,-1.6128,-1.6003};
//o0 pwm on time value: from -89 to 89 step 0.5
volatile int angle0[11] = {1832,1838,1843,1849,1855,1860,1866,1871,1877,1882,1888};

int getsita12(double x, double y){
    int i;
    double radius;
    radius = sqrt(x*x+y*y);
    for(i = 0; i <= 115; i++){
        if(radius <= length[i] && radius >= length[i+1])
            break;
    }
    if(i >= 115)
        i = 115;
    return i;
}
int getsita0(double x, double y){
    int k;
    double tan_sita;
    tan_sita = -x/y;
    for(k = 0; k <= 600; k++){
        if(tan_sita >= tan_value[k] && tan_sita <= tan_value[k+1])
            break;
    }
    if(k >= 600)
        k = 600;
    return k;
}

 void writexy(double x, double y){
                        int a,b;
                        a = getsita12(x,y);
                        b = getsita0(x,y);
                        Braccio.ServoMovement(20,angle0[b],angle1[a],angle2[a], angle3[a]+400, angle3[a]+300,  73);
                        delay(100);
                    }

void setup() {
  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:
  //Base (M1):90 degrees
  //Shoulder (M2): 45 degrees
  //Elbow (M3): 180 degrees
  //Wrist vertical (M4): 180 degrees
  //Wrist rotation (M5): 90 degrees
  //gripper (M6): 10 degrees
  Braccio.begin();
  Serial.begin(9600);
}

void loop() {
   /*
   Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
   M1=base degrees. Allowed values from 0 to 180 degrees
   M2=shoulder degrees. Allowed values from 15 to 165 degrees
   M3=elbow degrees. Allowed values from 0 to 180 degrees
   M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
   M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */
  
//                       //(step delay, M1, M2, M3, M4, M5, M6);
//  Braccio.ServoMovement(20,           0,  15, 180, 170, 0,  73);  
//
//  //Wait 1 second
//  delay(1000);
//
//  Braccio.ServoMovement(20,           180,  165, 0, 0, 180,  73);  
//
//  //Wait 1 second
//  delay(1000);
   
   double stepx,stepy,x1,x2,y1,y2;
                        int i;
                        Serial.println("Enter the value of x1:");
                        while (Serial.available() == 0){
    
                        }
                        x1 = Serial.readString().toDouble();
                        Serial.println("Enter the value of x2:");
                        while (Serial.available() == 0){
    
                        }
                        x2 = Serial.readString().toDouble();
                        Serial.println("Enter the value of y1:");
                        while (Serial.available() == 0){
    
                        }
                        y1 = Serial.readString().toDouble();
                        Serial.println("Enter the value of y2:");
                        while (Serial.available() == 0){
    
                        }
                        y2 = Serial.readString().toDouble();
                        stepx = (x2-x1)/10.0;                                                  // calculate the step size
                        stepy = (y2-y1)/10.0;
                        writexy(x1,y1);
                        for(i=0;i<=10;i++){
                            x1 = x1 + stepx;
                            y1 = y1 + stepy;
                            writexy(x1,y1);
                        }
}
