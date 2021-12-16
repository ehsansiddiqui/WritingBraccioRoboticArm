#include <Braccio.h>
#include <Servo.h>
#include <ESP8266WiFi.h>                                               
#include <FirebaseArduino.h>

#define FIREBASE_HOST "speechtotext-c9adb.firebaseio.com"              // the project name address from firebase id
#define FIREBASE_AUTH "hhZF9KU1hozlxIuTOKdm7WIHX9frTeDMq3qRWQzy"       // the secret key generated from firebase
#define WIFI_SSID "DESKTOP"                                          
#define WIFI_PASSWORD "ihsaansiddiqui1996" 


Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

String fireStatus = "";

void draw_symbol(char a){
    switch(a){
        case 'L':
//             setxy(x,y,1);
//            drawline(x,y,x,y-font_size*2);
//            drawline(x,y-font_size*2,x+font_size,y-font_size*2);
//            setxy(x+font_size,y-font_size*2,0);


            Braccio.ServoMovement(20, 90, 90, 180, 180, 90, 73);
            delay(1000);
            Braccio.ServoMovement(20, 90,  90, 170, 140,  90,   73);
            delay(1000);
            Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 73);
            delay(1000);
            Braccio.ServoMovement(20, 90,  90, 180, 180,  90,   73);
            delay(1000);
            Braccio.ServoMovement(20, 70,  90, 180, 180,  90,   73);
            delay(1000);
            Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
            
            break;
        case 'C':
            Braccio.ServoMovement(20, 90, 90, 180, 180, 90, 73);
            delay(1000);
             Braccio.ServoMovement(20, 70,  90, 180, 180,  90,   73);
            delay(1000);

            Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
             Braccio.ServoMovement(20, 90,  92, 180, 160,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 70,  92, 180, 160,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
            break;
        case 'E':
            Braccio.ServoMovement(20, 90, 90, 180, 180, 90, 73);
            delay(1000);
             Braccio.ServoMovement(20, 70,  90, 180, 180,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
             Braccio.ServoMovement(20, 90,  90, 180, 170,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 70,  90, 180, 170,  90,   73);
            delay(1000);
            Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
             Braccio.ServoMovement(20, 90,  92, 180, 160,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 70,  92, 180, 160,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
            break;
         case 'F':
             Braccio.ServoMovement(20, 90, 90, 180, 180, 90, 73);
            delay(1000);

            Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
             Braccio.ServoMovement(20, 90,  92, 180, 160,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 70,  92, 180, 160,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
             Braccio.ServoMovement(20, 90,  90, 180, 170,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 70,  90, 180, 170,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
            
            
            break;

          case 'H':
           Braccio.ServoMovement(20, 90, 90, 180, 180, 90, 73);
           delay(1000);
           Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
           delay(1000);
           Braccio.ServoMovement(20, 80,  90, 90, 90, 90, 73);
           delay(1000);
           Braccio.ServoMovement(20, 80, 98, 180, 180, 90, 73);
           delay(1000);
           Braccio.ServoMovement(20, 90, 80, 90, 90, 90, 73);
            delay(1000);
            Braccio.ServoMovement(20, 90,  90, 180, 170,  90,   73);
            delay(1000);
             Braccio.ServoMovement(20, 78,  90, 180, 170,  90,   73);
            delay(1000);
        }
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

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);                               
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {
  /*
  Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
  M1=base degrees. Allowed values from 0 to 180 degrees
  M2=shoulder degrees. Allowed values from 15 to 165 degrees
  M3=elbow degrees. Allowed values from 0 to 180 degrees
  M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
  M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
  M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the tongue is open, 73: the gripper is closed.
  */
  fireStatus = Firebase.getString("LED_STATUS");
  Serial.println(fireStatus);
  char x1;
  Serial.println("Enter the value of x1:");
  while (Serial.available() == 0){
    }
  x1 = Serial.read();
  draw_symbol(x1);
  //Starting position
//    Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 73);
//    delay(1000);



    
//    Braccio.ServoMovement(20, 90,  90, 180, 140,  90,   73);
//    delay(1000);
//    Braccio.ServoMovement(20, 90,  90, 180, 140,  90,   73);
//    delay(1000);
    
                      //(step delay  M1 , M2 , M3 , M4 , M5 , M6);
//  Braccio.ServoMovement(20,           0,  45, 180, 180,  90,  10);
//  
//  //Wait 1 second
//  delay(1000);
//
//  //The Braccio moves to the sponge. Only the M2 servo will moves
//  Braccio.ServoMovement(20,           0,  90, 180, 180,  90,   10);
//Serial.println("Only M6 Servo motor will moves");
//  //Close the gripper to take the sponge. Only the M6 servo will moves
//  Braccio.ServoMovement(10,           0,  90, 180, 180,  90,  60 );
//
//  //Brings the sponge upwards.
//  Braccio.ServoMovement(20,         0,   45, 180,  45,  0, 60);
//Serial.println("Only M1 Servo motor will moves");
//  //Show the sponge. Only the M1 servo will moves
//  Braccio.ServoMovement(20,         180,  45, 180,   45,   0,  60);
//
//  //Return to the start position.
//  Braccio.ServoMovement(20,         0,   90, 180,  180,  90, 60);
//
//  //Open the gripper
//  Braccio.ServoMovement(20,         0,   90, 180,  180,  90, 10 );


}
