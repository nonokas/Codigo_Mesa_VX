/* *******************************
The table will be controled by the targetHeight variable. The stepper will control the 
Table working variables:
  <XLow>  - interaxis horizontal distance of the bottom joints when table is in the minimum height ( lowermost position)
  <XHigh> - the same when the table is in the uppermost height.
  <maxHeight> - max height allowed for the table.Corresponds to <xHeight> + D1+D2
  <minHeigh> - min height allowed for the table. Corresponds to <xLow>
  <targetHeight> - the control variable of the elevation of the table
  <targetX> - the value of X corresponding to targetHeght
Table Configuration 
   L - stands for the inteeraxis of elevation bars
   D1 - Stands for the distance between the upper axis of rotation of the elevation bar to top of table (Sliding disks).
   D2 - Stands for the distance between the lower axis of rotation of the elevation bar to bottom of table.

MODEL_ME_1 refers to the first table. If another one is produced the values below must be configured

Behaviour on BOOT:

 The table moves a litle bi upwards to prevent pressing the switch if it is down, then moves down until reaches de limit swirtch.
 After a pause, it moves up until reaches the upmost switch. This establishes the OPERATION RANGE. Then it moves down again to the 
 normal working height (This must be defined after assembling it to the ROBOT)

Format and example of a JSON STRING command:

{"speed": "1500" , "targetHeight" : "120.5"}

THE MAXIMUM SPEED ALLOWED DEPENDS ON THE POWER SOURCE VOLTAGE, CURRENT AND THE QUALITY OF THE DRIVER.
THE ACCELSTEPPER LIBRARY ACCEPTS FREQUENCIES UP TO 4000 Hz
FOR MY DRIVER THE MAX CURRENT IS 3.5A POWER SOURCE VOLTAGE 24 V. THE MAX FREQ IS 4000 Hz in load free situation. At full load use 1000 Hz

****ALL print statements are for debugging ****

*********************************************/
#define MODEL_ME_1

#ifdef MODEL_ME_1
  double L =240.0; // mm
  double D1 = 41.0 ;  // Table intrinsic variable
  double D2 = 35.0 ;  // Table intrinsic variable
  const double XMIN = 234.5; // Physical dimension according to the position of the downmost limitswitch
#else
//  double L = 240.0; // mm
//  double D1 = 36.0 ; // Até aos patins de apoio
//  double D2 = 31.0 ;
//  const double XMIN = 230; // Physical dimension acording to table
#endif


#include <Arduino.h>
#include <Wire.h> 
#include <AccelStepper.h>
#include <ArduinoJson.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 7
#define enablePin 4
#define switchDown 12
#define switchUp 11
#define motorInterfaceType 1
#define stepsPerRevolution 200
#define gearBoxRatio 20
#define HOMING_DISTANCE 80000000L


//Table variables
double xHigh , xLow , targetX ;
double targetHeight;
float  screwPitch = 1.5;
double maxHeight, minHeight;
double workingHeight =  130; // To be checked upon calibration DO NOT CHANGE 

//Stepper Variables
int max_speed = 4000;     // In the accellstep lib this is the max frequency. For the current situation the max allowed by the DRIVER/MOTOR is 3000 Hz load free
int rotation_speed ;	  // 1000 Hz full load
long position;
long max_position;


const int JSON_CAPACITY = JSON_OBJECT_SIZE(2) + 40;  // Adjust the capacity as needed

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

//******************* Funcao de HOMING ******************************
// Para evitar descer com o switchDown já pressionado, move para cima um pouco, e depois anda com o elevador para baixo até encontrar o switchDown
//(pino 12) 
// All Serial.print statements are just for debuging purposes
//*******************************************************************

void calibrateStepper(){
    stepper.setCurrentPosition(0);
    Serial.println("calibrating..."); 
    stepper.setMaxSpeed(1200);
    if (digitalRead(switchDown) == 0) {
    position = 8000;
    stepper.runToNewPosition(position); //move UP a little to guarantee that microswitch is sufficiently appart
    stepper.setCurrentPosition(0);
    } ;
    delay(500);
    stepper.moveTo(-HOMING_DISTANCE);  // Move down
    while(digitalRead(switchDown) == 1){
      stepper.run();
    };
    stepper.stop();
    stepper.setCurrentPosition(0);
    xLow = XMIN;
    minHeight = L * sqrt(1 - sq(xLow/L ))+ D1 + D2;
    Serial.println("home zero: 0");
    Serial.print ("Min Height:");
    Serial.println(minHeight);
    delay(2000);
    stepper.moveTo(HOMING_DISTANCE);  // Move up
    while(digitalRead(switchUp) == 1){
      stepper.run();
      };
    stepper.stop();
    max_position = stepper.currentPosition();
    stepper.setCurrentPosition(max_position);
    stepper.runToNewPosition(max_position);
    xHigh = xLow - max_position*0.000375;  //max_position*(1/stepsPerRevolution)*(1/gearBoxRatio)*screwPitch;
    maxHeight = L * sqrt(1 - sq(xHigh/L ))+ D1 + D2;
    Serial.print("home max: ");
    Serial.println(max_position);
    Serial.print("X max: ");
    Serial.println(xHigh);

    Serial.print("Max Height: ");
    Serial.println(maxHeight);

    delay(2000);
    // DEFINE workingHeight
    workingHeight -= (D1 + D2);
    targetX = L * sqrt(1-sq(workingHeight/L));
    position = (long)((xLow - targetX)*stepsPerRevolution*gearBoxRatio/screwPitch); // Indicate desired position in no. of steps
    stepper.runToNewPosition(position);
    delay(2000);

}


void setup()
{
  // put your setup code here, to run once:s
  Serial.begin(115200);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(switchDown, INPUT);
  pinMode(switchDown, INPUT);
  stepper.setEnablePin(enablePin);
  stepper.setMinPulseWidth(10);
  digitalWrite(enablePin, HIGH);
  stepper.setMaxSpeed(max_speed);  
  stepper.setAcceleration(1000);  //maxima aceleracao para uma carga de 25kg
  calibrateStepper();
}


void loop() {
  stepper.disableOutputs();
  delay(100);
    
//**************** receive and parse json string ***********************
  if (Serial.available() > 0) {
//  Read the incoming JSON string
    String jsonString = Serial.readString();
    //String jsonString = {"speed":"3000", "targetHeight": "135"};
    // Create a JSON buffer
    StaticJsonDocument<JSON_CAPACITY> jsonBuffer;
    // Deserialize the JSON string
    DeserializationError error = deserializeJson(jsonBuffer, jsonString);
    // Check if parsing was successful
    if (error == DeserializationError::Ok) {
      // Retrieve the values from the JSON
      int speed = jsonBuffer["speed"];
      if (speed > max_speed) speed = max_speed; //Limit maxspeed according to the Stepper driver.
      // Serial.println(speed);
      targetHeight = jsonBuffer["targetHeight"];
      targetHeight -= (D1 + D2);
      targetX = L * sqrt(1-sq(targetHeight/L));
      position = (long)((xLow - targetX)*stepsPerRevolution*gearBoxRatio/screwPitch); // Indicate desired position in no. of steps
      if (position > max_position || position < minHeight) {      // Test HIGHER and LOWER limits to protect the table
        position = stepper.currentPosition();
        Serial.println("Out of Range");
        } else {
          rotation_speed = speed;
          // Print the values to the Serial monitor
          Serial.print("Speed: ");
          Serial.println(speed);
          Serial.print("targetHeight: ");
          Serial.println(targetHeight+ D1 + D2);
          Serial.print("targetX: ");
          Serial.println(targetX);
          Serial.print("position: ");
          Serial.println(position);
        }
      } else {
        // Print an error message if parsing failed
        Serial.print("JSON parsing error: ");
        Serial.println(error.c_str());
      //**********************************************************************
      } 
      stepper.enableOutputs();
      delay(300);
      stepper.setMaxSpeed(rotation_speed);
      delay(300);
      //stepper.runToNewPosition(stepper.currentPosition() + position);
      stepper.runToNewPosition(position); 
      Serial.print("CurrentPosition: ");
      Serial.println(stepper.currentPosition());
      Serial.print("CurrentHeight: ");
      Serial.println(targetHeight+ D1 + D2);
      Serial.print("CurrentX: ");
      Serial.println(targetX);
    }
}

