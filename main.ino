#include <Wire.h>
#include <Zumo32U4.h>
#include "ZumoController.h"


// =======================================================================================
// === GLOBAL VARIABLES & OBJECTS
// =======================================================================================


const uint16_t maxSpeed = 250; // Reduced slightly for safety in obstacles
const uint16_t cruiseSpeed = 150;


Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonC buttonC;


// Zumo controller (Kept for Odometry/Telemetry)
ZumoController zumoController;


// Line Sensor Variables
int16_t lastError = 0;
int16_t curr_error = 0;
#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];


// Thresholds
uint16_t th_line = 300; // Threshold for black line

// Timing variables
const long SenseInterval = 10; // 100Hz
unsigned long lastSense = 0;
unsigned long lastMillis = 0;
unsigned long lastMicros = 0;


// Stage Flags (Mapped to the 5 Phases of Course 2)
bool stage_1 = true;  // Phase 1: Line Follow to T-Junction
bool stage_2 = false; // Phase 2: Corridor (Bounce off walls)
bool stage_3 = false; // Phase 3: Cross, Push, Reverse
bool stage_4 = false; // Phase 4: Line Follow to End
bool stage_5 = false; // Phase 5: Return Home


// Sub-state variables for complex stages
int subState = 0;
unsigned long stateTimer = 0;


// Serial Communication
String inputString = "";      
bool stringComplete = false;  
float batteryVoltage = 0;


// =======================================================================================
// === COMMUNICATION & CONTROLLER FUNCTIONS
// =======================================================================================


void parse_msg(void){
    int ind1 = inputString.indexOf(',');
    String str = inputString.substring(0, ind1);
    int joyX = str.toInt();
    str = inputString.substring(ind1+1);
    int joyY = str.toInt();


    int leftMotor = joyY + joyX;
    int rightMotor = joyY - joyX;
   
    // Telemetry
    Serial.print(leftMotor); Serial.print(" , ");
    Serial.print(rightMotor); Serial.print(" , ");
    Serial.println(batteryVoltage);


    zumoController.motorsSetSpeed(leftMotor,rightMotor);
    inputString = "";
    stringComplete = false;  
}


void msg_handler(void){
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  if (stringComplete) {
    parse_msg();
  }
}


void PathControlInit() {
  Serial.begin(115200);
  inputString.reserve(200);
  lastMillis = millis();
  lastMicros = micros();
 
  // Initialize Odometry 
  zumoController.car_state.posx = 0.0;
  zumoController.car_state.posy = 0.0;
  zumoController.car_state.theta = 0.0;
}


// Update Odometry (Keeps track of X/Y even during reactive moves)
void updateOdometry() {
    unsigned long dtMicros = micros() - lastMicros;
    lastMicros = micros();
    zumoController.dt_time = dtMicros / 1000000.0f;
    zumoController.odometry();
}


// =======================================================================================
// === MOVEMENT LOGIC
// =======================================================================================




void calibrateSensors() {
  delay(1000);
  for(uint16_t i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) motors.setSpeeds(-200, 200);
    else motors.setSpeeds(200, -200);
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}


void LineFollowerInit() {
    lineSensors.initFiveSensors();
    proxSensors.initThreeSensors();
    buzzer.play(">g32>>c32");
    buttonC.waitForButton();
    calibrateSensors();
    buttonC.waitForButton();
    buzzer.play("L16 cdegreg4");
    while(buzzer.isPlaying());
    PathControlInit(); // Start Serial and Timers
}


// Standard Line Follower (PID)
void LineFollower(bool invert = false) {
  int16_t position = lineSensors.readLine(lineSensorValues);
  int16_t error = position - 2000; // Center is 2000 for 5 sensors
  int16_t speedDifference = error / 4 + 6 * (error - lastError);
  lastError = error;


  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;


  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
 
  motors.setSpeeds(leftSpeed, rightSpeed);
}


// =======================================================================================
// === SETUP
// =======================================================================================


void setup() {
  LineFollowerInit();
}


// =======================================================================================
// === MAIN LOOP
// =======================================================================================
void loop() {
  // 1. Update Sensors and Odometry
  unsigned long currentTime = millis();
 
  // Calculate DT for odometry
  if (currentTime - lastMillis >= 10) { // 100Hz Update
      updateOdometry();
      lastMillis = currentTime;
  }

  if (stage_2 == true || (stage_3 == true && subState == 2) || stage_5 == true){
        curr_error = 0;
     }
  else {
        curr_error = lastError;
     }

  // Read Sensors
  if (currentTime - lastSense >= SenseInterval) {
      lastSense = currentTime;
      lineSensors.readCalibrated(lineSensorValues);

      // Send Telemetry
      uint16_t batteryLevel = readBatteryMillivolts();
      batteryVoltage = float(batteryLevel)/1000.0;
      Serial.print(zumoController.car_state.posx); Serial.print(" , ");
      Serial.print(zumoController.car_state.posy); Serial.print(" , ");
      Serial.print(batteryVoltage); Serial.print(" , ");
      Serial.print(zumoController.car_state.v_left); Serial.print(" , ");
      Serial.print(zumoController.car_state.v_right); Serial.print(" , ");
      Serial.println(curr_error);
  }


  // 2. State Machine


  // --- STAGE 1: Line Follow to T-Junction ---
  if (stage_1) {
      if (lineSensorValues[0] > th_line && lineSensorValues[2] > th_line && lineSensorValues[4] > th_line) {
          motors.setSpeeds(0, 0);
          buzzer.play("c32");
          delay(400);
          stage_1 = false;
          Serial.println("STAGE 2");
          stage_2 = true;
          subState = 0;
          motors.setSpeeds(150, 150);
          delay(400);
      } else {
          LineFollower();
      }
  }


  // --- STAGE 2: Corridor (Bounce off walls) ---
  else if (stage_2) {
      if (subState == 0) {
          motors.setSpeeds(cruiseSpeed, cruiseSpeed);
          if (lineSensorValues[4] > th_line) { // Right sensor hits border
              motors.setSpeeds(-cruiseSpeed, cruiseSpeed);
              delay(200);
              subState = 1;
          }
          if (lineSensorValues[0] > th_line) { // Left hits line -> End
              motors.setSpeeds(0, 0);
              stage_2 = false;
              Serial.println("STAGE 3");
              stage_3 = true;
              subState = 0;
          }
      }
      else if (subState == 1) {
          motors.setSpeeds(-60, 150); // Correct Left
          delay(100);
          if (lineSensorValues[4] < th_line) {
              motors.setSpeeds(0, 0);
              subState = 0;
          }
      }
  }


  // --- STAGE 3: Cross, Push, Reverse ---
  else if (stage_3) {
      if (subState == 0) {
          LineFollower();
          if (lineSensorValues[0] > th_line && lineSensorValues[2] > th_line && lineSensorValues[4] > th_line) {
              buzzer.play("e32");
              subState = 1;
              stateTimer = millis();
          }
      }
      else if (subState == 1) {
          LineFollower();
          proxSensors.read();
          LineFollower();
          uint8_t ProxleftValue = proxSensors.countsFrontWithLeftLeds();
          LineFollower();
          uint8_t ProxrightValue = proxSensors.countsFrontWithRightLeds();
          LineFollower();
          // Check for Obstacle or Timeout
          if (ProxleftValue >= 6 || ProxrightValue >= 6) {
              buzzer.play("c32");      
              motors.setSpeeds(50, 50);
              delay(1000);              
              subState = 2;
          }
      }
      else if (subState == 2) {
          motors.setSpeeds(maxSpeed , maxSpeed); // Push
          if (lineSensorValues[4] > th_line) {
              delay(200);
              motors.setSpeeds(0, 0);
              delay(200);
              motors.setSpeeds(maxSpeed , -maxSpeed);
              delay(400);
              stage_3 = false;
              Serial.println("STAGE 4");
              stage_4 = true;
              subState = 0;
        }
     }
  }

  // --- STAGE 4: Line Follow Until Object Detected ---
  else if (stage_4) {
      LineFollower();
      proxSensors.read();
      LineFollower();
      uint8_t ProxleftValue = proxSensors.countsFrontWithLeftLeds();
      LineFollower();
      uint8_t ProxrightValue = proxSensors.countsFrontWithRightLeds();
      LineFollower();
      // 3. Check for Object
      if (ProxleftValue >= 6 || ProxrightValue >= 6) {
          motors.setSpeeds(0, 0);
          delay(700);
          stage_4 = false;
          Serial.println("STAGE 5");
          stage_5 = true;
          subState = 0;
          buzzer.play("c8");
      }
  }


  // --- STAGE 5: 180 Turn -> Cruise -> Return Home ---
  else if (stage_5) {
     
      if (subState == 0) {
          // Start 180 Turn (Blind)
          motors.setSpeeds(-maxSpeed, maxSpeed);
          delay(600);
          motors.setSpeeds(0, 0);
          delay(200);
          subState = 1;
      }
      else if (subState == 1) {
          // Finish turn when the line is found
          motors.setSpeeds(maxSpeed, maxSpeed);
          delay(800);
          motors.setSpeeds(maxSpeed, maxSpeed);
          subState = 2;          
          }
      
      else if (subState == 2) {
          // Check if any sensor hits the line (Center or Left/Right)
          if (lineSensorValues[0] > th_line || lineSensorValues[2] > th_line || lineSensorValues[4] > th_line) {
                motors.setSpeeds(-maxSpeed, maxSpeed);
                delay(300);
                motors.setSpeeds(0, 0);
                delay(400);
                subState = 3; // Move to Return Home logic
    }     else {
          // Drive straight forward blindly to pass the object area
          motors.setSpeeds(maxSpeed, maxSpeed);
    }
      }
      else if (subState == 3) {
        motors.setSpeeds(0, 0);
        buzzer.play("c8g8c8");
        delay(500);
        subState = 0;
        stage_5 = false;
        Serial.println("STAGE 1");
        stage_1 = true;
          }
      }
  


  // Check for incoming serial messages
  msg_handler();
}


