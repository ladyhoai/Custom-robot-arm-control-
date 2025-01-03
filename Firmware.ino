#include <AccelStepper.h>
#include <arduino-timer.h>

// Define the motor pins
#define STEP_PIN 2
#define DIR_PIN 5

#define STEP_PIN_2 3
#define DIR_PIN_2 6

#define STEP_PIN_3 4
#define DIR_PIN_3 7

#define STEP_PIN_4 12
#define DIR_PIN_4 13

// // Create an AccelStepper object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper stepperShoulder1(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepperShoulder2(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
AccelStepper stepperWrist2(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);

auto timer = timer_create_default();

// Variable to store the received key list
String keyList = "";

// Motor control speed and acceleration
int motorSpeed = 1000;   // Speed of the stepper motor (steps per second)
int motorAccel = 500;    // Acceleration (steps per second^2)
int i = 0;
bool updateVel(void *) {
    if (i < 900) {
      stepper.setSpeed(jointVel[i][0]);
      stepperShoulder1.setSpeed(jointVel[i][1]);
      stepperShoulder2.setSpeed(jointVel[i][2]/4);
      i++;
    }
    else if (i == 900) {
      stepper.setSpeed(0);
      stepperShoulder1.setSpeed(0);
      stepperShoulder2.setSpeed(0);
      stepper.moveTo(0);
      stepperShoulder1.moveTo(0);
      stepperShoulder2.moveTo(0);
      i++;
    }
    return true;
}


void setup() {
  Serial.begin(115200);

  // Set maximum speed and acceleration
  stepper.setMaxSpeed(1500); // steps per second
  stepper.setAcceleration(250/2); // steps per second squared

  stepperShoulder1.setMaxSpeed(1250); // steps per second
  stepperShoulder1.setAcceleration(1250/4); // steps per second squared

  stepperShoulder2.setMaxSpeed(1250/8); // steps per second
  stepperShoulder2.setAcceleration(1250/16); // steps per second squared

  stepperWrist2.setMaxSpeed(2000);
  stepperWrist2.setAcceleration(1000);
  // stepper.moveTo(-1146);
  // stepperShoulder1.moveTo(-3800);
  // stepperShoulder2.moveTo(-1150);

  // while (millis() < 15000) {
  //   stepper.run();
  //   stepperShoulder1.run();
  //   stepperShoulder2.run();
  // }
  // timer.every(10, updateVel);

}

bool RRMC = false;

double J1 = 0, J2 = 0, J3 = 0, J4 = 0;
//a043s057d069
void loop() {
  // Check if serial data is available
  if (Serial.available() && !RRMC) {
    
    keyList = Serial.readStringUntil('\n');  // Read the incoming list of keys
    
    // Check for each key and append corresponding number to the buffer
    int indexQ = keyList.indexOf('q'), indexA = keyList.indexOf('a');
    int indexW = keyList.indexOf('w'), indexS = keyList.indexOf('s');
    int indexE = keyList.indexOf('e'), indexD = keyList.indexOf('d');
    int indexR = keyList.indexOf('r'), indexF = keyList.indexOf('f');
    int indexT = keyList.indexOf('t');
    if (indexT != -1) {
      RRMC = true;
    }
    if (indexQ != -1) {
        double angle = keyList.substring(indexQ + 1, indexQ + 4).toDouble();
        stepper.moveTo( 9600 * (angle/360));
        J1 = angle;
    }
    else if (indexA != -1) {
        double angle = keyList.substring(indexA + 1, indexA + 4).toDouble();
        stepper.moveTo( - 9600 * (angle/360));
        J1 = angle;
    }

    if (indexW != -1) {
      double angle = keyList.substring(indexW + 1, indexW + 4).toDouble();
      stepperShoulder1.moveTo( + 24000 * (angle/360));
      J2 = angle;
    }
    else if (indexS != -1) {
      double angle = keyList.substring(indexS + 1, indexS + 4).toDouble();
      stepperShoulder1.moveTo( - 24000 * (angle/360));
      J2 = angle;
    }

    if (indexE != -1) {
      double angle = keyList.substring(indexE + 1, indexE + 4).toDouble();
      stepperShoulder2.moveTo( + 6000 * (angle/360));
      J3 = angle;
    }
    else if (indexD != -1) {
      double angle = keyList.substring(indexD + 1, indexD + 4).toDouble();
      stepperShoulder2.moveTo( - 6000 * (angle/360));
      J3 = angle;
    }

    if (indexR != -1) {
      double angle = keyList.substring(indexR + 1, indexR + 4).toDouble();
      stepperWrist2.moveTo( + 12800 * (angle/360));
      J4 = angle;
    }
    else if (indexF != -1) {
      double angle = keyList.substring(indexF + 1, indexF + 4).toDouble();
      stepperWrist2.moveTo( - 12800 * (angle/360));
      J4 = angle;
    }

    if (!RRMC) {
      Serial.print("J1: ");
      Serial.print(J1);
      Serial.print(" | J2: ");
      Serial.print(J2);
      Serial.print(" | J3: ");
      Serial.print(J3);
      Serial.print(" | J4: ");
      Serial.println(J4);
    }
  }

  if (Serial.available() && RRMC) {
    RRMC = true;
    int16_t dataArray[4]; // Array to store received values

    // Read the binary data
    for (int i = 0; i < 4; i++) {
      Serial.readBytes((char*)&dataArray[i], sizeof(int16_t));
    }

    stepper.setSpeed(-dataArray[0]);
    stepperShoulder1.setSpeed(-dataArray[1]);
    stepperShoulder2.setSpeed(-dataArray[2]);
    stepperWrist2.setSpeed(-dataArray[3]);

    if (dataArray[0] == 0 && dataArray[1] == 0 && dataArray[2] == 0 && dataArray[3] == 0) {
      RRMC = false;
    }
  
  }

  if (RRMC) {
    stepperShoulder1.runSpeed();
    stepperShoulder2.runSpeed();
    stepper.runSpeed();  // Run motor to the target position
    stepperWrist2.runSpeed();
  }
  else {
    stepperShoulder1.run();
    stepperShoulder2.run();
    stepper.run();  // Run motor to the target position
    stepperWrist2.run();
  }
}

// void runQMatrix() {
//   for (int i = 0; i < 300; i++) {
//     stepper.moveTo(-9600 * (jointVal[i][0]/360));
//     stepperShoulder1.moveTo(24000 * (jointVal[i][1]/360));
//     stepperShoulder2.moveTo(24000 * (jointVal[i][2]/360));

//     while (stepper.isRunning() || stepperShoulder1.isRunning() || stepperShoulder2.isRunning()) {
//     stepper.run();
//     stepperShoulder1.run();
//     stepperShoulder2.run();
//   }
//   }
// }
