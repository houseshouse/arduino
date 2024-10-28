//========================================================================
// TopFinishKits.com Template Program for Electric Vehicle
//
//  Board: Arduino Uno
//  Vehicle: Electric Vehicle R0
//  Version: 1.0
//
// The information contained in this program is for general education 
// purposes only. The information is provided by TopFinishKits.com and 
// while we endeavor to keep the information up to date and correct, 
// we make no representations or warranties of any kind, express or 
// implied, about the completeness, accuracy, reliability, suitability 
// or availability with respect to the this program, or the website,
// information, products, services, or related graphics contained on
// the website for any purpose. Any reliance you place on such 
// information is therefore strictly at your own risk.
//
// Change Log:
//
// Note: This program needs improve and is only provided as a starting point.
//
//========================================================================

#include <Arduino.h>

//
//  Board: Ardunio Uno
//  DEFINE ALL I/O PIN CONNECTIONS
//    *** DO NOT CHANGE ***
//
#define PIN_MTR_ENCA           2
#define PIN_MTR_ENCB           3
#define PIN_PB_START           4
#define PIN_MTR_DIR_FWD        7
#define PIN_MTR_DIR_REV        8
#define PIN_MTR_PWM           10
#define PIN_LED               13

    // The following are not used in this program.  But these should be used.
#define ENCODER_COUNTS_PER_REV  1080   // Set to the number of encoder pulses per wheel revolution
#define MM_PER_REV              295   // Set to the number of mm per wheel revolution (Hence : Diameter * Pi)

int flagLED;

long countEncoder;
long countEncoderLast;
int motorForward;
int motorReverse;
int motorTargetSpeed;
int motorCommandSpeed;

unsigned long usLast;
long usecElapsed;
long usScanLong;
int  usLongResetCount;
long usScanAvg;
long timerusScan;
int scanCount;

unsigned long msTimerPrint;

unsigned long timerPBStartOn;
unsigned long timerPBStartOff;

int flagUpdateSpeed;
unsigned long timerUpdateSpeed;

int vehicleState;

#define VEHICLE_START_WAIT      1       // Wait for the start button to be pressed
#define VEHICLE_START           2       // First motion command after button press
#define VEHICLE_ACCEL           10
#define VEHICLE_AT_SPEED        20
#define VEHICLE_DECEL           30
#define VEHICLE_FINISHED        900     // Must be at the end of the command list
#define VEHICLE_STOP            1000
#define VEHICLE_ABORT           2000    // Used to abort the current movement list and stop the robot

//---------------------------------------------------------------------------------------
// Interupt function for counting motor encoder pulses
// Note: A typical encoder has two wires, but this program is only using one.  Improve??
void encoderIncr()  { 
  countEncoder++;
}

//---------------------------------------------------------------------------------------
void setMotorOutputs() {
  if (motorForward) digitalWrite(PIN_MTR_DIR_FWD, HIGH);   
  else              digitalWrite(PIN_MTR_DIR_FWD, LOW); 
  if (motorReverse) digitalWrite(PIN_MTR_DIR_REV, HIGH);   
  else              digitalWrite(PIN_MTR_DIR_REV, LOW); 
}

//---------------------------------------------------------------------------------------
void setMotorSpeed() {
  analogWrite(PIN_MTR_PWM,motorCommandSpeed);
}

//---------------------------------------------------------------------------------------
void motorStop() {
  motorForward = 0;
  motorReverse = 0;
  motorTargetSpeed = 0;
  motorCommandSpeed = 0;
  setMotorOutputs();
  setMotorSpeed();
}

//=======================================================================================
// Toggle the Ardunio built in LED each time this function is executed
void toggleLED() {
  if (flagLED) {                
    digitalWrite(PIN_LED,LOW);
    flagLED = false;
  } else {
    digitalWrite(PIN_LED,HIGH);
    flagLED = true;    
  }
}

//=======================================================================================
// Call once to configure the hardware and initialize variables
void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  flagLED = false;

  Serial.begin(115200);
  Serial.println(F("Setup()..."));

  pinMode(PIN_PB_START, INPUT_PULLUP);

  pinMode(PIN_MTR_PWM,OUTPUT);

  pinMode(PIN_MTR_ENCA, INPUT_PULLUP);
  pinMode(PIN_MTR_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_MTR_ENCA), encoderIncr, RISING);

  motorForward = 0;
  motorReverse = 0;
  motorTargetSpeed = 0;
  motorCommandSpeed = 0;

  vehicleState = VEHICLE_START_WAIT;

  timerUpdateSpeed = 0;
  flagUpdateSpeed = 0;

}

//=======================================================================================
void loop() {
  // put your main code here, to run repeatedly:

    // this block calculates the number microseconds since this function's last execution
  unsigned long current = micros();
  usecElapsed = current - usLast;
  usLast = current;
  if (usecElapsed > usScanLong) usScanLong = usecElapsed;
  timerusScan += usecElapsed;
  scanCount++;
  if (timerusScan > 1000000) {
    usScanAvg = timerusScan / scanCount;
    timerusScan = 0;
    scanCount = 0;
    usLongResetCount++;
    if (usLongResetCount > 10) {
      usScanLong = 0;
      usLongResetCount = 0;
    }
  }

    // prints an update via the serial connection every 500000us or 0.5 seconds
  if (msTimerPrint > 500000) {
    Serial.print(F("State = "));
    Serial.print(vehicleState);
    Serial.print(F("   Encoder = "));
    Serial.print(countEncoder);
    Serial.print(F("   Speed = "));
    Serial.println(motorCommandSpeed);
    toggleLED();
    msTimerPrint = 0;
  }
  msTimerPrint += usecElapsed;

  // The variable 'flagUpdateSpeed' is used to either accelerate or decelerate the speed
  // command sent via the PWM output.  Increase or decreasing the time value will adjust
  // this rate.  This is a very simple method that could use improvement.
  timerUpdateSpeed += usecElapsed;
  if (timerUpdateSpeed > 3000) {
    timerUpdateSpeed = 0;
    flagUpdateSpeed = 1;
  } else {
    flagUpdateSpeed = 0;
  }

  int pbStart = !digitalRead(PIN_PB_START);

  if (pbStart) {
    timerPBStartOn  += usecElapsed;
    timerPBStartOff = 0;
  } else {
    timerPBStartOn = 0;
    timerPBStartOff += usecElapsed;
  }

  // This logic is the vehicle run abort
  if (vehicleState > VEHICLE_START && vehicleState < VEHICLE_ABORT) {
    if (timerPBStartOn > 100000) {
      motorStop();
      vehicleState = VEHICLE_ABORT;
    }
  }

  // The switch logic is used to control how the motor speed is set
  //
  // Sequence:
  //    VEHICLE_START_WAIT    Wait for the start pushbutton to be triggered
  //    VEHICLE_START         Clear encoder counts
  //                          Set direction and target speed
  //                          ** Wait until start button is released
  //    VEHICLE_ACCEL         Increase motor speed from 0 to target speed
  //    VEHICLE_AT_SPEED      Wait until the encoder count reaches the deceleration point
  //    VEHICLE_DECEL         Decrease motor speed from target speed to 0
  //    VEHICLE_STOP          Turn off motor and move to top of sequence
  //
  //  Notes:   -  Pushbutton logic delay is part of the vehicle run time.  Think how this
  //              could effect a run score.
  //           -  The deceleration starts at a set number of encoder counts.  Should this
  //              be encoder counts or a distance?
  //           -  The deceleration logic should be improved to stop more accurately
  //           -  There are other improvements needed
  //
  switch (vehicleState) {
    case VEHICLE_START_WAIT :
      if (timerPBStartOn > 300000) vehicleState = VEHICLE_START;
      break;
    case VEHICLE_START :
      countEncoder = 0;
      motorTargetSpeed = 250;
      motorCommandSpeed = 0;
      motorForward = 1;
      setMotorOutputs();
      if (timerPBStartOff > 100000) vehicleState = VEHICLE_ACCEL;
      break;
    case VEHICLE_ACCEL :
      if (flagUpdateSpeed) {
        if (motorCommandSpeed < motorTargetSpeed) motorCommandSpeed++;
        else vehicleState = VEHICLE_AT_SPEED;
        setMotorSpeed();
      }
      break;
    case VEHICLE_AT_SPEED :
      if (countEncoder > 3500) vehicleState = VEHICLE_DECEL;       // encoder counts to start decel - would be nice if calculated
      break;
    case VEHICLE_DECEL :
      if (flagUpdateSpeed) {
        if (motorCommandSpeed > 0) motorCommandSpeed--;
        setMotorSpeed();
      }
      break;
    case VEHICLE_FINISHED :
      motorStop();
      vehicleState = VEHICLE_START_WAIT;
      break;
    case VEHICLE_STOP :
      motorStop();
      vehicleState = VEHICLE_START_WAIT;
      break;
    case VEHICLE_ABORT : 
      if (timerPBStartOff > 1000000) vehicleState = VEHICLE_START_WAIT;
      break;
  }


}  // end of program
