#include <Adafruit_ADS1X15.h>


#define SL_Motor_In 5   // Slits move motor intwards (motor 1 relay 2)
#define SL_Motor_Out 4  // Slits move motor outwards (motor 1 relay 1)
#define SL_Speed_Pin 12

#define FC_Motor_In 6   // FC move motor inwards ( motor 2 relay 3)
#define FC_Motor_Out 7  //FC move motor outwards (motor 2 relay 4)

// Limit switches (1 - not triggered, 0 -triggered)
#define SL_LS_OUT 8
#define SL_LS_IN 9
#define FC_LS_IN 10
#define FC_LS_OUT 11


Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

const float SL_MIN_POSITION = 0.0;    // Minimum position (mm)
const float SL_MAX_POSITION = 123.5;  // Maximum position (mm)

const float SLOW_DOWN_DISTANCE = 1.0; // mm distance from target to slow down
const int SLOW_FREQ = 200;            // Hz for slow speed
const int NORMAL_FREQ = 800;          // Hz for normal speed
int currentUserFrequency = NORMAL_FREQ; // Default frequency

const unsigned long statusInterval = 500; // ms
unsigned long lastStatusTime = 0;


// ======================== State Variables ==================
enum MovementState {
  IDLE,
  MOVING_IN,
  MOVING_OUT
};

MovementState slState = IDLE;
MovementState fcState = IDLE;

float slStartPos = 0;
float slTargetPos = 0;
bool movingToTarget = false;

// ======================== Helper functions ================

float rdAvgVoltage(uint8_t channel = 0, int samples = 10) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += ads.readADC_SingleEnded(channel);
    delay(5);  // small delay between samples
  }
  float avg = sum / float(samples);
  return ads.computeVolts(avg);
}

//TODO: recalibrate
float getSlitPosition(uint8_t channel = 0) {
  float volts = rdAvgVoltage(channel);
  //const float minVoltage = 0.29;
  //const float maxVoltage = 4.242;
  const float minVoltage = 0.128;
  const float maxVoltage = 4.19;
  float pos = (volts - minVoltage) * (SL_MAX_POSITION - SL_MIN_POSITION) / (maxVoltage - minVoltage);
  //Serial.print(volts,3); Serial.print(" V; ");Serial.print(pos);  Serial.println(" mm");
  return pos;
}


void moveToPosition(float targetPos) {
  float currentPos = getSlitPosition();

  if (targetPos > SL_MAX_POSITION) {
    Serial.println("INFO: Target out of boundaries. Moving to closest safe position");
      targetPos = SL_MAX_POSITION;
  } else if (targetPos < SL_MIN_POSITION) {
    Serial.println("INFO: Target out of boundaries. Moving to closest safe position...");
      targetPos = SL_MIN_POSITION;
  }

  // Stop any current movement
  stopSlMovement();
  // Use the current user frequency
  tone(SL_Speed_Pin, currentUserFrequency);
  // Determine direction and move
  if (targetPos > currentPos) {
    digitalWrite(SL_Motor_In, HIGH);
    slState = MOVING_IN;
    Serial.print("INFO: Moving IN from ");
    Serial.print(currentPos);
    Serial.print(" to ");
    Serial.println(targetPos);
  } else {
    digitalWrite(SL_Motor_Out, HIGH);
    slState = MOVING_OUT;
    Serial.print("INFO: Moving OUT from ");
    Serial.print(currentPos);
    Serial.print(" to ");
    Serial.println(targetPos);
  }
  slTargetPos = targetPos;
  movingToTarget = true;
}


void stopSlMovement() {
  digitalWrite(SL_Motor_In, LOW);
  digitalWrite(SL_Motor_Out, LOW);
  slState = IDLE;
  movingToTarget = false;
}

void stopFcMovement() {
  digitalWrite(FC_Motor_In, LOW);
  digitalWrite(FC_Motor_Out, LOW);
  fcState = IDLE;
}

//Check if movement should stop due to limit switch
bool checkLimitSwitches() {
  // Check SLITS limit switches
  if (slState == MOVING_IN && !digitalRead(SL_LS_IN)) {
    Serial.println("SLITS: Limit switch IN triggered!");
    stopSlMovement();
    return true;
  }
  if (slState == MOVING_OUT && !digitalRead(SL_LS_OUT)) {
    Serial.println("SLITS: Limit switch OUT triggered!");
    stopSlMovement();
    return true;
  }

  // Check FC limit switches
  if (fcState == MOVING_IN && !digitalRead(FC_LS_IN)) {
    Serial.println("FC: Limit switch IN triggered!");
    stopFcMovement();
    return true;
  }
  if (fcState == MOVING_OUT && !digitalRead(FC_LS_OUT)) {
    Serial.println("FC: Limit switch OUT triggered!");
    stopFcMovement();
    return true;
  }

  return false;
}

void sendStatusMessage() {
  float pos = getSlitPosition();
  bool read_FC_LS_IN = !digitalRead(FC_LS_IN);
  bool read_FC_LS_OUT = !digitalRead(FC_LS_OUT);
  bool read_SL_LS_IN = !digitalRead(SL_LS_IN);
  bool read_SL_LS_OUT = !digitalRead(SL_LS_OUT);
  // Motion states ("1"=moving, "0"=idle)
  String sl_state = (slState != IDLE) ? "1" : "0";
  String fc_state = (fcState != IDLE) ? "1" : "0";

  // status message
  String message = "&FC:" + String(read_FC_LS_IN) + ";" + String(read_FC_LS_OUT) + ";" + fc_state + 
  ";&SL:" + String(read_SL_LS_IN) + ";" + String(read_SL_LS_OUT) + ";" + sl_state + ";" + 
  String(pos, 2) + "&";
  Serial.println(message);
}

void updatePositionMovement() {
  if (!movingToTarget) return;

  float currPos = getSlitPosition();
  float distanceToTarget = abs(currPos - slTargetPos);
  // Check if we should slow down when approaching target
  if (distanceToTarget < SLOW_DOWN_DISTANCE) {
    tone(SL_Speed_Pin, SLOW_FREQ);
    //Serial.println("Slowing down");
  }
  else if (distanceToTarget <2*SLOW_DOWN_DISTANCE){
    //Serial.println("Slowing down 1");
    tone(SL_Speed_Pin, (SLOW_FREQ+currentUserFrequency)/2);
  }else {
    tone(SL_Speed_Pin, currentUserFrequency);
  }

  // Check if target reached
  if ((slState == MOVING_IN && currPos >= slTargetPos) || (slState == MOVING_OUT && currPos <= slTargetPos)) {
    Serial.print("Target reached. Final position: ");
    Serial.println(currPos);
    // Restore user frequency
    tone(SL_Speed_Pin, currentUserFrequency);
    stopSlMovement();
  }
}




// ========================= MAIN PART ========================

void setup() {
  Serial.begin(38400);

  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  // 5V to transistor base to swith relay
  pinMode(SL_Motor_In, OUTPUT);
  pinMode(SL_Motor_Out, OUTPUT);
  pinMode(SL_Speed_Pin, OUTPUT);
  pinMode(FC_Motor_In, OUTPUT);
  pinMode(FC_Motor_Out, OUTPUT);

  tone(SL_Speed_Pin, NORMAL_FREQ);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    // ================== MOTOR 1 (SLITS) ======================
    // Turn on rotation INWARDS
    if (command == "SL_IN") {
      // if moves outwards, turn it off before moving inwards
      if (digitalRead(SL_Motor_Out)) {
        digitalWrite(SL_Motor_Out, LOW);
      }
      slState = MOVING_IN;
      movingToTarget = false;
      digitalWrite(SL_Motor_In, HIGH);
      Serial.println("SLITS: Turn ON rotation INWARDS");
    }
    // Turn on rotation OUTWARDS
    else if (command == "SL_OUT") {
      // if moves inwards, turn it off before moving outwards
      if (digitalRead(SL_Motor_In)) {
        digitalWrite(SL_Motor_In, LOW);
      }
      slState = MOVING_OUT;
      movingToTarget = false;
      digitalWrite(SL_Motor_Out, HIGH);
      Serial.println("SLITS: Turn ON rotation OUTWARDS");
    }
    // Turn off rotation in any direction
    else if (command == "SL_STOP") {
      stopSlMovement();
      Serial.println("SLITS: Turn OFF rotation");
    }
    // ================== MOTOR 2 (FC) ======================
    // Turn on rotation INWARDS
    else if (command == "FC_IN") {
      // if moves outwards, turn it off before moving inwards
      if (digitalRead(FC_Motor_Out)) {
        digitalWrite(FC_Motor_Out, LOW);
      }
      fcState = MOVING_IN;
      digitalWrite(FC_Motor_In, HIGH);
      Serial.println("FC: Turn ON rotation INWARDS");
    }
    // Turn on rotation OUTWARDS
    else if (command == "FC_OUT") {
      // if moves inwards, turn it off before moving outwards
      if (digitalRead(FC_Motor_In)) {
        digitalWrite(FC_Motor_In, LOW);
      }
      fcState = MOVING_OUT;
      digitalWrite(FC_Motor_Out, HIGH);
      Serial.println("FC: Turn ON rotation OUTWARDS");
    }
    // Turn off rotation in any direction
    else if (command == "FC_STOP") {
      stopFcMovement();
      Serial.println("FC: Turn OFF rotation");
    }

    // ========= Position control commands  ==============
    else if (command.startsWith("SL_TO#")) {
      float targetPos = command.substring(6).toFloat();
      moveToPosition(targetPos);
    }
    else if (command.startsWith("SL_IN_BY#")) {
      float x = command.substring(9).toFloat();
      float currentPos = getSlitPosition();
      float targetPos = currentPos + x;
      moveToPosition(targetPos);
    }
    else if (command.startsWith("SL_OUT_BY#")) {
      float x = command.substring(10).toFloat();
      float currentPos = getSlitPosition();
      float targetPos = currentPos - x;
      moveToPosition(targetPos);
    }
    else if (command.startsWith("SL_FR#")) {
      int slFreq = command.substring(6).toInt();
      if (slFreq < 31){
        currentUserFrequency = 32;
        tone(SL_Speed_Pin,currentUserFrequency);
        Serial.println("Frequency to low. Set to 32 Hz");
      }
      else if (slFreq >=1000){
        Serial.println("Frequency is too high. Enter value between 32 and 1000 Hz");
      }
      else{
        currentUserFrequency = slFreq;
        tone(SL_Speed_Pin,currentUserFrequency);
        Serial.print("Slit speed set to "); Serial.print(currentUserFrequency); Serial.println(" Hz");
      }
    }
    // If unknown command stop everything
    else {
      Serial.print("Unknown command. Stopping... Command: ");
      Serial.println(command);
      stopSlMovement();
      stopFcMovement();
    }
  }

  checkLimitSwitches();
  updatePositionMovement();

  // Send status at regular intervals
  unsigned long currentTime = millis();
  if (currentTime - lastStatusTime >= statusInterval) {
    sendStatusMessage();
    lastStatusTime = currentTime;
  }

}
