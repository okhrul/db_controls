#include <Adafruit_ADS1X15.h>


#define SL_Motor_In 5 // Slits move motor intwards (motor 1 relay 2)
#define SL_Motor_Out 4 // Slits move motor outwards (motor 1 relay 1)

#define FC_Motor_In 6 // FC move motor inwards ( motor 2 relay 3)
#define FC_Motor_Out 7 //FC move motor outwards (motor 2 relay 4)

// Limit switches (1 - not triggered, 0 -triggered)
#define SL_LS_OUT 8 
#define SL_LS_IN 9 
#define FC_LS_IN 10 
#define FC_LS_OUT 11 


Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

const float rangeDistance = 130.6-10.2; // slis range

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
    delay(5); // small delay between samples
  }
  float avg = sum / float(samples);
  return ads.computeVolts(avg);
}

//TODO: recalibrate
float getSlitPosition(uint8_t channel = 0) {
  float volts = rdAvgVoltage(channel);
  const float minVoltage = 0.29;
  const float maxVoltage = 4.242;
  return (volts - minVoltage) * rangeDistance / (maxVoltage - minVoltage);
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
  
  String message = "&FC:" + String(read_FC_LS_IN) + ";" + String(read_FC_LS_OUT) +
                   ";&SL:" + String(read_SL_LS_IN) + ";" + String(read_SL_LS_OUT) +
                   ";" + String(pos) + "&";
  Serial.println(message);
}

void updatePositionMovement() {
  if (!movingToTarget) return;
  
  // unsigned long currentTime = millis();
  // if (currentTime - lastPositionCheck < positionCheckInterval) return;
  // lastPositionCheck = currentTime;

  float currPos = getSlitPosition();
  
  // Check if target reached
  if ((slState == MOVING_IN && currPos >= slTargetPos) ||
      (slState == MOVING_OUT && currPos <= slTargetPos)) {
    Serial.print("Target reached. Final position: ");
    Serial.println(currPos);
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
  pinMode(FC_Motor_In, OUTPUT);
  pinMode(FC_Motor_Out, OUTPUT);

}

//TODO: 1)status in motion 2) non blocking loop

void loop() {
  if (Serial.available()){
    String command = Serial.readStringUntil('\n');
    command.trim();
    // ================== MOTOR 1 (SLITS) ======================
    // Turn on rotation INWARDS
    if (command == "SL_IN"){
      // if moves outwards, turn it off before moving inwards
      if (digitalRead(SL_Motor_Out)){
        digitalWrite(SL_Motor_Out, LOW);
      }
      slState = MOVING_IN;
      movingToTarget = false;
      digitalWrite(SL_Motor_In, HIGH);
      Serial.println("SLITS: Turn ON rotation INWARDS");
    }
    // Turn on rotation OUTWARDS
    else if (command == "SL_OUT"){
      // if moves inwards, turn it off before moving outwards
      if (digitalRead(SL_Motor_In)){
        digitalWrite(SL_Motor_In, LOW);
      }
      slState = MOVING_OUT;
      movingToTarget = false;
      digitalWrite(SL_Motor_Out, HIGH);
      Serial.println("SLITS: Turn ON rotation OUTWARDS");
    }
    // Turn off rotation in any direction
    else if (command == "SL_STOP"){
      stopSlMovement();
      Serial.println("SLITS: Turn OFF rotation");
    }
    // ================== MOTOR 2 (FC) ======================
    // Turn on rotation INWARDS
    else if (command == "FC_IN"){
      // if moves outwards, turn it off before moving inwards
      if (digitalRead(FC_Motor_Out)){
        digitalWrite(FC_Motor_Out, LOW);
      }
      fcState = MOVING_IN;
      digitalWrite(FC_Motor_In, HIGH);
      Serial.println("FC: Turn ON rotation INWARDS");
    }
    // Turn on rotation OUTWARDS
    else if (command == "FC_OUT"){
      // if moves inwards, turn it off before moving outwards
      if (digitalRead(FC_Motor_In)){
        digitalWrite(FC_Motor_In, LOW);
      }
      fcState = MOVING_OUT;
      digitalWrite(FC_Motor_Out, HIGH);
      Serial.println("FC: Turn ON rotation OUTWARDS");
    }
    // Turn off rotation in any direction
    else if (command == "FC_STOP"){
      stopFcMovement();
      Serial.println("FC: Turn OFF rotation");
    }

    // ========= Move IN by a certain value x (mm) ==============
    else if (command.startsWith("SL_IN_BY#")) {
      // doesn't check if we entered a number...
      float x = command.substring(9).toFloat();
      Serial.print("Moving IN by ");
      Serial.println(x);
      // Stop any current movement
      stopSlMovement();
      
      slStartPos = getSlitPosition();
      slTargetPos = slStartPos + x;
      digitalWrite(SL_Motor_In, HIGH);
      slState = MOVING_IN;
      movingToTarget = true;
    }
     // Move OUT by a certain value x (mm)
    else if (command.startsWith("SL_OUT_BY#")) {
      // doesn't check if we entered a number...
      float x = command.substring(10).toFloat();
      Serial.print("Moving OUT by ");
      Serial.println(x);
      // Stop any current movement
      stopSlMovement();
      
      slStartPos = getSlitPosition();
      slTargetPos = slStartPos - x;
      digitalWrite(SL_Motor_Out, HIGH);
      slState = MOVING_OUT;
      movingToTarget = true;
    }
    // If unknown command stop everything
    else{
      Serial.print("Unknown command. Stopping... Command: ");
      Serial.println(command);
      stopSlMovement();
      stopFcMovement();
    }
  }

  checkLimitSwitches();
  updatePositionMovement();
  sendStatusMessage();

  delay(100);
}
