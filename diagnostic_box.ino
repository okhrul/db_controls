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

//Check if movement should stop due to limit switch
bool limitSwitchHit(String dir) {
  if (dir == "IN" && (!digitalRead(SL_LS_IN))) {
    Serial.println("SL Limit switch IN triggered!");
    return true;
  }
  if (dir == "OUT" && (!digitalRead(SL_LS_OUT))) {
    Serial.println("SL Limit switch OUT triggered!");
    return true;
  }
  return false;
}

// ========================= MAIN PART ========================

void setup() {
  Serial.begin(38400);

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
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
      digitalWrite(SL_Motor_In, HIGH);
      Serial.println("SLITS: Turn ON rotation INWARDS");
    }
    // Turn on rotation OUTWARDS
    else if (command == "SL_OUT"){
      // if moves inwards, turn it off before moving outwards
      if (digitalRead(SL_Motor_In)){
        digitalWrite(SL_Motor_In, LOW);
      }
      digitalWrite(SL_Motor_Out, HIGH);
      Serial.println("SLITS: Turn ON rotation OUTWARDS");
    }
    // Turn off rotation in any direction
    else if (command == "SL_STOP"){
      digitalWrite(SL_Motor_In, LOW);
      digitalWrite(SL_Motor_Out, LOW);
      Serial.println("SLITS: Turn OFF rotation");
    }
    // ================== MOTOR 2 (FC) ======================
    // Turn on rotation INWARDS
    else if (command == "FC_IN"){
      // if moves outwards, turn it off before moving inwards
      if (digitalRead(FC_Motor_Out)){
        digitalWrite(FC_Motor_Out, LOW);
      }
      digitalWrite(FC_Motor_In, HIGH);
      Serial.println("FC: Turn ON rotation INWARDS");
    }
    // Turn on rotation OUTWARDS
    else if (command == "FC_OUT"){
      // if moves inwards, turn it off before moving outwards
      if (digitalRead(FC_Motor_In)){
        digitalWrite(FC_Motor_In, LOW);
      }
      digitalWrite(FC_Motor_Out, HIGH);
      Serial.println("FC: Turn ON rotation OUTWARDS");
    }
    // Turn off rotation in any direction
    else if (command == "FC_STOP"){
      digitalWrite(FC_Motor_In, LOW);
      digitalWrite(FC_Motor_Out, LOW);
      Serial.println("FC: Turn OFF rotation");
    }

    // ========= Move IN by a certain value x (mm) ==============
    // TODO: avoid loopong in moveInBy 
    else if (command.startsWith("SL_IN_BY#")) {
      // doesn't check if we entered a number...
      float x = command.substring(9).toFloat();
      Serial.print("Moving IN by ");
      Serial.println(x);
      moveInBy(x, "IN");
    }
     // Move OUT by a certain value x (mm)
    else if (command.startsWith("SL_OUT_BY#")) {
      // doesn't check if we entered a number...
      float x = command.substring(10).toFloat();
      Serial.print("Moving OUT by ");
      Serial.println(x);
      moveInBy(x, "OUT");
    }
    // If unknown command stop everything
    else{
      Serial.print("Unknown command. Stopping... Command: ");
      Serial.println(command);
      digitalWrite(SL_Motor_In, LOW);
      digitalWrite(SL_Motor_Out, LOW);
    }
  }


  //========== LIMITS SWITCHES ==============
  if ((!digitalRead(SL_LS_OUT)) && digitalRead(SL_Motor_Out)){
    digitalWrite(SL_Motor_Out, LOW);
    Serial.println("Movement stopped due to limit switch.");
  }  
  if ((!digitalRead(SL_LS_IN)) && digitalRead(SL_Motor_In)){
    digitalWrite(SL_Motor_In, LOW);
    Serial.println("Movement stopped due to limit switch.");
  }  
  if ((!digitalRead(FC_LS_OUT)) && digitalRead(FC_Motor_Out)){
    digitalWrite(FC_Motor_Out, LOW);
    Serial.println("Movement stopped due to limit switch.");
  }  
  if ((!digitalRead(FC_LS_IN)) && digitalRead(FC_Motor_In)){
    digitalWrite(FC_Motor_In, LOW);
    Serial.println("Movement stopped due to limit switch.");
  }  



  // ============= SEND INFO ==================================

  //Serial.print("FC_LS_IN "); Serial.print(digitalRead(FC_LS_IN)); Serial.print(" FC_LS_OUT "); Serial.println(digitalRead(FC_LS_OUT));
  //Serial.print("SL_LS_IN "); Serial.print(digitalRead(SL_LS_IN)); Serial.print(" SL_LS_OUT "); Serial.println(digitalRead(SL_LS_OUT));


  float pos = getSlitPosition(); 
  
  //Serial.print(pos,2); Serial.println("mm");
 //slits down 4.24V
 //slits up 0.29V 10.2mm

  bool read_FC_LS_IN = !digitalRead(FC_LS_IN);
  bool read_FC_LS_OUT = !digitalRead(FC_LS_OUT);
  bool read_SL_LS_IN = !digitalRead(SL_LS_IN);
  bool read_SL_LS_OUT = !digitalRead(SL_LS_OUT);
  // limit in, out, pos "&FC:0;0;&SL:0;0;pos&"
  String message = "&FC:" + String(read_FC_LS_IN) + ";" + String(read_FC_LS_OUT) +
                   ";&SL:" + String(read_SL_LS_IN) + ";" + String(read_SL_LS_OUT) +
                   ";" + String(pos) + "&";
  Serial.println(message);
  delay(100);
}



void moveInBy(float x, String dir){

  // Stop all motors before starting 
  digitalWrite(SL_Motor_In, LOW);
  digitalWrite(SL_Motor_Out, LOW);

  float startPos = getSlitPosition();
  float targetPos;
  Serial.print("Initial position: ");
  Serial.println(startPos);

  if (dir=="IN"){
    targetPos = startPos + x; //moving in -> x increases
    digitalWrite(SL_Motor_In, HIGH);
  }
  else if (dir == "OUT"){
    targetPos = startPos - x;
    digitalWrite(SL_Motor_Out, HIGH);
  }
  else{
    Serial.println("Provided Invalid movement direction");
    return;
  }

  Serial.print("Starting move ");  Serial.print(dir);  Serial.print(" from ");
  Serial.print(startPos);  Serial.print(" mm to ");  Serial.print(targetPos);  Serial.println(" mm");

  while (true) {
    float currPos = getSlitPosition();
    Serial.println(currPos);
    // Check if target reached
    if ((dir == "IN" && currPos >= targetPos) ||
        (dir == "OUT" && currPos <= targetPos)) {
      Serial.println("Target reached.");
      break;
    }

    // Check for STOP command
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input == "SL_STOP") {
        Serial.println("Movement interrupted by STOP command.");
        break;
      }
    }
    // Limit switch check
    if (limitSwitchHit(dir)) {
      Serial.println("Movement stopped due to limit switch.");
      break;
    }
    delay(10); 
  }
  // Stop motors
  digitalWrite(SL_Motor_In, LOW);
  digitalWrite(SL_Motor_Out, LOW);
}