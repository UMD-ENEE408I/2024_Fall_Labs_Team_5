#include <Arduino.h>

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 4;
const unsigned int M1_IN_2_CHANNEL = 2;
const unsigned int M2_IN_1_CHANNEL = 11;
const unsigned int M2_IN_2_CHANNEL = 5;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const unsigned int PWM_VALUE = 512; // Do not give max PWM. Robot will move fast

const int freq = 5000;
const int resolution = 10;


void setup() {
  Serial.begin(115200);
  
  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);
}

void loop() {
  // Rotate Forward
  Serial.print("Going Forward \n");
  analogWrite(M1_IN_1, 100);
  analogWrite(M1_IN_2, 0);
  analogWrite(M2_IN_1, 100);
  analogWrite(M2_IN_2, 0);
  delay(1000);
  // Rotate Backwards
  Serial.print("Going Backwards \n");
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 100);
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 100);
  delay(1000);
}
