#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

// ADC (line sensor)
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

int adc1_buf[8];
int adc2_buf[8];

uint8_t lineArray[13]; 

// Encoders
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

// Motors
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

const unsigned int PWM_MAX = 255;
const int freq = 5000;
const int resolution = 8; // 8-bit resolution -> PWM values go from 0-255

// LED
const int ledChannel = 0;

// PID
const int base_pid = 80; // Base speed for robot
const float mid = 6;

float e;
float d_e;
float total_e;

// Assign values to the following feedback constants:
float Kp = 2;
float Kd = 70;
float Ki = .01;

/*
 *  Line sensor functions
 */
void readADC() {
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
}

// Converts ADC readings to binary array lineArray[] (Check threshold for your robot) 
void digitalConvert() {
  int threshold = 700;
  for (int i = 0; i < 7; i++) {
    if (adc1_buf[i]>threshold) {
      lineArray[2*i] = 0; 
    } else {
      lineArray[2*i] = 1;
    }

    if (i<6) {
      if (adc2_buf[i]>threshold){
        lineArray[2*i+1] = 0;
      } else {
        lineArray[2*i+1] = 1;
      }
    }

    // print line sensor position
    // for(int i = 0; i < 13; i++) {
    //   Serial.print(lineArray[2*i+1]); Serial.print(" ");
    // }
  }
}

// Calculate robot's position on the line 
// When calling getPosition, whiteLine = true means white line on black background, whiteLine = false means black line on white background
float getPosition(bool whiteLine) {
  // This portion of code sets up the detection algorithm with the correct orientation of line color (depending on input)
  int lineValue = 0;
  if (whiteLine) {
    lineValue = 1;
  } 

  int start_of_line = -1;
  int line_count = 0;

  for(int i = 0; i < 13; i++) { // Iterate through the lineArray
    readADC();
    digitalConvert();
    if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
      if(start_of_line == -1) { // Set the start_of_line value to the first value detected
        start_of_line = i;
      }
      line_count += 1; // Increment line count by one
    }
  }
  return start_of_line + (line_count / 2); // Return the detected center of the line
}

// Function not yet finished
bool turnDirection(bool whiteLine) {
  int lineValue = 0;
  if (whiteLine) {
    lineValue = 1;
  } 

  int line_count = 0;
  for(int i = 0; i < 13; i++) { // Iterate through the lineArray
    readADC();
    digitalConvert();
    if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
      line_count += 1; // Increment line count by one
    }
  }
}

/*
 *  Movement functions
 */
void M1_forward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}
void M2_forward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
}

void M1_backward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, pwm_value);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}
void M2_backward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, pwm_value);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M1_IN_2_CHANNEL, PWM_MAX);
}
void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M2_IN_2_CHANNEL, PWM_MAX);
}

// When calling turnCorner, right = true, left = false
void turnCorner(bool turningRight, int leftStart, int rightStart) {
  /* 
   * Use the encoder readings to turn the robot 90 degrees clockwise or 
   * counterclockwise depending on the argument. You can calculate when the 
   * robot has turned 90 degrees using either the IMU or the encoders + wheel measurements
   */
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  while (true) {
    if (turningRight) {
      M1_forward(80);
      M2_backward(100);
      if (abs(enc2.read() - rightStart) > 90) {
        return;
      }
    } else {
      M1_forward(80);
      M2_backward(100);
      if (abs(enc2.read() - rightStart) > 90) {
        return;
      }
    }
  }
}

/*
 *  setup and loop
 */
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

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

  M1_stop();
  M2_stop();

  delay(100);
}

void loop() {

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  while(true) {

    int u;
    int rightWheelPWM;
    int leftWheelPWM;
    float pos;

    readADC();
    digitalConvert();

    pos = getPosition(true);
    
    // Define the PID errors
    int e = mid - pos;
    int prev_e = e;
    int d_e = e - prev_e; 
    int total_e;
    if (pos > -1) {
      total_e = total_e + prev_e;
    }

    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    u = Kp * e + Kd * d_e + Ki * total_e;

    // If no line was detected, set u to 0 since pos defaults to -1 if nothing detected
    if (pos == -1) {
      u = 0;
    } 
  
    rightWheelPWM = 80;
    leftWheelPWM = 100;
    int leftMotorSpeed; 
    int rightMotorSpeed;

    leftMotorSpeed = rightWheelPWM - u;
    rightMotorSpeed = rightWheelPWM + u;

    // Ensure that the motor speeds are not negative or over the PWM limit
    if (leftMotorSpeed > 120) {
      leftMotorSpeed = 120;
    } else if (leftMotorSpeed < 50) {
      leftMotorSpeed = 50;
    }

    if (rightMotorSpeed > 120) {
      rightMotorSpeed = 120;
    } else if (rightMotorSpeed < 50) {
      rightMotorSpeed = 50;
    }

    // Run the motors
    M1_forward(rightMotorSpeed);
    M2_forward(leftMotorSpeed);

    // Print values
    // Serial.print("\nPos value: ");
    // Serial.print(pos);
    // Serial.print("\nU value: ");
    // Serial.print(u);
    // Serial.print("\nRight motor: ");
    // Serial.print(rightMotorSpeed);
    // Serial.print("\nLeft motor: ");
    // Serial.print(leftMotorSpeed);
    // Serial.print("\n");
    //delay(1000);
    

    // Check for corners
    if(pos == -1) {
      turnCorner(true, enc1.read(), enc2.read());
    }

  }
}
