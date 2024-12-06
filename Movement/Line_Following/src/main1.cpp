#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>
#include <WiFi.h>

struct __attribute__((packed)) Data {
    int16_t seq;     // sequence number
    int32_t distance; // distance
    float voltage;   // voltage
    char text[10];   // text
};

// WiFi network credentials
// const char* ssid = "ENEE408I";
// const char* password = "ENEE408I";

const char* ssid = "iPhone";
const char* password = "rozrozroz";

// const char* ssid = "Karen's Phone";
// const char* password = "dingus1999";

//const char* ssid = "rozwifi";
//const char* password = "rambutan1to5";


// // Server IP and port
const char* host = "172.20.10.11";  // jetson ip
const uint16_t port = 10000;

// Create a client
WiFiClient client;

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

bool old_rat;

float Kp;
float Kd;
float Ki;

// Checkpoint variables
int checkpoint_counter = 0;
int checkpoint_turns[3][3]; // For this 2D array, values of 0 represent no turn, while 1 represents a possible turn

// Maze variables
// This array is to be updated in the run of the first robot, and utilized by the second
int maze_path[15]; // This array has a max of 15 ints to represent what to do at each possible turn

int maze_colors[3];
int common_color = 0; // This represents the most common color, 0 = default, 1 = red, 2 = green, 3 = blue, 4 = other (purple, yellow, etc.)


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
void digitalConvert(bool whiteLine) {
  int threshold = 700;
    if(!whiteLine) {
      threshold = 650;
    }
  for (int i = 0; i < 7; i++) {
    // Serial.print(adc1_buf[i]);
    // Serial.print(" ");

    // if (adc1_buf[i] < threshold) {
    //     Serial.print(1);
    //   } else {
    //     Serial.print(0);
    //   }
    if (adc1_buf[i] >= threshold) {
      lineArray[2*i] = 0; 
    } else {
      lineArray[2*i] = 1;
    }

    if (i < 6) {
      // Serial.print(adc2_buf[i]);
      // Serial.print(" ");  

      // if (adc2_buf[i] < threshold) {
      //   Serial.print(1);
      // } else {
      //   Serial.print(0);
      // }
      if (adc1_buf[i] >= threshold) {
        lineArray[2*i+1] = 0;
      } else {
        lineArray[2*i+1] = 1;
      }
    }
  }
  // print line sensor position
  // Serial.print("\n");
  // for(int i = 0; i < 13; i++) {
  //   if(i > 1) {
  //     Serial.print(lineArray[i]); Serial.print(" ");
  //   }
  // }
  // Serial.print("\n");
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
    digitalConvert(whiteLine);
    //Serial.println("In get pos");
    if(i > 1) {
      if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
        if(start_of_line == -1) { // Set the start_of_line value to the first value detected
          start_of_line = i;
        }
        line_count += 1; // Increment line count by one
      }
    }
  }
  //Serial.println("If i see this last then int -> float is the problem");
  //return start_of_line + (line_count / 2); // Return the detected center of the line

  return start_of_line + ((line_count + 1) / 2); // Return the detected center of the line
}

// This decides what direction to turn based on the most prominent color detected
// Returning 1 = left, 2 = forward, 3 = right, 0 = default no value
int turnJunction(int color) {
  // This function takes an input of the most common detected color

  // Use machine learning to see all possible paths and their respective colors

  // Return the direction to be turned
  return 1;
}

// Determines which direction to turn, true = right, false = left
bool turnDirection(bool whiteLine) {
  // This portion of code sets up the detection algorithm with the correct orientation of line color (depending on input)
  int lineValue = 0;
  if (whiteLine) {
    lineValue = 1;
  } 

  int line_start = 0;
  for(int i = 0; i < 13; i++) { // Iterate through the lineArray
    readADC();
    digitalConvert(whiteLine);
    if(i > 1) {
      if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
        line_start = i; // Find the line start for turning purposes
        break;
      }
    }
  }
  if (line_start <= 3) {
    return true; //turn right
  } else {
    return false; //turn left
  }
}

// Determines if the robot should turn based off the amount of detected lines.
bool should_turn(bool whiteLine) {
  // This portion of code sets up the detection algorithm with the correct orientation of line color (depending on input)
  int lineValue = 0;
  if (whiteLine) {
    lineValue = 1;
  } 

  int line_count = 0;

  for(int i = 0; i < 13; i++) { // Iterate through the lineArray
    readADC();
    digitalConvert(whiteLine);
    if(i > 1) {
      if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
        line_count += 1; // Increment line count by one
      }
    }
  }

  if (line_count >= 6 && line_count < 10) {
    return true; // Since the majority of the line sensors detected a line, turn!
  } else {
    return false; // Don't turn
  }
}

// This function returns true if the rat is within a checkpoint by determining if all values of the line sensor return positive
bool in_checkpoint(bool whiteLine) {
  // This portion of code sets up the detection algorithm with the correct orientation of line color (depending on input)
  int lineValue = 0;
  if (whiteLine) {
    lineValue = 1;
  } 

  int line_count = 0;

  for(int i = 0; i < 13; i++) { // Iterate through the lineArray
    readADC();
    digitalConvert(whiteLine);
    if(i > 1) {
      if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
        line_count += 1; // Increment line count by one
      }
    }
  }

  if (line_count >= 10) {
    return true; // We are in the checkpoint
  } else {
    return false; // Not in the checkpoint
  }
}

bool start_turn(bool whiteLine) {
  // This portion of code sets up the detection algorithm with the correct orientation of line color (depending on input)
  int lineValue = 0;
  if (whiteLine) {
    lineValue = 1;
  } 

  int line_count = 0;

  for(int i = 0; i < 13; i++) { // Iterate through the lineArray
    readADC();
    digitalConvert(whiteLine);
    if(i > 1) {
      if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
        line_count += 1; // Increment line count by one
      }
    }
  }

  if (line_count >= 5) {
    return true; // Since the majority of the line sensors detected a line, turn!
  } else {
    return false; // Don't turn
  }
}

bool detect_junction(bool whiteLine) {
  // This portion of code detects when a junction is seen based on the line portions
  int lineValue = 0;
  if (whiteLine) {
    lineValue = 1;
  } 

  bool gap_detected = false;
  bool line_detected = false;

  for(int i = 0; i < 13; i++) { // Iterate through the lineArray
    readADC();
    digitalConvert(whiteLine);
    if(i > 1) {
      if(lineArray[i] == lineValue) { // Check each value to see if it detects a line
        if(i < 4 ||  i > 9) {
          line_detected = true;
        } else {
          return false;
        }
      } else {
        if (i > 4 &&  i < 9) {
          gap_detected = true;
        }
      }
    }
  }

  if(line_detected && gap_detected) {
    return true;
  }
  return false;
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

void line_follow(bool white_line) {
  //Serial.println("Start of line_follow");
  int u;
  int rightWheelPWM;
  int leftWheelPWM;
  float pos;

  readADC();
  digitalConvert(white_line);

  //Serial.println("Line follower values gotted");

  pos = getPosition(white_line);

  //Serial.println("Start PID");

  // Define the PID errors
  int prev_e = e;
  int e = mid - pos;
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

  // New RAT
  rightWheelPWM = 100;
  leftWheelPWM = 90;

  int leftMotorSpeed; 
  int rightMotorSpeed;

  leftMotorSpeed = leftWheelPWM - u;
  rightMotorSpeed = rightWheelPWM + u;

  // Ensure that the motor speeds are not negative or over the PWM limit
  if (leftMotorSpeed > 130) {
    leftMotorSpeed = 130;
  } else if (leftMotorSpeed < 50) {
    leftMotorSpeed = 50;
  }

  if (rightMotorSpeed > 130) {
    rightMotorSpeed = 130;
  } else if (rightMotorSpeed < 50) {
    rightMotorSpeed = 50;
  }

  // Run the motors
  M1_forward(rightMotorSpeed);
  M2_forward(leftMotorSpeed);
  //Serial.println(u);
  delay(30);
  M1_stop();
  M2_stop();
  delay(1);
}

void turnCorner(bool turningRight, int leftStart, int rightStart) {
  /* 
   * Use the encoder readings to turn the robot 90 degrees clockwise or 
   * counterclockwise depending on the argument. You can calculate when the 
   * robot has turned 90 degrees using either the IMU or the encoders + wheel measurements
   */
  // Encoder enc1(M1_ENC_A, M1_ENC_B);
  // Encoder enc2(M2_ENC_A, M2_ENC_B);

  Serial.print("\nStarting Turn!");

  while (true) {
    if (turningRight) {
      if (old_rat) {
        // Old Rat
        M1_forward(85);
        M2_backward(85);
      } else {
        // New Rat
        M1_forward(85);
        M2_backward(85);
      }

      delay(300);
      return;
      
      // if (abs(enc2.read() - rightStart) > 2) {
      //   //Serial.println("Turn ended");
      //   return;
      // }
    } else {

      if (old_rat) {
        // Old Rat
        M1_backward(90);
        M2_forward(90);
      } else {
        // New Rat
        M1_backward(90);
        M2_forward(90);
      }

      delay(300);
      return;

      // if (abs(enc2.read() - rightStart) > 2) {
      //   //Serial.println("Turn ended");
      //   return;
      // }
    }
  }
}

// This function detects where the audio comes from and what frequency the detected audio is
void followAudio() {
  // First detect where the audio is coming from
  // If the audio is detected to be a word (either "left" or "right") turn to follow that word
  // Other wise, simply go where the audio is detected
  return;
}

// This section is to follow the dashed line section of the course
void dashedLine(bool white_line) {
  while(!in_checkpoint(white_line)) {
    line_follow(white_line);
  }
}

// This section is for solving the maze portion of the course
void solveMaze(int color) {
  while(true) {
    // This function takes an input of the most common detected color
    detect_junction(true);
    // follow line

    // If line suddenly ends, junction found, run turnJunction function and turn accordingly, then repeat loop
    // Be sure to save the value of each turn for the baby rat to solve the maze
    // Since there is a maximum of 15 possible turns, these values will be stored in an array of length 15 with default values of 0

    // If a standard turn was detected, maze has been solved, exit function (and we're done :) )
  }
  return;
}

void sendPkt(const char* msg) {
  // Prepare data packet
  Data data;
  data.seq = 1;
  data.distance = 1000;
  data.voltage = 3.7f;
  strncpy(data.text, msg, sizeof(data.text) - 1);
  data.text[sizeof(data.text) - 1] = '\0'; // Ensure null termination
  Serial.printf("seq %d distance %ld voltage %f text %s\n", data.seq, data.distance, data.voltage, data.text);

  // Check if connected to the server
  if (client.connected()) {
    // Read server's response (if any)
    while (client.available()) {
      Data response;
      client.readBytes((char*)&response, sizeof(response)); // Read data from the server and unpack it into the response struct
      Serial.printf("seq %d distaace %ld voltage %f text %s\n", (int)response.seq, (long)response.distance, response.voltage, response.text);
    }

    // Send data to the server
    client.write((uint8_t*)&data, sizeof(data));

    // Increment sequence number for the next packet and add a delay between messages
    data.seq++;
    delay(5000); // Send data every 5 seconds
  } else {
    Serial.println("Disconnected from server.");
    client.stop();
  }
}

/*
 *  setup and loop
 */
void setup() {
  Serial.begin(115200);
  int wifi_timer = 0;

    // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && wifi_timer < 10) {
    delay(500);
    Serial.println("Connecting to WiFi...");
    wifi_timer = wifi_timer + 1;
  }
  Serial.println("Connected to WiFi!");

  // Connect to the server
  if (client.connect(host, port)) {
    Serial.println("Connected to server!");
  } else {
    Serial.println("Connection to server failed.");
    //return;
  }

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

  // For testing things other than the motors, don't run the line following code if testing = true
  bool testing = false;

  // Differet robots have different PID values and motor encoders
  bool old_rat = true;

  // Change this depending on white line on black background (true) or black line on white background (false)
  bool white_line = true;

  char msg[] = "rat time";
  sendPkt(msg);

  int checkpointCounter = 0;

  // Assign values to the following feedback constants:
  if (old_rat) {
    // OLD RAT
    Serial.print("Old rat");
    Kp = 2;
    Kd = 50;
    Ki = .01;
  } else {
    // NEW RAT
    Serial.print("New rat");
    Kp = 2;
    Kd = 70;
    Ki = .01; 
  }

  if (testing) {
    //dashedLine(white_line);
    while(true) {
      //sendPkt(msg);
      delay(1000);
      M1_stop();
      M2_stop();
    }
  }

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  while(true) {
    //Serial.println("Start of loop");

    // int u;
    // int rightWheelPWM;
    // int leftWheelPWM;
    // float pos;

    // readADC();
    // digitalConvert(white_line);

    // pos = getPosition(white_line);
    
    // // Define the PID errors
    // int e = mid - pos;
    // int prev_e = e;
    // int d_e = e - prev_e; 
    // int total_e;
    // if (pos > -1) {
    //   total_e = total_e + prev_e;
    // }

    // // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    // u = Kp * e + Kd * d_e + Ki * total_e;

    // // If no line was detected, set u to 0 since pos defaults to -1 if nothing detected
    // if (pos == -1) {
    //   u = 0;
    // } 
  
    // if (old_rat) {
    //   // OLD RAT
    //   rightWheelPWM = 85;
    //   leftWheelPWM = 85;
    // } else {
    //   // NEW RAT
    //   rightWheelPWM = 85;
    //   leftWheelPWM = 85;
    // }

    // int leftMotorSpeed; 
    // int rightMotorSpeed;

    // leftMotorSpeed = leftWheelPWM - u;
    // rightMotorSpeed = rightWheelPWM + u;

    // // Ensure that the motor speeds are not negative or over the PWM limit
    // if (leftMotorSpeed > 120) {
    //   leftMotorSpeed = 120;
    // } else if (leftMotorSpeed < 50) {
    //   leftMotorSpeed = 50;
    // }

    // if (rightMotorSpeed > 120) {
    //   rightMotorSpeed = 120;
    // } else if (rightMotorSpeed < 50) {
    //   rightMotorSpeed = 50;
    // }

    // // Run the motors
    // M1_forward(rightMotorSpeed);
    // M2_forward(leftMotorSpeed);

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

    line_follow(white_line);
    //Serial.println("Passed line follow");

    // Check for corners, and turn accordingly
    // if(should_turn(white_line)) {
    //   //Serial.println("Turn detected");
    //   //delay(5000);
    //   bool turn_direction = turnDirection(white_line);
    //   // Once we detect a turn, drive forwards

    //   if (old_rat) {
    //   // OLD RAT
    //     M1_forward(90);
    //     M2_forward(90);
    //   } else {
    //   // NEW RAT
    //     M1_forward(90);
    //     M2_forward(90);
    //   }

    //   while(!start_turn(white_line)) {
    //     M1_forward(90);
    //     M2_forward(90);
    //     delay(10);
    //     M1_stop();
    //     M2_stop();
    //     delay(10);
    //   }
    //   turnCorner(turn_direction, enc1.read(), enc2.read());
    //   //Serial.println("We turned lets go");

    // // This portion outlines what to do when a checkpoint is detected
    // } else 
    if(in_checkpoint(white_line)) {
      checkpointCounter = checkpointCounter + 1;

      if(checkpointCounter == 1) {
        // Color detection aaaaaaaaaaa
      }

      if(checkpointCounter == 2) {
        // AUDIO PART AAAAAAAAAAAAAAAAAAAAAAAA
      }

      if(checkpointCounter == 100) {
        // Maze part
      }

      delay(1000);


      if (old_rat) {
      // OLD RAT
        M1_forward(90);
        M2_forward(90);
      } else {
      // NEW RAT
        M1_forward(90);
        M2_forward(90);
      }
      delay(250);
      turnCorner(false, enc1.read(), enc2.read());

      while(true) {
        line_follow(white_line);
        //Serial.println(getPosition(white_line));
        if(getPosition(white_line) == -1) {
          if (old_rat) {
          // OLD RAT
            M1_forward(90);
            M2_forward(90);
          } else {
          // NEW RAT
            M1_forward(90);
            M2_forward(90);
          }
          delay(300);
          turnCorner(true, enc1.read(), enc2.read());
        }
        if(in_checkpoint(white_line)) {
          if (old_rat) {
          // OLD RAT
            M1_forward(90);
            M2_forward(90);
          } else {
          // NEW RAT
            M1_forward(90);
            M2_forward(90);
          }
          delay(300);
          turnCorner(false, enc1.read(), enc2.read());
          break;
        }
      }
      //Serial.println("Found a checkpoint AND turned wow");
      // while(true) {
      //   Serial.print("We be stuck here");
      //   // Initially turn left to start detecting turns
      //   turnCorner(false, enc1.read(), enc2.read());
      //   // Drive until there is no more line
      //   while(getPosition(white_line) != -1) {
      //     line_follow(white_line);
      //   }

      //   // Turn right, check side 1 of checkpoint
      //   turnCorner(true, enc1.read(), enc2.read());
      //   // Drive until there is no more line, checking for turns along the way
      //   while(getPosition(white_line) != -1) {
      //     line_follow(white_line);
      //     if(should_turn(white_line)) {
      //       checkpoint_turns[checkpoint_counter][0] = 1;
      //     }
      //   }

      //   // Turn right, check side 2 of checkpoint
      //   turnCorner(true, enc1.read(), enc2.read());
      //   // Drive until there is no more line, checking for turns along the way
      //   while(getPosition(white_line) != -1) {
      //     line_follow(white_line);
      //     if(should_turn(white_line)) {
      //       checkpoint_turns[checkpoint_counter][1] = 1;
      //     }
      //   }

      //   // Turn right, check side 3 of checkpoint
      //   turnCorner(true, enc1.read(), enc2.read());
      //   // Drive until there is no more line, checking for turns along the way
      //   while(getPosition(white_line) != -1) {
      //     line_follow(white_line);
      //     if(should_turn(white_line)) {
      //       checkpoint_turns[checkpoint_counter][2] = 1;
      //     }
      //   }

      //   // Back to original line, go until line is detected, the turn to face center
      //   turnCorner(true, enc1.read(), enc2.read());
      //   // Until we detect the turn we arrived from, follow line
      //   while(!should_turn(white_line)) {
      //     line_follow(white_line);
      //   }
      //   turnCorner(true, enc1.read(), enc2.read());

      //   M1_stop();
      //   M2_stop();
      //   checkpoint_counter += 1;
      //   break;
      // }
    }

    // This section is for the audio portion of the maze
    while(true) {
      break;
      // Update this maze_color value maze_colors[0] = 0;

      // Call the function to correctly determine where to turn (based off audio input)
      followAudio();
    }

    // This section is for the dashed line following portion
    while(true) {
      break;
      // Update this maze_color value maze_colors[1] = 0;

      // Call the function to follow the dashed line (might need object detection for assistance)
      dashedLine(white_line);
      break;
      // Update this maze_color value maze_colors[2] = 0;
    }

    // This section is for the color based maze portion
    while(true) {
      break;
      // Call the color based maze algorithm, the input is an int representing the most common color
      // 1 = red, 2 = green, 3 = blue, 4 = other
      solveMaze(common_color);
    }

    //Serial.println("End of loop");
  }
}
