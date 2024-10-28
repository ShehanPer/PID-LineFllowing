#include <Arduino.h>
//#include <Wire.h>
//#include <Adafruit_TCS34725.h>

#define M2f 25 // Motor 1 forward
#define M2b 24 // Motor 1 backward
#define M1_pwm 4 // Motor 1 PWM
#define M1f 27 // Motor 2 forward
#define M1b 26 // Motor 2 backward
#define M2_pwm 5  // Motor 2 PWM

//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};
bool calib_flag = false;
bool ref_color_set = false;
int calib_sum = 0;
int cal_array[numSensors];
int calibrated_value = 50;
//String read_color;
//String ref_color;

void IR_array(int array[], int &sum, int &w_sum, int &calibrated_value);
void mdrive(int m1, int m2);
void calibrate(int cal_array[], int &calibrated_value, int &calib_sum, bool &calib_flag);
//void get_color();
void turn_right(int array[]);
void turn_left(int array[]);
void m_stop();


void setup()
{
   Serial.begin(9600);
  // Set pins
  pinMode(M1f, OUTPUT);
  pinMode(M1b, OUTPUT);
  pinMode(M1_pwm, OUTPUT);
  pinMode(M2f, OUTPUT);
  pinMode(M2b, OUTPUT);
  pinMode(M2_pwm, OUTPUT);

  //color sensor
  // if (tcs.begin()) {
  //   Serial.println("Found sensor");
  // } else {
  //   Serial.println("No TCS34725 found ... check your connections");
  //   while (1);
  // }  
}
float kp = 7.4;
float kd = 0.00001;
float ki = 0; // Add integral term if needed
int last_err = 0;
int integral = 0; // Integral term accumulator
int count =0;

void loop(){

    int array[numSensors];
    int sum, w_sum;

    
    // Calculate PID terms
    
    //get_color();

    if (calib_flag == false) {
        calibrate(cal_array, calibrated_value, calib_sum, calib_flag);
        Serial.print("Calibrated value = ");
        Serial.println(calibrated_value);
    }
    IR_array(array, sum, w_sum, calibrated_value);

    if (array[0] == 1 && array[1] == 1 && array[2] == 1 && array[3] && array[4]==1 && array[5]==0 && array[6]==0 && array[7]==0) { // Condition for turning right at L-junction 
        m_stop();
        mdrive(80,80);
        delay(200);
        m_stop();

        if (array[0] == 1 && array[1] == 1 && array[2] == 1 && array[3] && array[4]==1 && array[5]==0 && array[6]==0 && array[7]==0) { // Condition for turning right at L-junction
          turn_right(array, sum, w_sum, calibrated_value);
          int error = w_sum;

      }
    } else if (array[0] == 0 && array[1] == 0 && array[2] == 0 && array[3] && array[4]==1 && array[5]==1 && array[6]==1 && array[7]==1) { // Condition for turning left at L-junction
        m_stop();
        mdrive(80,80);
        delay(200);
        m_stop();

        if (array[0] == 0 && array[1] == 0 && array[2] == 0 && array[3] && array[4]==1 && array[5]==1 && array[6]==1 && array[7]==1) { // Condition for turning left at L-junction
          turn_left(array, sum, w_sum, calibrated_value);
          int error = w_sum;
      }
    }
    else {
      int error = w_sum;
      integral += error; // Accumulate the integral term
      int derivative = error - last_err;
      int dift = kp * error + ki * integral + kd * derivative;
      int m1 =80;
      int m2 =90;
      error = last_err;
      mdrive(m1 + dift, m2 - dift);
      Serial.print(analogRead(A0));
      Serial.print(calibrated_value);
    }
    
}


// Read IR sensor array
void IR_array(int array[], int &sum, int &w_sum, int &calibrated_value) {
  sum = 0;
  w_sum = 0;

  // Read sensor values and populate the array
  for (int i = 0; i < numSensors; i++) {
    int val = analogRead(sensorPins[i]);
    array[i] = (val < calibrated_value) ? 1 : 0;
  }

  // Calculate sum and weighted sum
  for (int i = 0; i < numSensors; i++) {
    sum += array[i];
  }

  w_sum += array[7] * 5 + array[6] * 4 + array[5] * 3 + array[4] * 2 + array[3] * -2 + array[2] * -3 + array[1] * -4 + array[0] * -5;
  Serial.print("Weight_sum = ");
  Serial.println(w_sum);
}

// Motor control function
void mdrive(int m1, int m2) {
  
    // Motor 1 control
    if (m1 > 0) {
      if (m1 > 255) m1 = 255;
      digitalWrite(M2f, HIGH);
      digitalWrite(M2b, LOW);
      analogWrite(M2_pwm, m1);
    } else {
      if (m1 < 0) m1 = 0;
      digitalWrite(M2f,HIGH);
      digitalWrite(M2b, LOW);
      analogWrite(M2_pwm, m1);
    }
    // Motor 2 control
    if (m2 > 0) {
      if (m2 > 255) m2 = 255;
      digitalWrite(M1f, HIGH);
      digitalWrite(M1b, LOW);
      analogWrite(M1_pwm, m2);
    } else {
      if (m2 < 0) m2 =0;
      digitalWrite(M1f, HIGH);
      digitalWrite(M1b, LOW);
      analogWrite(M1_pwm, m2);
    }
  }


void calibrate(int cal_array[], int &calibrated_value,int &calib_sum, bool &calib_flag) {
    
    for (int i = 0; i < numSensors; i++) {
        int val = analogRead(sensorPins[i]);
        cal_array[i] = val;
    }

    for (int i = 0; i < numSensors; i++) {
        calib_sum += cal_array[i];
    }
    calibrated_value = calib_sum / numSensors + 20;
    calib_flag = true;
    delay(1000);
}

//read color sensor
// void get_color(){

 

//   uint16_t r, g, b, c;
//   tcs.getRawData(&r, &g, &b, &c);

//   // Set the reference color
  
//   // Normalize the RGB values
//   float red = r / (float)c;
//   float green = g / (float)c;
//   float blue = b / (float)c;

//   // Determine the color
//   if (red > 0.8 && green > 0.8 && blue > 0.8) {
//     Serial.println("Color: White");
//     read_color="W";
//   } else if (red > blue && red > green) {
//     Serial.println("Color: Red");
//     read_color="R";
//   } else if (blue > red && blue > green) {
//     Serial.println("Color: Blue");
//     read_color="B";
//   } else {
//     Serial.println("Color: Unknown");
//     read_color="U";
//   }
//   if (!ref_color_set) {
//     ref_color = read_color;
//     ref_color_set = true;
//   }

//   delay(1000); // Wait for a second before reading again
// }

void turn_right(int array[], int &sum, int &w_sum, int calibrated_value) {
    // Start turning right
    digitalWrite(M1b, HIGH); 
    digitalWrite(M1f,LOW); // Motor 1 backward
    digitalWrite(M2f, HIGH);
    digitalWrite(M2b,LOW);  // Motor 2 forward
    analogWrite(M1_pwm, 95); // Adjust speed if necessary
    analogWrite(M2_pwm, 75); // Adjust speed if necessary
    
    while (true) {
        IR_array(array, sum, w_sum, calibrated_value);
        // Assuming calibrated_value is the threshold to detect the line
        if (array[0]==0 && array[3]==1 && array[4]==1 && array[7]==0) {
            // Stop rotation when aligned with the line
            m_stop();
            delay(100);
            break;
        }
    }
}

void turn_left(int array[], int &sum, int &w_sum, int calibrated_value) {
    // Start turning left
    digitalWrite(M1f, HIGH);
    digitalWrite(M1b,LOW);  // Motor 1 forward
    digitalWrite(M2b, HIGH); 
    digitalWrite(M2f,LOW); // Motor 2 backward
    analogWrite(M1_pwm, 75); // Adjust speed if necessary
    analogWrite(M2_pwm, 95); // Adjust speed if necessary
    
    while (true) {
       
        IR_array(array, sum, w_sum, calibrated_value);
        // Assuming calibrated_value is the threshold to detect the line
        if (array[0]==0 && array[3]==1 && array[4]==1 && array[7]==0) {
            // Stop rotation when aligned with the line
            m_stop();
            delay(100);
           
            break;
        }
    }
}

void m_stop() {
    digitalWrite(M1f, LOW);
    digitalWrite(M1b, LOW);
    digitalWrite(M2f, LOW);
    digitalWrite(M2b, LOW);
    analogWrite(M1_pwm, 0);
    analogWrite(M2_pwm, 0);
}

