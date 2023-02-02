/**
 * @file motor-angular-rate.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to estimate motor speed from encoder.
 * @version 2.0
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

#include <Arduino_LSM6DS3.h>

#define EA 11  // Wheel PWM pin (must be a PWM pin)
#define I1 8   // Wheel direction digital pin 1
#define I2 7   // Wheel direction digital pin 2
#define EB 9
#define I4 12
#define I3 13

const float dgx = 0.49, dgy = -0.18, dgz = -0.08;
float omega_x, omega_y, omega_z;

const float dx = 0.045, dy = -0.02, dz = -0.01;
float a_x, a_y, a_z;

// Variables to store sample rates from sensor [Hz]
float a_f, g_f;

// Motor driver PWM pin
const byte E1 = 6;

// Motor driver direction pin
const byte M1 = 7;

// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A = 13;
const byte SIGNAL_B = 12;

const byte SIGNAL_AR = 11;
const byte SIGNAL_BR = 10;

float k = 200;
float u_ms, u_last = 0.0;
float v_d = 0.5;
float v_a = 0;
float i_term = 0;
float k_cof = k/2;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;
const double l = 0.2775;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;
volatile long encoder_ticks_r = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double v_L = 0.0;
double omega_R = 0.0;
double v_R = 0.0;
double omega = 0.0;
double v = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 200;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;
long t_last_a = 0;
float a_last = 0.0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks() {
  if (digitalRead(SIGNAL_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks++;
  }
}

void decodeEncoderTicksR() {
  if (digitalRead(SIGNAL_BR) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_r--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_r++;
  }
}

void setup() {
  // Open the serial port at 9600 bps
  Serial.begin(115200);

  // Wait for serial connection before starting
  while (!Serial) {
    delay(10);
  }

  if (!IMU.begin()) {
    // Print an error message if the IMU is not ready
    Serial.print("Failed to initialize IMU :(");
    Serial.print("\n");
    while (1) {
      delay(10);
    }
  }

  a_f = IMU.accelerationSampleRate();
  g_f = IMU.gyroscopeSampleRate();

  // Set the pin modes for the motor driver
  pinMode(M1, OUTPUT);

  // Set the pin modes for the encoders
  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_AR, INPUT);
  pinMode(SIGNAL_BR, INPUT);

  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicksR, RISING);

  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");
}

void loop() {
  // Get the elapsed time [ms]
  t_now = millis();

  // if (t_now - t_last >= T) {
  // Estimate the rotational speed [rad/s]
  omega_L = -2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
  omega_R = 2.0 * PI * ((double)encoder_ticks_r / (double)TPR) * 1000.0 / (double)(t_now - t_last);

  v_L = RHO * omega_L;
  v_R = RHO * omega_R;
  omega = (v_R - v_L) / l;
  v = .5 * (v_L + v_R);

  // Serial.print("Estimated turning speed (encoder): ");
  // Serial.print(omega);
  // Serial.print(" rad/s");
  // Serial.print("\n");


  // Record the current time [ms]
  t_last = t_now;

  // Reset the encoder ticks counter
  encoder_ticks = 0;
  encoder_ticks_r = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(a_x, a_y, a_z);
    a_x -= dx;
    a_y -= dy;
    a_z -= dz;

    v_a += (t_last_a-t_now)*(a_last-a_x)/2;
    a_last = a_x;
    t_last_a = t_now;

// current time = t_now, last time = t_last
// current acc = a_x, last acc = ???    

  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(omega_x, omega_y, omega_z);
    omega_x -= dgx;
    omega_y -= dgy;
    omega_z -= dgz;

  }
  // }

  // Set the wheel motor PWM command [0-255]
  i_term += k_cof*(v_d - v_a);
  u_ms = k * (v_d - v_a) + i_term;
  u = u_ms * 318.75 * v_d;

  if (u > 255) u = 255;
  if (u < -255) u = -255;
  Serial.print("u=");
  Serial.println(u);


  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);

  analogWrite(EA, u);
  analogWrite(EB, u);

  delay(10);

  // if(u > u_last - 50){
  //   u = u_last + 50;
  // }
  // u_last = u;

  // Write to the output pins
  // digitalWrite(M1, LOW);  // Drive forward (left wheels)
  // analogWrite(E1, u);     // Write left motors command
}

float readEncoder() {
}

float readGyro() {
}

short P_controller(double k, double v_d, double v_m) {
  return k * (v_d - v_m);
}
