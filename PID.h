#pragma once
#include <Arduino.h>
double integral, derivative, previous, output;
integral = derivative = previous = output = 0;
double kp = 0.79;
double ki = 0.5075;
double kd = 0.015;
// double setpoint = 2.5;
double actual, error;
// double data[] = {1.5, 1.8, 1.9, 2.1, 2.3, 2.4, 2.6, 2.45, 2.55, 2.48, 2.51, 2.52, 2.47, 2.52, 2.49, 2.55, 2.53, 2.45, 2.47, 2.53, 2.49, 2.51, 2.45, 2.48, 2.5};


double pid(double error, double previous) {
  double proportional = error;
  integral += error * 0.001;  // Where dt = 1 sec / 1000
  derivative = (error - previous) / 0.001;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  //Serial.println(output);
  return output;
}

