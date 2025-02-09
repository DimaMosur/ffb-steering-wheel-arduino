#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

#include <Arduino.h>
// Mapping of analog pins as digital I/O
// A6-A11 share with digital pins 
// from pins_arduino.h
// #define PIN_A0   (18)
// #define PIN_A1   (19)
// #define PIN_A2   (20)
// #define PIN_A3   (21)
// #define PIN_A4   (22)
// #define PIN_A5   (23)
// #define PIN_A6   (24)
// #define PIN_A7   (25)
// #define PIN_A8   (26)
// #define PIN_A9   (27)
// #define PIN_A10  (28)
// #define PIN_A11  (29)

#define POT_THR A0    //  gas btn
#define POT_BREAK_PIN A1    // stop btn
#define POT_CLUTCH_PIN A2   // clutch btn

#define POT_JOY_X A3 // джойстик Y
#define POT_JOY_Y A4 // джойстик Y
#define POT_WHEEL_PIN A5

#define BUTT_HANDLEBRAKE_PIN 4  // handbrake 
#define BUTT_CALIB_PIN 3 // calibration button

#define MAIN_PWM_PIN 8
#define L_PWM_PIN 9
#define R_PWM_PIN 10

void setupPinsInput()
{
    pinMode(BUTT_HANDLEBRAKE_PIN, INPUT_PULLUP);
    pinMode(BUTT_CALIB_PIN, INPUT_PULLUP);
}

void setupPinsOutput()
{
    pinMode(MAIN_PWM_PIN, OUTPUT);
    pinMode(L_PWM_PIN, OUTPUT);   
    pinMode(R_PWM_PIN, OUTPUT);  
}

#endif // PINS_CONFIG_H