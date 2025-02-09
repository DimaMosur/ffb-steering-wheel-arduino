#include <Arduino.h>
#include <Joystick.h>
#include "pinsConfig.h"
#include "eepromLogic.h"

#define DEBUG_MODE 1

// X-axis & Y-axis REQUIRED
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 4, 0,
                   true, true, true, // X,Y,Z
                   true, true, true, // Rx,Ry,Rz
                   false, false, false, false, false);

Gains mygains[2] = {0}; //check in platfornmio file
EffectParams myeffectparams[2] = {0};
int32_t forces[2] = {0};

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println(F("\nSetup started"));

    debug();
    calibration();
    getConfig();
    
    Joystick.setGains(mygains);
    Joystick.begin();

    Serial.println(F("\nSetup end"));
}

void loop()
{
    static uint32_t timerNow = millis();
    debug();
    int wheel, thr, br, cth, joyx, joyy;
    wheel = map(analogRead(POT_WHEEL_PIN), configMain.wheelMin, configMain.wheelMax, 0, UINT16_MAX);
    wheel = constrain(wheel, 0, ADC_MAX);

    myeffectparams[0].springMaxPosition = ADC_MAX;
    myeffectparams[0].springPosition = wheel;

    myeffectparams[0].damperMaxVelocity = ADC_MAX;
    myeffectparams[0].damperVelocity = wheel;

    myeffectparams[0].inertiaMaxAcceleration = ADC_MAX;
    myeffectparams[0].inertiaAcceleration = wheel;

    myeffectparams[0].frictionMaxPositionChange = ADC_MAX;
    myeffectparams[0].frictionPositionChange = wheel;

    // todo for X?
    Joystick.setXAxis(wheel);

    thr = map(analogRead(POT_THR), throttleMin, throttleMax, 0, ADC_MAX);
    thr = constrain(thr, 0, ADC_MAX);
    Joystick.setYAxis(thr);

    br = map(analogRead(POT_BREAK_PIN), brakeMin, brakeMax, 0, ADC_MAX);
    br = constrain(br, 0, ADC_MAX);
    Joystick.setZAxis(br);

    cth = map(analogRead(POT_CLUTCH_PIN), clutchMin, clutchMax, 0, ADC_MAX);
    cth = constrain(cth, 0, ADC_MAX);
    Joystick.setRxAxis(cth);

    joyx = map(analogRead(POT_JOY_X), joystickXMin, joystickXMax, 0, ADC_MAX);
    joyx = constrain(joyx, 0, ADC_MAX);
    Joystick.setRyAxis(joyx);

    joyy = map(analogRead(POT_JOY_Y), joystickYMin, joystickYMax, 0, ADC_MAX);
    joyy = constrain(joyy, 0, ADC_MAX);
    Joystick.setRzAxis(joyy);

    Joystick.setEffectParams(myeffectparams);
    Joystick.getForce(forces);
    Joystick.sendState();

    if (forces[0])
    {
        digitalWrite(L_PWM_PIN, LOW);
        digitalWrite(R_PWM_PIN, HIGH);
        analogWrite(MAIN_PWM_PIN, abs(forces[0]));
    }
    else
    {
        digitalWrite(L_PWM_PIN, HIGH);
        digitalWrite(R_PWM_PIN, LOW);
        analogWrite(MAIN_PWM_PIN, abs(forces[0]));
    }
  
}

void calibration() // todo make struct type for configs
{
    if (!digitalRead(BUTT_CALIB_PIN))
    {
        Serial.print(F("Calibration start"));
        ConfigEE reConfig;
        reConfig.throttleMin = analogRead(POT_THR);
        reConfig.brakeMin = analogRead(POT_BREAK_PIN);
        reConfig.clutchMin = analogRead(POT_CLUTCH_PIN);
        reConfig.clutchMin = analogRead(POT_WHEEL_PIN);
        reConfig.joystickXMin = analogRead(POT_JOY_X);
        reConfig.joystickYMin = analogRead(POT_JOY_Y);

        uint32_t timerLoopStarted = millis();
        while (true && millis() - timerLoopStarted < 30000)
        {
            if (!digitalRead(BUTT_CALIB_PIN))
            {
                Serial.println(F("Calibration end BUTT_CALIB_PIN"));
                break;
            }
            reConfig.throttleMax = analogRead(POT_THR);
            reConfig.brakeMax = analogRead(POT_BREAK_PIN);
            reConfig.clutchMax = analogRead(POT_CLUTCH_PIN);
            reConfig.wheelMax = analogRead(POT_WHEEL_PIN);
            reConfig.joystickXMax = analogRead(POT_JOY_X);
            reConfig.joystickYMax = analogRead(POT_JOY_Y);
        }

        //         EEPROM.put(0, zeroTHR);
        //         EEPROM.put(2, zeroBR);
        //         EEPROM.put(4, zeroCTH);
        //         EEPROM.put(12, zeroWHEEL);
        //         EEPROM.put(14, zeroJOY_X);
        //         EEPROM.put(18, zeroJOY_Y);
        // //        delay(100); // дебаунс

        //         EEPROM.put(6, maxTHR);
        //         EEPROM.put(8, maxBR);
        //         EEPROM.put(10, maxCTH);
        //         EEPROM.put(16, maxJOY_X);
        //         EEPROM.put(20, maxJOY_Y);
        //         EEPROM.put(22, maxWHEEL);

        EEPROM.put(EE_BASE_ADDR, reConfig);
        configMain = reConfig;
    }
    Serial.print(F("Calibration start"));
    // Serial.end();
    //!!!!    delay(3000); // задержка чтобы кнопку отпустить
}

void debug()
{
#if (DEBUG_MODE == 1)

    static uint32_t timerDebug = millis();
    if (timerNow - timerDebug > 1000)
    {
        timerDebug = millis();
        Serial.print(F("Wheel: "));
        Serial.print(analogRead(POT_WHEEL_PIN));
        Serial.print(F("\tThrottle: "));
        Serial.print(analogRead(POT_THR));
        Serial.print(F("\tBrake: "));
        Serial.print(analogRead(POT_BREAK_PIN));
        Serial.print(F("\tClutch: "));
        Serial.print(analogRead(POT_CLUTCH_PIN));
        Serial.print(F("\tJoystick X: "));
        Serial.print(analogRead(POT_JOY_X));
        Serial.print(F("\tJoystick Y: "));
        Serial.print(analogRead(POT_JOY_Y));
        Serial.print(F("\tHandbrake: "));
        Serial.print(!digitalRead(BUTT_HANDLEBRAKE_PIN));
        Serial.print(F("\tCalibration: "));
        Serial.println(!digitalRead(BUTT_CALIB_PIN));
    }

#endif
}
