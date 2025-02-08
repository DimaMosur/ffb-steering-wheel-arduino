#ifndef EEPROM_LOGIC_H
#define EEPROM_LOGIC_H

#include <Arduino.h>
#include <EEPROM.h>

#define EE_BASE_ADDR 0
#define EE_CONF_END 24

#ifndef ADC_MAX
#define ADC_MAX 1023
#endif

// struct Gains {
//     // Define the structure of Gains
//     int gain1;
//     int gain2;
// };

// struct EffectParams {
//     // Define the structure of EffectParams
//     int springMaxPosition;
//     int springPosition;
//     int damperMaxVelocity;
//     int damperVelocity;
//     int inertiaMaxAcceleration;
//     int inertiaAcceleration;
//     int frictionMaxPositionChange;
//     int frictionPositionChange;
// };

typedef struct ConfigEE {
    int16_t throttleMin;
    int16_t throttleMax;
    int16_t brakeMin;
    int16_t brakeMax;
    int16_t clutchMin;
    int16_t  clutchMax;
    int16_t joystickXMin;
    int16_t joystickXMax;
    int16_t joystickYMin;
    int16_t joystickYMax;
    int16_t wheelMin;
    int16_t wheelMax;
};

ConfigEE configMain = {};

//default values???
// int zeroWHEEL = 0;
// int maxWHEEL = 1023;
// int zeroTHR = 0;
// int maxTHR = 1023;
// int zeroBR = 0;
// int maxBR = 1023;
// int zeroCTH = 0;
// int maxCTH = 1023;
// int zeroJOY_X = 0;
// int maxJOY_X = 1023;
// int zeroJOY_Y = 0;
// int maxJOY_Y = 1023;

void getConf() 
{
    // EEPROM.get(0, config.throttleMin);
    // EEPROM.get(2, config.throttleMax);
    // EEPROM.get(4, config.brakeMin);
    // EEPROM.get(6, config.brakeMax);
    // EEPROM.get(8, config.clutchMin);
    // EEPROM.get(10, config.clutchMax);
    // EEPROM.get(12, config.joystickXMin);
    // EEPROM.get(14, config.joystickXMax);
    // EEPROM.get(16, config.joystickYMin);
    // EEPROM.get(18, config.joystickYMax);
    // EEPROM.get(20, config.wheelMin);
    // EEPROM.get(22, config.wheelMax);

    EEPROM.get(EE_BASE_ADDR, configMain);
}
void printConfig(ConfigEE config)
{
    Serial.print(F("Throttle: "));
    Serial.print(config.throttleMin);
    Serial.print(" - ");
    Serial.println(config.throttleMax);

    Serial.print(F("Brake: "));
    Serial.print(config.brakeMin);
    Serial.print(" - ");
    Serial.println(config.brakeMax);

    Serial.print(F("Clutch: "));
    Serial.print(config.clutchMin);
    Serial.print(" - ");
    Serial.println(config.clutchMax);

    Serial.print(F("Joystick X: "));
    Serial.print(config.joystickXMin);
    Serial.print(" - ");
    Serial.println(config.joystickXMax);

    Serial.print(F("Joystick Y: "));
    Serial.print(config.joystickYMin);
    Serial.print(" - ");
    Serial.println(config.joystickYMax);

    Serial.print(F("Wheel: "));
    Serial.print(config.wheelMin);
    Serial.print(" - ");
    Serial.println(config.wheelMax);

    Serial.println(F("Calibration end"));
}

#endif // EEPROM_LOGIC_H