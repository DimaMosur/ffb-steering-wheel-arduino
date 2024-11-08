#include "Joystick.h"
#include <EEPROM.h>

// X-axis & Y-axis REQUIRED
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK, 4, 0,
true, true, true, //X,Y,Z
true, true, true,//Rx,Ry,Rz
false, false, false, false, false);


Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};

#define DEBUG 0             // режим отладки
#define POT_THR A0          // педаль газа
#define POT_BR A1           // педаль тормоза
#define POT_CTH A2          // педаль сцепления
#define POT_JOY_X A3        // джойстик Y
#define POT_JOY_Y A4        // джойстик Y
#define POT_WHEEL A5
#define BUTT_BR 4           // кнопка ручника
#define BUTT_CAL 3          // кнопка калибровки

int throttleMin, throttleMax, brakeMin, brakeMax, clutchMin, clutchMax, joystickXMin, joystickXMax, joystickYMin, joystickYMax, wheelMin, wheelMax;

void setup() {
    Serial.begin(115200);
    pinMode(BUTT_BR, INPUT_PULLUP);
    pinMode(BUTT_CAL, INPUT_PULLUP);

    debug();
    calibration();

    pinMode(8, OUTPUT); 
    pinMode(9, OUTPUT);  // l_pwm
    pinMode(10, OUTPUT); // r_pwn

    EEPROM.get(0, throttleMin);
    EEPROM.get(2, brakeMin);
    EEPROM.get(4, clutchMin);
    EEPROM.get(6, throttleMax);
    EEPROM.get(8, brakeMax);
    EEPROM.get(10, clutchMax);
    EEPROM.get(14, joystickXMin);
    EEPROM.get(16, joystickXMax);
    EEPROM.get(18, joystickYMin);
    EEPROM.get(20, joystickYMax);
    EEPROM.get(12, wheelMin);
    EEPROM.get(22, wheelMax);
    Joystick.setGains(mygains);
    Joystick.begin();
}

void loop() {

    int wheel, thr, br, cth, joyx, joyy;
    wheel = map(analogRead(POT_WHEEL), wheelMin, wheelMax, 0, 1023);
    wheel = constrain(wheel, 0, 1023);
    myeffectparams[0].springMaxPosition = 1023;
    myeffectparams[0].springPosition = wheel;    // 0-1023
    myeffectparams[0].damperMaxVelocity = 1023;
    myeffectparams[0].damperVelocity = wheel;
    myeffectparams[0].inertiaMaxAcceleration = 1023;
    myeffectparams[0].inertiaAcceleration = wheel;
    myeffectparams[0].frictionMaxPositionChange = 1023;
    myeffectparams[0].frictionPositionChange = wheel;
    Joystick.setXAxis(wheel);

    thr = map(analogRead(POT_THR), throttleMin, throttleMax, 0, 1023);
    thr = constrain(thr, 0, 1023);
    Joystick.setYAxis(thr);

    br = map(analogRead(POT_BR), brakeMin, brakeMax, 0, 1023);
    br = constrain(br, 0, 1023);
    Joystick.setZAxis(br);

    cth = map(analogRead(POT_CTH), clutchMin, clutchMax, 0, 1023);
    cth = constrain(cth, 0, 1023);
    Joystick.setRxAxis(cth);

    joyx = map(analogRead(POT_JOY_X), joystickXMin, joystickXMax, 0, 1023);
    joyx = constrain(joyx, 0, 1023);
    Joystick.setRyAxis(joyx);

    joyy = map(analogRead(POT_JOY_Y), joystickYMin, joystickYMax, 0, 1023);
    joyy = constrain(joyy, 0, 1023);
    Joystick.setRzAxis(joyy);

    Joystick.setEffectParams(myeffectparams);
    Joystick.getForce(forces);
    Joystick.sendState();

    if (forces[0] > 0) {
        digitalWrite(9, LOW);
        digitalWrite(10, HIGH);
        analogWrite(8, abs(forces[0]));
    } else {
        digitalWrite(9, HIGH);
        digitalWrite(10, LOW);
        analogWrite(8, abs(forces[0]));
    }
    delay(1);
}

void calibration() {
    if (!digitalRead(BUTT_CAL)) {      // нажата кнопка
        while (!digitalRead(BUTT_CAL)); // пока кнопка удерживается
        Serial.begin(9600);
        delay(100);
        Serial.print(F("Calibration start"));
        int zeroTHR = analogRead(POT_THR);
        int zeroBR = analogRead(POT_BR);
        int zeroCTH = analogRead(POT_CTH);
        int zeroWHEEL = analogRead(POT_WHEEL);
        int zeroJOY_X = analogRead(POT_JOY_X);
        int zeroJOY_Y = analogRead(POT_JOY_Y);
        int maxTHR, maxBR, maxCTH, maxWHEEL, maxJOY_X, maxJOY_Y;

        EEPROM.put(0, zeroTHR);
        EEPROM.put(2, zeroBR);
        EEPROM.put(4, zeroCTH);
        EEPROM.put(12, zeroWHEEL);
        EEPROM.put(14, zeroJOY_X);
        EEPROM.put(18, zeroJOY_Y);
        delay(100);                     // дебаунс
        while (true) {                  // крутимся
            if (!digitalRead(BUTT_CAL)) break;
            maxTHR = analogRead(POT_THR);
            maxBR = analogRead(POT_BR);
            maxCTH = analogRead(POT_CTH);
            maxWHEEL = analogRead(POT_WHEEL);
            maxJOY_X = analogRead(POT_JOY_X);
            maxJOY_Y = analogRead(POT_JOY_Y);
        }
        EEPROM.put(6, maxTHR);
        EEPROM.put(8, maxBR);
        EEPROM.put(10, maxCTH);
        EEPROM.put(16, maxJOY_X);
        EEPROM.put(20, maxJOY_Y);
        EEPROM.put(22, maxWHEEL);

        Serial.println(F("Calibration end"));
        Serial.print(F("Wheek: "));
        Serial.print(zeroWHEEL);
        Serial.print(" - ");
        Serial.println(maxWHEEL);
        Serial.print(F("Throat: "));
        Serial.print(zeroTHR);
        Serial.print(" - ");
        Serial.println(maxTHR);
        Serial.print(F("Brake: "));
        Serial.print(zeroBR);
        Serial.print(" - ");
        Serial.println(maxBR);
        Serial.print(F("Clutch: "));
        Serial.print(zeroCTH);
        Serial.print(" - ");
        Serial.println(maxCTH);
        Serial.print(F("JoystickX: "));
        Serial.print(zeroJOY_X);
        Serial.print(" - ");
        Serial.println(maxJOY_X);
        Serial.print(F("JoystickY: "));
        Serial.print(zeroJOY_Y);
        Serial.print(" - ");
        Serial.println(maxJOY_Y);
        Serial.println();
    }
    Serial.end();
    delay(3000); // задержка чтобы кнопку отпустить
}

void debug() {
    #if (DEBUG == 1)
        Serial.begin(9600);
        uint32_t timer;
        while (true) {
            if (millis() - timer > 100) {
                timer = millis();
                Serial.print(analogRead(POT_WHEEL));
                Serial.print("\t");
                Serial.print(analogRead(POT_THR));
                Serial.print("\t");
                Serial.print(analogRead(POT_BR));
                Serial.print("\t");
                Serial.print(analogRead(POT_CTH));
                Serial.print("\t");
                Serial.print(analogRead(POT_JOY_X));
                Serial.print("\t");
                Serial.print(analogRead(POT_JOY_Y));
                Serial.print("\t");
                Serial.print(!digitalRead(BUTT_BR));
                Serial.print("\t");
                Serial.println(!digitalRead(BUTT_CAL));
            }
        }
        Serial.end();
    #endif
}
