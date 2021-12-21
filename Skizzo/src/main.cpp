#include <Arduino.h>

#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>

// Définition des constantes globales
// Numéro de port auquel est branchée la LED servant de flash.
#define PORT_LED_FLASH 4   

MotorDriver motor;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(14, 15);
    Serial.begin(115200);
    motor.init();
    pinMode(PORT_LED_FLASH, OUTPUT);
}

void loop()
{
    Serial.println("Test");
    digitalWrite(PORT_LED_FLASH, HIGH);
    delay(1000);
    digitalWrite(PORT_LED_FLASH, LOW);
    delay(1000);

    // drive 2 dc motors at speed=255, clockwise
    Serial.println("run at speed=255");
    motor.dcMotorRun(MOTOR_CHA, 255);
    motor.dcMotorRun(MOTOR_CHB, 255);
    delay(1000);
    
    // brake
    Serial.println("brake");
    motor.dcMotorBrake(MOTOR_CHA);
    motor.dcMotorBrake(MOTOR_CHB);
    delay(1000);

    // drive 2 dc motors at speed=200, anticlockwise
    Serial.println("run at speed=-200");
    motor.dcMotorRun(MOTOR_CHA, -200);
    motor.dcMotorRun(MOTOR_CHB, -200);
    delay(1000);

    // stop 2 motors
    Serial.println("stop");
    motor.dcMotorStop(MOTOR_CHA);
    motor.dcMotorStop(MOTOR_CHB);
    delay(1000);
}