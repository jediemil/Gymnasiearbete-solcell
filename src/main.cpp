#include "SPI.h"
#include "Wire.h"
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>
#include "../.pio/libdeps/esp32s3box/ESP32Servo/src/ESP32Servo.h"

Adafruit_ADS1115 ads;
TwoWire I2C = TwoWire(0);

Servo multiBottom;
Servo multiTop;

void setup() {
    I2C.begin(8, 9);
    Serial.begin(115200);
    ads.setGain(GAIN_SIXTEEN);
    ads.begin(0x48, &I2C);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    multiBottom.setPeriodHertz(50);
    multiBottom.attach(41, 500, 2500);
    multiTop.attach(42, 500, 2500);
}

void loop() {
    for (int pos = 0; pos <= 180; pos += 1) {
        multiBottom.write(pos);
        multiTop.write(180 - pos);
        delay(15);
    }
    for (int pos = 180; pos >= 0; pos -= 1) {
        multiBottom.write(pos);
        multiTop.write(180 - pos);
        delay(15);
    }

    delay(50);

    multiBottom.write(90);
    multiTop.write(90);
    delay(1000);
    int16_t adc0;

    adc0 = ads.readADC_SingleEnded(0);
    //Serial.println(adc0);
    float volts0 = ads.computeVolts(adc0); //adc0 / (float) 0x8000 * 0.256;
    /*Serial.print("AIN0: ");
    Serial.println(adc0);
    Serial.print("VOLT0: ");
    Serial.println(volts0);

    Serial.println(" ");*/
    Serial.print("Variable_1:");
    Serial.println(volts0*2000);
}