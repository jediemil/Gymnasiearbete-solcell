#include "SPI.h"
#include "Wire.h"
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;
TwoWire I2C = TwoWire(0);

void setup() {
    I2C.begin(8, 9);
    Serial.begin(115200);
    ads.setGain(GAIN_SIXTEEN);
    ads.begin(0x48, &I2C);
}

void loop() {
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