#include "Wire.h"
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>
#include "../.pio/libdeps/esp32s3box/ESP32Servo/src/ESP32Servo.h"
#include "../.pio/libdeps/esp32s3box/NeoPixelBus/src/NeoPixelBus.h"
#include "SDLogger.h"

#define I2C_CLOCK_PIN 0
#define I2C_DATA_PIN 0

#define TWO_AXIS_TOP_RIGHT_PHOTORESISTOR 1
#define TWO_AXIS_TOP_LEFT_PHOTORESISTOR 1
#define TWO_AXIS_BOTTOM_RIGHT_PHOTORESISTOR 1
#define TWO_AXIS_BOTTOM_LEFT_PHOTORESISTOR 1

#define ONE_AXIS_TOP_PHOTORESISTOR 1
#define ONE_AXIS_BOTTOM_PHOTORESISTOR 1
#define ONE_AXIS_LEFT_PHOTORESISTOR 1
#define ONE_AXIS_RIGHT_PHOTORESISTOR 1

#define TWO_AXIS_LOWER_SERVO 1
#define TWO_AXIS_TOP_SERVO 1

#define ONE_AXIS_LOWER_SERVO 1
#define ONE_AXIS_TOP_SERVO 1

#define SD_MISO 1
#define SD_MOSI 1
#define SD_CS 1
#define SD_CLOCK 1

#define BUF_SIZE 256

SPIClass sdSPI;

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
TwoWire ADC_I2C = TwoWire(0);

Servo multiBottom;
Servo multiTop;

Servo singleBottom;
Servo singleTop;

SDLogger sdLogger(&SD);

NeoPixelBus<NeoBgrFeature, NeoEsp32Rmt0800KbpsMethod> statLED(1, 38);

float measurementBuf[BUF_SIZE][6];
unsigned long timestampsBuf[BUF_SIZE];

void stepServos() {
    uint16_t topLeftVal = analogRead(TWO_AXIS_TOP_LEFT_PHOTORESISTOR);
    uint16_t topRightVal = analogRead(TWO_AXIS_TOP_RIGHT_PHOTORESISTOR);
    uint16_t bottomLeftVal = analogRead(TWO_AXIS_BOTTOM_LEFT_PHOTORESISTOR);
    uint16_t bottomRightVal = analogRead(TWO_AXIS_BOTTOM_RIGHT_PHOTORESISTOR);
    int multiTopAngle = multiTop.read();
    int multiBottomAngle = multiBottom.read();


    uint16_t singleLeftVal = analogRead(ONE_AXIS_LEFT_PHOTORESISTOR);
    uint16_t singleRightVal = analogRead(ONE_AXIS_RIGHT_PHOTORESISTOR);
    uint16_t singleTopVal = analogRead(ONE_AXIS_TOP_PHOTORESISTOR);
    uint16_t singleBottomVal = analogRead(ONE_AXIS_BOTTOM_PHOTORESISTOR);

    int singleTopAngle = singleTop.read();
    int singleBottomAngle = singleBottom.read();

    if (topLeftVal > topRightVal || bottomLeftVal > bottomRightVal) {
        multiBottomAngle++;
    } else {
        multiBottomAngle--;
    }

    if (topRightVal > bottomRightVal || topLeftVal > bottomLeftVal) {
        multiTopAngle++;
    } else {
        multiTopAngle--;
    }


    if (singleLeftVal > singleRightVal) {
        singleBottomAngle++;
    } else {
        singleBottomAngle--;
    }

    if (singleTopVal > singleBottomVal) {
        singleTopAngle++;
    } else {
        singleTopAngle--;
    }

    multiBottom.write(multiBottomAngle);
    multiTop.write(multiTopAngle);
    singleBottom.write(singleBottomAngle);
    singleTop.write(singleTopAngle);
}

void measurementLoop() {
    for (int i = 0; i < BUF_SIZE; i++) {
        measurementBuf[i][0] = ads1.readADC_SingleEnded(0);
        measurementBuf[i][1] = ads1.readADC_SingleEnded(1);
        measurementBuf[i][2] = ads1.readADC_SingleEnded(2);
        measurementBuf[i][3] = ads1.readADC_SingleEnded(3);

        measurementBuf[i][4] = ads2.readADC_SingleEnded(0);
        measurementBuf[i][5] = ads2.readADC_SingleEnded(1);

        timestampsBuf[i] = millis();
        delay(1000);
    }
    if (!sdLogger.writeBufToFile(measurementBuf, timestampsBuf, BUF_SIZE)) {
        statLED.ClearTo(RgbColor(255, 0, 0));
        statLED.Show();
    }
}

void setup() {
    Serial.begin(115200);

    statLED.Begin();
    statLED.ClearTo(RgbColor(255, 255, 0));
    statLED.Show();

    sdSPI.begin(SD_CLOCK, SD_MISO, SD_MOSI, SD_CS);
    sdLogger.setupSD(SD_CS, sdSPI);
    sdLogger.openLogFile("test.txt");

    ADC_I2C.begin(I2C_DATA_PIN, I2C_CLOCK_PIN);

    ads1.setGain(GAIN_SIXTEEN);
    ads1.begin(0x48, &ADC_I2C);

    ads2.setGain(GAIN_SIXTEEN);
    ads2.begin(0x49, &ADC_I2C);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    multiBottom.attach(TWO_AXIS_LOWER_SERVO, 500, 2500);
    multiTop.attach(TWO_AXIS_TOP_SERVO, 500, 2500);

    singleBottom.attach(ONE_AXIS_LOWER_SERVO, 500, 2500);
    singleTop.attach(ONE_AXIS_TOP_SERVO, 500, 2500);
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
    int16_t adc1_0;

    adc1_0 = ads1.readADC_SingleEnded(0);
    //Serial.println(adc0);
    float volts0 = ads1.computeVolts(adc1_0); //adc0 / (float) 0x8000 * 0.256;
    /*Serial.print("AIN0: ");
    Serial.println(adc0);
    Serial.print("VOLT0: ");
    Serial.println(volts0);

    Serial.println(" ");*/
    Serial.print("Variable_1:");
    Serial.println(volts0*2000);
}