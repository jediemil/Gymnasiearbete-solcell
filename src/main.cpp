#include "Wire.h"
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>
#include "../.pio/libdeps/esp32s3box/ESP32Servo/src/ESP32Servo.h"
#include "../.pio/libdeps/esp32s3box/NeoPixelBus/src/NeoPixelBus.h"
#include "SDLogger.h"

#define I2C_CLOCK_PIN 48
#define I2C_DATA_PIN 47

#define TWO_AXIS_TOP_RIGHT_PHOTORESISTOR 8
#define TWO_AXIS_TOP_LEFT_PHOTORESISTOR 7
#define TWO_AXIS_BOTTOM_RIGHT_PHOTORESISTOR 9
#define TWO_AXIS_BOTTOM_LEFT_PHOTORESISTOR 6

#define ONE_AXIS_TOP_PHOTORESISTOR 1
#define ONE_AXIS_BOTTOM_PHOTORESISTOR 2
#define ONE_AXIS_LEFT_PHOTORESISTOR 4
#define ONE_AXIS_RIGHT_PHOTORESISTOR 5

#define TWO_AXIS_LOWER_SERVO 36
#define TWO_AXIS_TOP_SERVO 35

#define ONE_AXIS_LOWER_SERVO 39
#define ONE_AXIS_TOP_SERVO 37

#define SD_MISO 11
#define SD_MOSI 13
#define SD_CS 10
#define SD_CLOCK 12

#define BUF_SIZE 256

//#define DEFAULT_TIMER_WIDTH 8

SPIClass sdSPI;

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
TwoWire ADC_I2C = TwoWire(0);

Servo multiBottom;
Servo multiTop;

Servo singleBottom;
Servo singleTop;

SDLogger sdLogger(&SD);

NeoPixelBus<NeoRgbFeature, NeoEsp32BitBangWs2812Method> statLED(1, 38);

float measurementBuf[BUF_SIZE][6];
unsigned long timestampsBuf[BUF_SIZE];

int multiTopAngle = 90;
int multiBottomAngle = 90;
int singleTopAngle = 90;
int singleBottomAngle = 90;

void stepServos() {
    uint16_t topLeftVal = analogRead(TWO_AXIS_TOP_LEFT_PHOTORESISTOR);
    uint16_t topRightVal = analogRead(TWO_AXIS_TOP_RIGHT_PHOTORESISTOR);
    uint16_t bottomLeftVal = analogRead(TWO_AXIS_BOTTOM_LEFT_PHOTORESISTOR);
    uint16_t bottomRightVal = analogRead(TWO_AXIS_BOTTOM_RIGHT_PHOTORESISTOR);
    //int multiTopAngle = multiTop.read();
    //int multiBottomAngle = multiBottom.read();

    Serial0.println(multiBottomAngle);
    Serial0.println(topLeftVal);
    Serial0.println(topLeftVal > topRightVal || bottomLeftVal > bottomRightVal);
    Serial0.println(topRightVal > bottomRightVal || topLeftVal > bottomLeftVal);


    uint16_t singleLeftVal = analogRead(ONE_AXIS_LEFT_PHOTORESISTOR);
    uint16_t singleRightVal = analogRead(ONE_AXIS_RIGHT_PHOTORESISTOR);
    uint16_t singleTopVal = analogRead(ONE_AXIS_TOP_PHOTORESISTOR);
    uint16_t singleBottomVal = analogRead(ONE_AXIS_BOTTOM_PHOTORESISTOR);

    //int singleTopAngle = singleTop.read();
    //int singleBottomAngle = singleBottom.read();

    if (topLeftVal > topRightVal || bottomLeftVal > bottomRightVal) {
        if (multiTopAngle > 90) {
            multiBottomAngle = max(0, multiBottomAngle - 1);
        } else {
            multiBottomAngle = min(180, multiBottomAngle + 1);
        }
    } else {
        if (multiTopAngle > 90) {
            multiBottomAngle = min(180, multiBottomAngle + 1);
        } else {
            multiBottomAngle = max(0, multiBottomAngle - 1);
        }

    }

    if (topRightVal > bottomRightVal || topLeftVal > bottomLeftVal) {
        multiTopAngle = max(0, multiTopAngle - 1);
    } else {
        multiTopAngle = min(180, multiTopAngle + 1);
    }


    if (singleLeftVal > singleRightVal) {
        singleBottomAngle = min(180, singleBottomAngle + 1);
    } else {
        singleBottomAngle = max(0, singleBottomAngle - 1);
    }

    if (singleTopVal > singleBottomVal) {
        singleTopAngle = min(180, singleTopAngle + 1);
    } else {
        singleTopAngle = max(0, singleTopAngle - 1);
    }

    multiBottom.write(multiBottomAngle);
    Serial0.println(singleTopVal);
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
    Serial0.begin(115200);
    Serial0.println("Boot");
    delay(100);

    statLED.Begin();
    statLED.ClearTo(RgbColor(255, 255, 0));
    statLED.Show();

    Serial0.println("LED startad");
    delay(100);

    sdSPI.begin(SD_CLOCK, SD_MISO, SD_MOSI, SD_CS);
    sdLogger.setupSD(SD_CS, sdSPI);
    sdLogger.openLogFile("test.txt");

    Serial0.println("SPI och SD startad");
    delay(100);

    ADC_I2C.begin(I2C_DATA_PIN, I2C_CLOCK_PIN);

    Serial0.println("I2C startad");
    delay(100);

    ads1.setGain(GAIN_SIXTEEN);
    ads1.begin(0x48, &ADC_I2C);

    ads2.setGain(GAIN_SIXTEEN);
    ads2.begin(0x49, &ADC_I2C);

    Serial0.println("ADC igångsatt");
    delay(100);

    multiBottom.getPwm()->pwmChannel = 0;
    multiTop.getPwm()->pwmChannel = 1;
    singleTop.getPwm()->pwmChannel = 2;
    singleBottom.getPwm()->pwmChannel = 3;
    /*ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);*/

    Serial0.println("Timer allokerad");
    delay(100);

    multiBottom.attach(TWO_AXIS_LOWER_SERVO, 500, 2500);
    multiTop.attach(TWO_AXIS_TOP_SERVO, 500, 2500);

    singleBottom.attach(ONE_AXIS_LOWER_SERVO, 500, 2500);
    singleTop.attach(ONE_AXIS_TOP_SERVO, 500, 2500);

    Serial0.println("Servo igång");
    delay(100);
    Serial0.println("Startar program");
    delay(100);
    multiTop.write(90);
    multiBottom.write(90);
    singleBottom.write(90);
    singleTop.write(90);
}

void loop() {
    /*for (int pos = 0; pos <= 180; pos += 1) {
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
    //Serial0.println(adc0);
    float volts0 = ads1.computeVolts(adc1_0); //adc0 / (float) 0x8000 * 0.256;
    Serial0.print("Variable_1:");
    Serial0.println(volts0*2000);*/
    stepServos();
    delay((10));
}