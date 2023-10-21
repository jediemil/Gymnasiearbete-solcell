#include "Wire.h"
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>
#include "../.pio/libdeps/esp32s3box/ServoESP32/src/Servo.h"
#include "../.pio/libdeps/esp32s3box/NeoPixelBus/src/NeoPixelBus.h"
#include "SDLogger.h"
#include <FreeRTOS.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

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

#define TWO_AXIS_LOWER_SERVO 16
#define TWO_AXIS_TOP_SERVO 17

#define ONE_AXIS_LOWER_SERVO 39
#define ONE_AXIS_TOP_SERVO 15

#define SD_MISO 11
#define SD_MOSI 13
#define SD_CS 10
#define SD_CLOCK 12

#define ARGB_LED_PIN 38
#define STOP_BUTTON_PIN 0

#define SERVO_DELAY_MS 100
#define LOG_RATE_MS 1000
#define TRACKER_DEAD_SPACE 400

//#define DEFAULT_TIMER_WIDTH 8

//SPIClass sdSPI;

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
TwoWire ADC_I2C = TwoWire(0);

Servo multiBottom;
Servo multiTop;

Servo singleBottom;
Servo singleTop;

SDLogger sdLogger(&SD);

NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> statLED(1, ARGB_LED_PIN);

TaskHandle_t logTaskHandle;
TaskHandle_t servoTaskHandle;

QueueHandle_t queue;
int queueSize = 2;

int16_t measurementBuf[BUF_SIZE][6];
unsigned long timestampsBuf[BUF_SIZE];

int multiTopAngle = 90;
int multiBottomAngle = 90;
int singleTopAngle = 90;
int singleBottomAngle = 90;

void stepServos(int trackerDeadSpace) {
    uint16_t topLeftVal = analogRead(TWO_AXIS_TOP_LEFT_PHOTORESISTOR);
    uint16_t topRightVal = analogRead(TWO_AXIS_TOP_RIGHT_PHOTORESISTOR);
    uint16_t bottomLeftVal = analogRead(TWO_AXIS_BOTTOM_LEFT_PHOTORESISTOR);
    uint16_t bottomRightVal = analogRead(TWO_AXIS_BOTTOM_RIGHT_PHOTORESISTOR);
    //int multiTopAngle = multiTop.read();
    //int multiBottomAngle = multiBottom.read();

    /*Serial0.println(multiBottomAngle);
    Serial0.println(topLeftVal);
    Serial0.println(topLeftVal > topRightVal || bottomLeftVal > bottomRightVal);
    Serial0.println(topRightVal > bottomRightVal || topLeftVal > bottomLeftVal);*/


    uint16_t singleLeftVal = analogRead(ONE_AXIS_LEFT_PHOTORESISTOR);
    uint16_t singleRightVal = analogRead(ONE_AXIS_RIGHT_PHOTORESISTOR);
    uint16_t singleTopVal = analogRead(ONE_AXIS_TOP_PHOTORESISTOR);
    uint16_t singleBottomVal = analogRead(ONE_AXIS_BOTTOM_PHOTORESISTOR);

    //int singleTopAngle = singleTop.read();
    //int singleBottomAngle = singleBottom.read();
    int tLvtR = topLeftVal - topRightVal;
    int blvtR = bottomLeftVal - bottomRightVal;
    //Serial0.println(tLvtR);
    //Serial0.println(blvtR);

    if (tLvtR > trackerDeadSpace || blvtR > trackerDeadSpace) {
        if (multiTopAngle > 90) {
            multiBottomAngle = max(0, multiBottomAngle - 1);
        } else {
            multiBottomAngle = min(180, multiBottomAngle + 1);
        }
    } else if (tLvtR < -trackerDeadSpace || blvtR < -trackerDeadSpace) {
        if (multiTopAngle > 90) {
            multiBottomAngle = min(180, multiBottomAngle + 1);
        } else {
            multiBottomAngle = max(0, multiBottomAngle - 1);
        }

    }

    int tRvbR = topRightVal - bottomRightVal;
    int tLvbL = topLeftVal - bottomLeftVal;

    if (tRvbR > trackerDeadSpace || tLvbL > trackerDeadSpace) {
        multiTopAngle = max(20, multiTopAngle - 1);
    } else if (tRvbR < -trackerDeadSpace || tLvbL < -trackerDeadSpace) {
        multiTopAngle = min(160, multiTopAngle + 1);
    }

    int sLvsR = singleLeftVal - singleRightVal;
    int sTvsB = singleTopVal - singleBottomVal;

    if (sLvsR > trackerDeadSpace) {
        singleBottomAngle = max(0, singleBottomAngle - 1);
    } else if (sLvsR < -trackerDeadSpace) {
        singleBottomAngle = min(180, singleBottomAngle + 1);
    }

    if (sTvsB > trackerDeadSpace) {
        singleTopAngle = min(160, singleTopAngle + 1);
    } else if (sTvsB < -trackerDeadSpace){
        singleTopAngle = max(20, singleTopAngle - 1);
    }

    multiBottom.write(multiBottomAngle);
    //Serial0.println(singleTopVal);
    multiTop.write(multiTopAngle);
    singleBottom.write(singleBottomAngle);
    singleTop.write(singleTopAngle);
    /*for (int i = 0; i < 180; i++) {
        multiBottom.write(i);
        delay(100);
    }
    for (int i = 0; i < 180; i++) {
        multiTop.write(i);
        delay(100);
    }
    for (int i = 0; i < 180; i++) {
        singleBottom.write(i);
        delay(100);
    }
    for (int i = 0; i < 180; i++) {
        singleTop.write(i);
        delay(100);
    }*/
}

bool measurementLoop() {
    for (int i = 0; i < BUF_SIZE; i++) {
        measurementBuf[i][0] = ads1.readADC_SingleEnded(0);
        measurementBuf[i][1] = ads1.readADC_SingleEnded(1);
        measurementBuf[i][2] = ads1.readADC_SingleEnded(2);
        measurementBuf[i][3] = ads1.readADC_SingleEnded(3);

        measurementBuf[i][4] = ads2.readADC_SingleEnded(0);
        measurementBuf[i][5] = ads2.readADC_SingleEnded(1);

        timestampsBuf[i] = millis();
        delay(LOG_RATE_MS);
    }
    bool success = sdLogger.writeBufToFile(measurementBuf, timestampsBuf, BUF_SIZE);
    return success;
}

void logTask(void * args) {
    Serial0.println("Mättask startad");
    statLED.ClearTo(RgbColor(0, 0, 255));
    statLED.Show();
    while (true) {
        Serial0.println("B");
        bool success = measurementLoop();
        if (!success) {
            break;
        }
        int e;
        Serial0.println("A");
        delay(100);
        xQueueReceive(queue, &e, 10);
        Serial0.println(e);
        if (e == 1) {
            Serial0.println("Avslutar log");
            sdLogger.closeLogFile();
            statLED.ClearTo(RgbColor(0, 255, 0));
            statLED.Show();
            delay(100);
            vTaskDelete(NULL);
        }
    }

    statLED.ClearTo(RgbColor(255, 0, 0));
    statLED.Show();
    Serial0.println("Kunde inte skriva till SD-kort. Log avbruten.");
    sdLogger.closeLogFile();
    delay(100);
    vTaskDelete(NULL);
}

void servoTask(void * args) {
    Serial0.println("Servotask startad");
    //int i = 0;
    while (true) {
        //stepServos(min(i * TRACKER_DEAD_SPACE, TRACKER_DEAD_SPACE));
        stepServos(0);
        delay(SERVO_DELAY_MS);
        //i++;
        //i %= 10;
    }
}

void stopIrq() {
    Serial0.println("Stänger av loggning");
    Serial0.println("Skickar meddelande");
    int i = 1;
    xQueueSend(queue, &i, portMAX_DELAY);
    vTaskDelete(servoTaskHandle);
}

void setup() {
    Serial0.begin(115200);
    Serial0.println("Boot");
    delay(100);

    statLED.Begin();
    statLED.ClearTo(RgbColor(255, 70, 0));
    statLED.Show();

    Serial0.println("LED startad");
    delay(100);

    pinMode(STOP_BUTTON_PIN, INPUT);
    attachInterrupt(STOP_BUTTON_PIN, stopIrq, FALLING);

    Serial0.println("Interrupt registrerad");
    delay(100);
    SPI.begin(SD_CLOCK, SD_MISO, SD_MOSI, SD_CS);
    //pinMode(SD_CS, OUTPUT);
    //digitalWrite(SD_CS, HIGH);
    sdLogger.setupSD(SD_CS);
    if (!sdLogger.openLogFile()) {
        statLED.ClearTo(RgbColor(255, 0, 0));
        statLED.Show();
        for (;;) {}
    }

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

    /*multiBottom.getPwm()->pwmChannel = 0;
    multiTop.getPwm()->pwmChannel = 1;
    singleTop.getPwm()->pwmChannel = 2;
    singleBottom.getPwm()->pwmChannel = 3;*/
    /*ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);*/

    Serial0.println("Timer allokerad");
    delay(100);

    /*ledcSetup(0, 50, 14); // channel X, 50 Hz, 16-bit depth
    ledcAttachPin(ONE_AXIS_TOP_SERVO, 0);
    ledcWrite(0, 2<<9);*/
    multiBottom.attach(TWO_AXIS_LOWER_SERVO, 0);
    multiTop.attach(TWO_AXIS_TOP_SERVO, 1);

    singleBottom.attach(ONE_AXIS_LOWER_SERVO, 2);
    singleTop.attach(ONE_AXIS_TOP_SERVO, 3);

    Serial0.println("Servo igång");
    delay(100);
    Serial0.println("Startar program");
    delay(100);
    multiTop.write(90);
    multiBottom.write(90);
    singleBottom.write(90);
    singleTop.write(90);

    queue = xQueueCreate( queueSize, sizeof( int ) );

    if(queue == NULL){
        Serial.println("Error creating the queue");
    }

    xTaskCreatePinnedToCore(servoTask, "ServoTask", 10000, NULL, 10, &servoTaskHandle, 0);

    xTaskCreatePinnedToCore(logTask, "LogTask", 100000, NULL, 10, &logTaskHandle, 1);

    Serial0.println("Startad");
}

void loop() {
    //sdLogger.writeBufToFile(measurementBuf, timestampsBuf, BUF_SIZE);
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
    delay(1000);
}