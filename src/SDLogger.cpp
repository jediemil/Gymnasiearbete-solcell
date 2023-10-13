//
// Created by emil on 2023-10-13.
//

#include "SDLogger.h"

//
// Created by emil on 2023-10-13.
//
#include "FS.h"
#include "SD.h"
#include "SPI.h"

SDLogger::SDLogger(fs::SDFS* sd) {
    fileOpen = false;
    this->sd = sd;
}

void SDLogger::setupSD(uint8_t cs_pin, SPIClass spiInterface) { // Från ett exempel i bibloteket (https://github.com/espressif/arduino-esp32/blob/master/libraries/SD/examples/SD_Test/SD_Test.ino)
    if(!sd->begin(cs_pin, spiInterface)){
        Serial.println("Kortet kunde inte monteras");
        return;
    }
    uint8_t cardType = sd->cardType();

    if(cardType == CARD_NONE){
        Serial.println("Hittade inget SD-kort");
        return;
    }

    Serial.print("SD-korttyp: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("Okänd");
    }
}

bool SDLogger::openLogFile(String name) {
    if (!fileOpen) {
        logFile = sd->open(name, FILE_WRITE);
        if (!logFile) {
            Serial.println("Failed to open file for writing");
            return false;
        }

        fileOpen = true;
        return true;
    }
    return false;
}

bool SDLogger::writeBufToFile(float buf[][6], unsigned long* timeBuf, uint32_t len) {
    unsigned int success = 1;
    for (int i; i < len; i++) {
        success *= logFile.print(timeBuf[i]);
        for (int j; j < 6; j++) {
            success *= logFile.print(buf[i][j]);
            success *= logFile.print(" ");
        }
        success *= logFile.println(" ");
    }

    logFile.flush();

    if (success) {
        return true;
    }
    return false;
}

bool SDLogger::closeLogFile() {
    if (fileOpen) {
        logFile.close();
        fileOpen = false;
        return true;
    }
    return false;
}