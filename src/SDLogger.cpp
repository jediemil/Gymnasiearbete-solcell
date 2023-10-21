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

void SDLogger::setupSD(uint8_t cs_pin) { // Från ett exempel i bibloteket (https://github.com/espressif/arduino-esp32/blob/master/libraries/SD/examples/SD_Test/SD_Test.ino)
    if(!sd->begin(cs_pin)){
        Serial0.println("Kortet kunde inte monteras");
        return;
    }
    uint8_t cardType = sd->cardType();

    if(cardType == CARD_NONE){
        Serial0.println("Hittade inget SD-kort");
        return;
    }

    Serial0.print("SD-korttyp: ");
    if(cardType == CARD_MMC){
        Serial0.println("MMC");
    } else if(cardType == CARD_SD){
        Serial0.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial0.println("SDHC");
    } else {
        Serial0.println("Okänd");
    }
}

bool SDLogger::openLogFile() {
    if (!fileOpen) {
        int numFiles = 0;
        //SD.mkdir("/tests");
        File testRoot = SD.open("/tests");
        while (true) {
            File entry = testRoot.openNextFile();
            if (!entry) {
                // no more files
                break;
            }
            numFiles++;
            entry.close();
        }
        testRoot.close();

        Serial0.print("Filer på SD-kort: ");
        Serial0.println(numFiles);

        delay(100);
        String numFilesStr = String(numFiles + 1);
        logFile = SD.open("/tests/test" + numFilesStr + ".txt", FILE_WRITE);
        if (!logFile) {
            Serial0.println("Misslyckades med att öppna fil");
            return false;
        }

        fileOpen = true;
        return true;
    }
    return false;
}

bool SDLogger::writeBufToFile(int16_t buf[BUF_SIZE][6], unsigned long timeBuf[BUF_SIZE], uint32_t len) {
    bool success = true;
    //noInterrupts();
    for (int i = 0; i < BUF_SIZE; i++) {
        //Serial0.println(timeBuf[i]);
        //Serial0.println(uxTaskGetStackHighWaterMark(NULL));
        success &= logFile.print(timeBuf[i]) > 0;
        //Serial0.println(success);
        for (int j = 0; j < 6; j++) {
            success &= logFile.print(" ") > 0;
            success &= logFile.print(buf[i][j]) > 0;
        }
        success &= logFile.println("") > 0;
    }

    logFile.flush();
    Serial0.println("Flushad");
    //interrupts();

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