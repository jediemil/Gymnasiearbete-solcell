//
// Created by emil on 2023-10-13.
//


#ifndef GYMNASIEARBETE_SOLCELL_SDLOGGER_H
#define GYMNASIEARBETE_SOLCELL_SDLOGGER_H
#define BUF_SIZE 256

#include "FS.h"
#include "SD.h"
#include "SPI.h"


class SDLogger {
public:
    SDLogger(SDFS* sd);

    void setupSD(uint8_t cs_pin);
    bool openLogFile();
    bool writeBufToFile(int16_t buf[BUF_SIZE][6], unsigned long timeBuf[BUF_SIZE], uint32_t len);
    bool closeLogFile();

private:
    SDFS* sd;
    File logFile;
    bool fileOpen;
};


#endif //GYMNASIEARBETE_SOLCELL_SDLOGGER_H
