//
// Created by emil on 2023-10-13.
//


#ifndef GYMNASIEARBETE_SOLCELL_SDLOGGER_H
#define GYMNASIEARBETE_SOLCELL_SDLOGGER_H

#include "FS.h"
#include "SD.h"
#include "SPI.h"


class SDLogger {
public:
    SDLogger(SDFS* sd);

    void setupSD(uint8_t cs_pin, SPIClass spiInterface);
    bool openLogFile(String name);
    bool writeBufToFile(float buf[][6], unsigned long* timeBuf, uint32_t len);
    bool closeLogFile();

private:
    SDFS* sd;
    File logFile;
    bool fileOpen;
};


#endif //GYMNASIEARBETE_SOLCELL_SDLOGGER_H
