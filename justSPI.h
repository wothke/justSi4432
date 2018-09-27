#ifndef JUSTSPI_H_
#define JUSTSPI_H_

#include <pins_arduino.h>
#include <SPI.h>

/**
 * Utility for use of hardware SPI (see https://en.wikipedia.org/wiki/Serial_Peripheral_Interface;
 * https://arduino.stackexchange.com/questions/16348/how-do-you-use-spi-on-an-arduino).
 *
 * Limitations: Specifically done for ProMini328 and ATMega128 (nothing else has been
 * tested). Used SPISettings (bit order, etc) are hard-coded (and likely would need to be adapted for use
 * on different platforms).
 */
class JustSPI {
public:
    JustSPI(uint8_t slaveSelectPin = SS);

    /**
     * Use this deferred initialization from within sketch's setup().
     */
    void begin();

    uint8_t read(uint8_t reg);
    uint8_t readBlock(uint8_t reg, uint8_t* dest, uint8_t len);

    uint8_t write(uint8_t reg, uint8_t val);
    uint8_t writeBlock(uint8_t reg, const uint8_t* src, uint8_t len);
protected:
    uint8_t _slaveSelectPin;
    SPISettings _spiSettings;
};

#endif
