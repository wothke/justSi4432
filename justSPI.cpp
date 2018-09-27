#include <util/atomic.h>

#include <justSPI.h>

#define WRITE_MASK 0b10000000
#define READ_MASK ~WRITE_MASK


JustSPI::JustSPI(uint8_t slaveSelectPin) {
	_slaveSelectPin= slaveSelectPin;

	// todo: hard-coded settings could be opened up for configuration
	uint32_t freq= 1000000;		// 1MHz SPI bus
	uint8_t order= MSBFIRST;
	uint8_t mode= 0;			// CPOL = 0, CPHA = 0, CKE/NCPHA = 1
	_spiSettings = SPISettings(freq, order, mode);
}

void JustSPI::begin() {
	pinMode(_slaveSelectPin, OUTPUT);
	digitalWrite(_slaveSelectPin, HIGH);

	SPI.begin();
}

uint8_t JustSPI::read(uint8_t reg) {
	uint8_t retVal;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		digitalWrite(_slaveSelectPin, LOW);
		SPI.transfer(reg & READ_MASK);
		retVal = SPI.transfer(0);
		digitalWrite(_slaveSelectPin, HIGH);
	}
	return retVal;
}

uint8_t JustSPI::readBlock(uint8_t reg, uint8_t* dest, uint8_t len) {
	uint8_t retVal = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		SPI.beginTransaction(_spiSettings);
		digitalWrite(_slaveSelectPin, LOW);
		retVal = SPI.transfer(reg & READ_MASK);
		while (len--) {
			*dest++ = SPI.transfer(0);
		}
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();
	}
	return retVal;
}

uint8_t JustSPI::write(uint8_t reg, uint8_t val) {
	uint8_t retVal = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		SPI.beginTransaction(_spiSettings);
		digitalWrite(_slaveSelectPin, LOW);
		retVal = SPI.transfer(reg | WRITE_MASK);
		SPI.transfer(val);
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();
	}
	return retVal;
}

uint8_t JustSPI::writeBlock(uint8_t reg, const uint8_t* src, uint8_t len) {
	uint8_t retVal = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		SPI.beginTransaction(_spiSettings);
		digitalWrite(_slaveSelectPin, LOW);
		retVal = SPI.transfer(reg | WRITE_MASK);
		while (len--) {
			SPI.transfer(*src++);
		}
		digitalWrite(_slaveSelectPin, HIGH);
		SPI.endTransaction();
	}
	return retVal;
}
