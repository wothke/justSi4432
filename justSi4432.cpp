#include <Arduino.h>
#include <util/atomic.h>

#include <justSi4432.h>
#include <registersSi4432.h>




// This sample configuration was generated using the Si443x-Register-Settings_RevB1.xls (FSK, No Manchester, CF=470Mhz, Rb=2.4kbs, AFC, Fd=36kHz). For
// people familiar with the RadioHead library it might be useful as a reference. Generate a respective config to match your needs!
// Note: registers marked ---- are ignored, the only reason for keeping them here is to document which pre-calculated
// settings are not actually used.
#ifdef USE_SAMPLE_CONFIG
static uint8_t SAMPLE_CONFIG[] =
//     ++++  ++++    <-- clock recovery settings         ->          <-- OOK     ->  ----  ----  ----                                            ++++  ++++  ++++
// 1c,   1d,   1e,   1f,   20,   21,   22,   23,   24,   25,   2a,   2c,   2d,   2e,   30,   32,   33,   58,   69,   6e,   6f,   70,   71,   72,   75,   76,   77
{0x1b, 0x44, 0x0a, 0x03, 0x41, 0x60, 0x27, 0x52, 0x00, 0x07, 0x21, 0x2a, 0x08, 0x2a, 0xad, 0x8c, 0x02, 0x80, 0x60, 0x13, 0xa9, 0x2c, 0x22, 0x3a, 0x57, 0x00, 0x00};
#endif


static const float MAX_FREQ= 960.0;	// according to spreadsheet - whereas some docs say 930 (see https://www.silabs.com/documents/public/data-sheets/Si4430-31-32.pdf)

// --- FIFO
#define FIFO_SIZE 64                           // size of built-in RX/TX FIFOs

#define TXFAETHR 4                             // TX FIFO Almost Empty Threshold (default)..
#define TX_ALMOST_EMPTY_THRESHOLD (TXFAETHR+1) // .. means:  generate interrupt when 5 bytes remain

#define RXAFTHR 55                             // RX FIFO Almost Full Threshold (default)..
#define RX_ALMOST_FULL_THRESHOLD (RXAFTHR+1)   // .. means: generate interrupt when 56 bytes were received

#define TX_AVAILABLE_SIZE   (FIFO_SIZE - TX_ALMOST_EMPTY_THRESHOLD)


Si4432_Transceiver* Si4432_Transceiver::_singleton = 0;				// only one instance can be used per sketch!

#define READY_TO_SEND_TIMEOUT 2000


Si4432_Transceiver::Si4432_Transceiver(uint8_t slaveSelectPin, uint8_t interruptPin)
{
	_spi= new JustSPI(slaveSelectPin);
	_interruptNumber= digitalPinToInterrupt(interruptPin);	// other platforms may need to do this differently

	resetRxState();

	_txBufLen = 0;
	_txBufProgressIdx = 0;

}

bool Si4432_Transceiver::passAssertions() {
#ifdef USE_SAMPLE_CONFIG
	if (sizeof(RegisterSettings) != sizeof(SAMPLE_CONFIG)) {
		return false;
	}
#endif
	if (_singleton != NULL) {
		return false;	// cannot not be used more than once
	} else {
		_singleton = this;
	}

	uint8_t deviceType = _spi->read(Si4432_DEVICE_TYPE);
	if ( (deviceType != DEVICE_TYPE_RX_TRX) && (deviceType != DEVICE_TYPE_TX))  {
		return false;
	}

	uint8_t deviceVersion = _spi->read(Si4432_DEVICE_VERSION);
	if (deviceVersion != VERSION_Si4432B) {
		return false;
	}
	return true;
}


bool Si4432_Transceiver::init(const RegisterSettings *config, uint8_t address, boolean addressCheck) {
	_spi->begin();

	if (!passAssertions()) return false;

	resetRegisters();

	setStateIdle();

#ifdef USE_SAMPLE_CONFIG
	config=  (config != NULL) ? config : (const RegisterSettings*)SAMPLE_CONFIG;
#endif

	setupRegisters(config, address, addressCheck);

	startInterruptHandler();

	return true;
}

void Si4432_Transceiver::setupRegisters(const RegisterSettings *config, uint8_t address, boolean addressCheck) {
	// apply settings from the Si443x-Register-Settings_RevB1.xls sheet:

	_spi->writeBlock(Si4432_IF_FILTER_BANDWIDTH,       &config->_1C, 10);
	_spi->write(Si4432_AFC_LIMITER,                    config->_2A);
	_spi->writeBlock(Si4432_OOK_COUNTER_VALUE_1,       &config->_2C, 3);
	_spi->write(Si4432_RESERVED_58,                    config->_58);
	_spi->write(Si4432_AGC_OVERRIDE_1,                 config->_69);
	_spi->writeBlock(Si4432_TX_DATA_RATE_1,            &config->_6E, 5);
	_spi->writeBlock(Si4432_FREQUENCY_BAND_SELECT,     &config->_75, 3);

	// settings specific to this implementation:

	// note: there seem to be devices where this would need to be reversed
	_spi->write(Si4432_GPIO_CONFIGURATION_0, GPIO_TX_STATE_OUTPUT);
	_spi->write(Si4432_GPIO_CONFIGURATION_1, GPIO_RX_STATE_OUTPUT);

	// same as default - just to be real sure the constants are in sync
	_spi->write(Si4432_TX_FIFO_CONTROL_2, TXFAETHR);
	_spi->write(Si4432_RX_FIFO_CONTROL,  RXAFTHR);

	setTxPower(TXOUT_TXPOW_11DBM);	// with the crappy default antenna, higher is probably better


	// for compatibility with RadioHead library the same message format
	// is used here (even though this implementation does not care for most
	// of the headers).. but having some already functional library
	// as a sparing partner makes testing so much easier :-)

	// format: 4-byte preamble, 2-byte sync, 4-byte header, 1-byte payload length,
	// 0-255 bytes payload, 2-byte CRC (using header, length and payload)

	uint8_t  poly= DAC_CRC_CRC_16_IBM;	// except for compatibility anything could be used here
	_spi->write(Si4432_DATA_ACCESS_CONTROL, DAC_ENPACRX | DAC_ENPACTX | DAC_ENCRC | poly);

	uint16_t preambleLen= 8; 	// in nibbles.. max is 512

	_spi->write(Si4432_HEADER_CONTROL_1, HC_BCEN_BYTE_3 | HC_HDCH_BYTE_3);	// enable header check (destination address)
	_spi->write(Si4432_HEADER_CONTROL_2, HC_HDLEN_3_2_1_0 | HC_SYNCLEN_3_2 | ((preambleLen & 0x100) ? HC_PREALEN_BIT8 : 0));
	_spi->write(Si4432_PREAMBLE_LENGTH, preambleLen & 0xff);

	_spi->write(Si4432_CHECK_HEADER_3, address);	// // header byte is used for the "destination address" check
	_spi->write(Si4432_HEADER_ENABLE_3, addressCheck ? 0xff : 0x00); // bit-mask limits what it actually checked

	uint8_t sync[] = { 0x2D, 0xD4 };			// default from Excel sheet
	_spi->writeBlock(Si4432_SYNC_WORD_3, sync, sizeof(sync));
}

void Si4432_Transceiver::resetRegisters() {
	_spi->write(Si4432_OPERATING_MODE_FUNCTION_CONTROL_1, OMFC_SWRES);

	for (uint8_t i= 0;i<100; i++ ) {	// timeout in case the flag might be missed
		delay(1);	// prepared for potential future esp8266 experiments..

		// "user must wait for CHIPRDY after issuing a SWRESET" (respective interrupt is NOT
		// enabled so the status is polled here)
		if (_spi->read(Si4432_INTERRUPT_STATUS_2) & INT_ICHIPRDY) break;
	}
}

uint8_t Si4432_Transceiver::getDeviceStatus() {
	return _spi->read(Si4432_DEVICE_STATUS);
}

void Si4432_Transceiver::clearRxTxFIFO() {
	_spi->write(Si4432_OPERATING_MODE_FUNCTION_CONTROL_2, OMFC_FFCLRRX | OMFC_FFCLRTX);
	_spi->write(Si4432_OPERATING_MODE_FUNCTION_CONTROL_2, 0);
}
void Si4432_Transceiver::clearRxFIFO() {
	_spi->write(Si4432_OPERATING_MODE_FUNCTION_CONTROL_2, OMFC_FFCLRRX);
	_spi->write(Si4432_OPERATING_MODE_FUNCTION_CONTROL_2, 0);
}

boolean Si4432_Transceiver::setAFCLimiter(float center, float pullInRange) {
	// Auto-frequency calibration (see Si4432 registers.pdf page 30ff)
	// AFC_pull_in_range = ±AFCLimiter[7:0] x (hbsel+1) x 625 Hz
	// i.e. max is (255*625=0.159375MHz or 0.318750MHz)

	uint8_t hbsel= (center >= (MAX_FREQ/2));	// FREQBAND_HBSEL (480-960)

	float max= hbsel ? 0.318750 : 0.159375;

	if ((pullInRange < 0) || (pullInRange > max)) {
		return false;
	}

	uint8_t afcLimiter=  pullInRange * 1000000.0 / (hbsel ? 1250.0 : 625.0);
	_spi->write(Si4432_AFC_LIMITER, afcLimiter);

	return !(getDeviceStatus() & STATUS_FREQERR);
}

bool Si4432_Transceiver::setFrequency(float center, float afcPullInRange) {
	// the below calculation assumes NO "frequency hopping" and a NO "frequency offset"

	if (center < 240.0 || center > MAX_FREQ) {
		return false;
	}

	// "The RF carrier frequency can be calculated as follows (without frequency hopping/offset;
	// see Si4432 registers.pdf page 55ff):
	// fcarrier = (fb+24+fc/64000) x 10 x (hbsel+1)  [MHz],
	// where parameters fc(16-bit), fo(16-bit), fb(5-bit) and hbsel come from registers 73h–77h."

	// frequency band selection: see table on Si4430-31-32.pdf page 26:
	uint8_t hbsel= (center >= (MAX_FREQ/2)) & 0x1;	// FREQBAND_HBSEL (480-960)
	float t=  center / ((hbsel+1) * 10.0);
	float n=  floor(t);

	uint8_t fbsel = hbsel ? (FREQBAND_SBSEL |FREQBAND_HBSEL) : FREQBAND_SBSEL;	// use of FREQBAND_SBSEL is recommended
	fbsel |= ((uint8_t)n - 24);	// // fb: 0..23

	// fc= (center / ((hbsel+1) * 10)-n)*64000, i.e.  fc= (t-n)*64000
	uint16_t fc = (t - n) * 64000;


	_spi->write(Si4432_FREQUENCY_BAND_SELECT, fbsel);
	_spi->write(Si4432_NOMINAL_CARRIER_FREQUENCY_1, fc >> 8);
	_spi->write(Si4432_NOMINAL_CARRIER_FREQUENCY_0, fc & 0xff);

	return setAFCLimiter(center, afcPullInRange);
}

void Si4432_Transceiver::setTxPower(uint8_t power) {
	_spi->write(Si4432_TX_POWER, power);
}

void Si4432_Transceiver::startInterruptHandler() {
	_spi->write(Si4432_INTERRUPT_ENABLE_1, INT_ENTXFFAEM | INT_ENRXFFAFULL | INT_ENPKSENT | INT_ENPKVALID | INT_ENCRCERROR | INT_ENFFERR);
	_spi->write(Si4432_INTERRUPT_ENABLE_2, INT_ENPREAVAL);
	attachInterrupt(_interruptNumber, interruptServiceRoutine, FALLING);
}

void Si4432_Transceiver::interruptServiceRoutine() {
	if (_singleton != NULL) _singleton->isr();
}

void Si4432_Transceiver::isr() {
	// "The nIRQ pin will remain low until the microcontroller reads the Interrupt Status
	// Register(s) (Registers 03h–04h) containing the active Interrupt Status bit. The
	// nIRQ output signal will then be reset until the next change in status is detected"
	uint8_t status[2];
	_spi->readBlock(Si4432_INTERRUPT_STATUS_1, status, 2);

	if (status[0] & INT_ICRCERROR)  { handleCrcError(); }
	if (status[0] & INT_IPKVALID)   { handlePacketReceived(); }
	if (status[0] & INT_IPKSENT)    { handlePacketSent(); }
	if (status[0] & INT_IRXFFAFULL) { handleRxFifoAlmostFull(); }
	if (status[0] & INT_ITXFFAEM)   { handleTxFifoAlmostEmpty(); }
	if (status[0] & INT_IFFERR)     { handleFifoError(); }

	if (status[1] & INT_IPREAVAL)   { handlePreambleDetected(); }
}

void Si4432_Transceiver::setOperatingMode(uint8_t mode) {
	// using "Ready Mode" for IDLE state (power could be saved by using
	// "SLEEP Mode" or "STANDBY Mode" instead but for now it doesn't seem
	// to be worth the extra hassle)
	_spi->write(Si4432_OPERATING_MODE_FUNCTION_CONTROL_1, OMFC_READY_MODE | mode);
}

void Si4432_Transceiver::setStateIdle() {
	if (_shadowState != IDLE) {
		setOperatingMode(OMFC_STANDBY_MODE);
		_shadowState = IDLE;
	}
}

void Si4432_Transceiver::setStateRX(boolean force) {
	if (force || (_shadowState != RX)) {
		setOperatingMode(OMFC_RXON);
		_shadowState = RX;
	}
}

void Si4432_Transceiver::setStateTX(boolean force) {
	if (force || (_shadowState != TX)) {
		setOperatingMode(OMFC_TXON);
		_shadowState = TX;
	}
}

void Si4432_Transceiver::resetRxState() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_rxBufLen = 0;
		_rxMsgAvailable = false;
	}
}

bool Si4432_Transceiver::waitReceive(uint16_t timeoutMs) {
	uint32_t t= millis();
	while (!timeoutMs || ((millis() - t) < timeoutMs)) {
		yield();		// make sure this cannot crash the ESP8266!
		if (hasReceivedMsg()) {
			return true;
		}
	}
	return false;
}

bool Si4432_Transceiver::hasReceivedMsg() {
	if (!_rxMsgAvailable) {
		if (_shadowState != TX) {
			setStateRX();
		}
	}
	return _rxMsgAvailable;
}

bool Si4432_Transceiver::receiveMsg(uint8_t* buf, uint8_t* len) {
	boolean success= false;

	cli();  // disable SPI interrupts

	if (buf && len && hasReceivedMsg()) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			if (*len >= _rxBufLen) {
				*len = _rxBufLen;
				memcpy(buf, _rxBuf, *len);

				success= true;
			} else {
				// message is too big.. no point to return a partial message!
				*len = _rxBufLen;	// signal overflow
			}
			resetRxState();
		}
	}

	sei();

	return success;
}

void Si4432_Transceiver::startTX(boolean force) {
	// load FIFO with 1st chunk of the message to be sent
	// (transmission of remaining chunks - if any - will be triggered later via interrupt)
	_txBufProgressIdx = 0;
	handleTxFifoAlmostEmpty();

	_spi->write(Si4432_PACKET_LENGTH, _txBufLen);

	setStateTX(force); // start transmitter
}

bool Si4432_Transceiver::sendMsg(uint8_t destAddress, const uint8_t* data, uint8_t len) {
	bool success = false;

	cli();  // disable SPI interrupts

	if (len  && (len <= MAX_SI4432_MSG_LEN) && waitMsgSent(READY_TO_SEND_TIMEOUT)) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			_spi->write(Si4432_TRANSMIT_HEADER_3, destAddress);	// RadioHead specific address

			// note: three more header fields are available if needed (since I am using higher
			// level protocols for this stuff I don't really care about this header..)

			// load TX state
			memcpy(_txBuf, data, len);
			_txBufLen= len;

			startTX();

			success = true;
		}
	}

	sei();

	return success;
}

bool Si4432_Transceiver::waitMsgSent(uint16_t timeout) {
	uint32_t t= millis();
	while (!timeout || ((millis() - t) < timeout)) {
		yield();		// just in case: make sure this cannot crash the ESP8266!
		if (_shadowState != TX) { // wait for end of previous transmission
			return true;
		}
	}
	return false;
}

void Si4432_Transceiver::resetRX() {
	// completely reset RX
	resetRxState();
	clearRxFIFO();
	setStateRX(true);
}

//-------------- interrupt handlers -----------------------

void Si4432_Transceiver::handleCrcError() {
	resetRX();
}

void Si4432_Transceiver::handleRxFifoAlmostFull() {
	// a chunk of received data is available in the FIFO - copy it to the _rxBuf

	// note: MAX_SI4432_MSG_LEN may be smaller than the available data (even though that
	// would be some kind of MAX_SI4432_MSG_LEN configuration error since handlePacketReceived()
	// should then have fired before... still make sure to guard against malicious messages...)

	uint16_t len= 0;
	while (!len) {
		len= MAX_SI4432_MSG_LEN -_rxBufLen;

		if (RX_ALMOST_FULL_THRESHOLD < len) len= RX_ALMOST_FULL_THRESHOLD;

		if (!len) {
			// there is more data than fits the message buffer: might be a faulty configuration (incorrect
			// MAX_SI4432_MSG_LEN) or a corrupted transmission... the question is: what might be the
			// best recovery strategy here?

			// strategy: just avoid buffer overflow and let the thing fail later in handlePacketReceived().
			// makes sure that the FIFO data is eventually consumed..
			resetRxState();	// this should make enough space
		}
	}
	_spi->readBlock(Si4432_FIFO_ACCESS, _rxBuf + _rxBufLen, len);
	_rxBufLen += len;
}

void Si4432_Transceiver::handlePacketReceived() {
	// interrupt just signaled that last chuck of message's data is available in the FIFO;
	// previously received data: _rxBufLen

	uint8_t msgLen = _spi->read(Si4432_RECEIVED_PACKET_LENGTH);
	int16_t remaining = msgLen - _rxBufLen;

	if ((msgLen >  MAX_SI4432_MSG_LEN) || (remaining < 0) || (remaining > FIFO_SIZE))  {
		// FUBAR: "reboot"
		resetRX();
	} else {
		_spi->readBlock(Si4432_FIFO_ACCESS, _rxBuf + _rxBufLen, msgLen - _rxBufLen);

		// if needed the destination address used by the sender could be fetched here
		// (the field is already used by the automatic message filtering - if enabled - and
		// I don't care about it.. if you need the header, then you might want to add
		// the respective code here..)
		//		uint8_t destAddr = _spi->read(Si4432_RECEIVED_HEADER_3);

		_rxBufLen = msgLen;
		_rxMsgAvailable = true;
	}

	// "When in FIFO mode, the chip will automatically exit the TX or RX
	// State when either the ipksent or ipkvalid interrupt occurs"
	_shadowState = IDLE;
}

void Si4432_Transceiver::handlePacketSent() {
	// "When in FIFO mode, the chip will automatically exit the TX or RX
	// State when either the ipksent or ipkvalid interrupt occurs"
	_shadowState = IDLE;
}

void Si4432_Transceiver::handleTxFifoAlmostEmpty() {
	// fill transmitter FIFO with available message data.. if any

	if (_txBufProgressIdx < _txBufLen) {
		uint8_t len = _txBufLen - _txBufProgressIdx;
		if (len > TX_AVAILABLE_SIZE) {
			len = TX_AVAILABLE_SIZE;
		}
		_spi->writeBlock(Si4432_FIFO_ACCESS, _txBuf + _txBufProgressIdx, len);
		_txBufProgressIdx += len;
	}
}

void Si4432_Transceiver::handleFifoError() {
	// "[triggered] if there is a TX or RX FIFO Overflow or Underflow condition. [flag] is cleared
	// only by applying FIFO reset to the specific FIFO that caused the condition.

	clearRxTxFIFO(); // brute force

	if (_shadowState == TX) {
		startTX(true);	// retransmit existing buffer
	}
}

void Si4432_Transceiver::handlePreambleDetected() {
	// get ready for reception of message data
	clearRxFIFO();
	resetRxState();
}
