/*
 Poor man's driver for SiliconLabs Si4432 transceivers.

 This implementation is meant to deal with the Si4432 modules that can be bought (on AliExpress, etc)
 for about $1.50 apiece (they also seem to be used in HopeRF RFM22). Even though there are other very similar
 modules (e.g. Si4430, Si4431) this implementation has NOT been tested with anything else but Si4432 and it
 only handles a subset of that transceiver's capabilities.

 Known limitations: only one module can be used in the same device, depends on hardware SPI, analog-digital
 converter (ADC) not used (see temperature sensor, etc), no frequency hopping, no EZMAC, no wake-up timer,
 no support for power saving sleep modes, etc. If you are looking for a more portable/comprehensive
 implementation you might want to use RH_RF22 from the RadioHead library instead - which wasn't an option
 for me due to licensing concerns (GPL is just a NO-GO). Also see Si4432_registers.h for stuff that is NOT used.

 Instructions for how to correctly wire a Si4432 to an Arduino can be found on the Internet and they are NOT
 repeated here (you may also search for RFM22).

 WARNING: The Si4432 will draw more power than what can be supplied by some 3v3 Arduino (50mA) - you will
 need a separate regulator to adequately power the transceiver!

 Compatibility: This implementation tries to preserve a certain compatibility with the existing
 RadioHead RF22 library, i.e. it uses the same message format (though, except for the "destination address"
 it doesn't care about the RadioHead specific headers).

 Copyright (C) 2018 Juergen Wothke

 License: CC BY-SA-NC (separate commercial license available on request)
 */
#ifndef JUST_SI4432_H_
#define JUST_SI4432_H_

#include <justSPI.h>
#include <registersSi4432.h>

#ifndef MAX_SI4432_MSG_LEN
#define MAX_SI4432_MSG_LEN 60
#endif

#define USE_SAMPLE_CONFIG


/**
 * This struct is used to configure the driver with the recommended register settings generated by the
 * Si443x-Register-Settings_RevB1.xls. It contains all those registers listed in the "REGISTERS Settings SUMMARY"
 * tab that are not marked as "This is the Default Value after RESET" (at least in those configurations I tried).
 *
 * The general approach is to completely configure the device with the respective init() call with no need
 * for later add-on setters.
 *
 * Note: As compared to the implementation used by RadioHead, additional registers are directly set here. For
 * ease of comparison respective fields are commented with a * below. In order to communicate with existing
 * RadioHead based devices respective configuration data would need to be properly regenerated.
 *
 * CAUTION: In order to use the Si443x-Register-Settings_RevB1.xls calculator make sure to use
 * original Excel WITH "Analysis ToolPak". LibreOffice Calc DOES NOT WORK!
 *
 * See SAMPLE_CONFIG in the .cpp for more information.
 */
typedef struct {
	uint8_t    _1C;   //  Si4432_IF_FILTER_BANDWIDTH
	uint8_t    _1D;   // *Si4432_AFC_LOOP_GEARSHIFT_OVERRIDE
	uint8_t    _1E;   // *Si4432_AFC_TIMING_CONTROL
	uint8_t    _1F;   //  Si4432_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE
	uint8_t    _20;   //  Si4432_CLOCK_RECOVERY_OVERSAMPLING_RATIO
	uint8_t    _21;   //  Si4432_CLOCK_RECOVERY_OFFSET_2
	uint8_t    _22;   //  Si4432_CLOCK_RECOVERY_OFFSET_1
	uint8_t    _23;   //  Si4432_CLOCK_RECOVERY_OFFSET_0
	uint8_t    _24;   //  Si4432_CLOCK_RECOVERY_TIMING_LOOP_GAIN_1
	uint8_t    _25;   //  Si4432_CLOCK_RECOVERY_TIMING_LOOP_GAIN_0

	uint8_t    _2A;   // *Si4432_AFC_LIMITER

	uint8_t    _2C;   //  Si4432_OOK_COUNTER_VALUE_1
	uint8_t    _2D;   //  Si4432_OOK_COUNTER_VALUE_2
	uint8_t    _2E;   //  Si4432_SLICER_PEAK_HOLD

	uint8_t    _30;   // *Si4432_DATA_ACCESS_CONTROL (ignored!)
	uint8_t    _32;   // *Si4432_HEADER_CONTROL_1 (ignored!)
	uint8_t    _33;   // *Si4432_HEADER_CONTROL_2 (ignored!)

	uint8_t    _58;   //  Si4432_RESERVED_58

	uint8_t    _69;   //  Si4432_AGC_OVERRIDE_1

	uint8_t    _6E;   //  Si4432_TX_DATA_RATE_1
	uint8_t    _6F;   //  Si4432_TX_DATA_RATE_0
	uint8_t    _70;   //  Si4432_MODULATION_MODE_CONTROL_1
	uint8_t    _71;   //  Si4432_MODULATION_MODE_CONTROL_2
	uint8_t    _72;   //  Si4432_FREQUENCY_DEVIATION

	uint8_t    _75;   // *Si4432_FREQUENCY_BAND_SELECT
	uint8_t    _76;   // *Si4432_NOMINAL_CARRIER_FREQUENCY_1
	uint8_t    _77;   // *Si4432_NOMINAL_CARRIER_FREQUENCY_0
} RegisterSettings;




/**
 * The Si4432 driver.
 */
class Si4432_Transceiver {
public:
	/**
	 * Creates instance of the driver.
	 *
	 * @param slaveSelectPin used by Si4432 module
	 * @param interruptPin used by Si4432 module
	 *
	 * Limitation: The current implementation supports use of exactly one instance - also see init().
	 */
	Si4432_Transceiver(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2);

	virtual ~Si4432_Transceiver() {}

	// ---------------- configuration --------------------

	/**
	 * Initialization to be called from sketch's "setup()".
	 *
	 * Can only be used once.
	 *
	 * @param settings base Si4432 base configuration generated using Si443x-Register-Settings_RevB1.xls
	 * @param address used for this device
	 * @param addressCheck false means any messages are accepted
	 */
	bool init(const RegisterSettings *config= 0, uint8_t address= 0xff, boolean addressCheck= true);

	/**
	 * This mimics the respective functionality from "RadioHead RF22".
	 *
	 * It overwrites respective settings made by setupRegisters() and its
	 * use should rather be avoided: The recommended approach is to always
	 * create the complete set of recommended register settings using "the Excel sheet"
	 * and not tinker with individual registers.
	 *
	 * It might be useful though in scenarios where the peer device is using RadioHead.
	 *
	 * @param center in MHz
	 * @param afcPullInRange e.g. 0.05 for 5%
	 */
	bool setFrequency(float center, float afcPullInRange = 0.05);

	/**
	 * Sets the power level used by the transmitter.
	 *
	 * Without a proper antenna range usually sucks, but unless you want to buy a real
	 * antenna you can still try to tinker with the transmission power. (Seriously, Who would
	 * buy a $1.50 transceiver to then buy a $100 antenna?)
	 *
	 * @param power see Si4432_TX_POWER constant section for options
	 */
	void setTxPower(uint8_t power);

	// ---------------- receiver --------------------

	/**
	 * Checks if a message has been received.
	 */
	bool hasReceivedMsg();

	/**
	 * Waits for reception of a message.
	 *
	 * @param timeoutMs in milliseconds; 0 means indefinitely
	 */
	bool waitReceive(uint16_t timeout);

	/**
	 * Polls for reception of message.
	 *
	 * @param len returns actual length in case of success, may return exceeded length in case of failure
	 * @return true If data was available/returned (completes reception cycle).
	 */
	bool receiveMsg(uint8_t* buf, uint8_t* len);


	// ---------------- transmitter --------------------

	/**
	 * Triggers the sending of a message.
	 *
	 * @param destAddress address of the targeted receiver
	 * @param data buffer containing the message
	 * @param len length of the message
	 */
	bool sendMsg(uint8_t destAddress, const uint8_t* data, uint8_t len);

	/**
	 * Waits until previous "sendMsg" has completed.
	 */
	bool waitMsgSent(uint16_t timeout);

protected:
	bool passAssertions();
	void resetRegisters();	// Triggers software register reset on Si4432
	void setupRegisters(const RegisterSettings *config, uint8_t address, boolean addressCheck);
	boolean setAFCLimiter(float center, float pullInRange);

	// --- interrupt handling
	void startInterruptHandler();
	static void interruptServiceRoutine();
	void isr();	// actual interrupt service routine

	void handleCrcError();
	void handlePacketReceived();
	void handlePacketSent();
	void handleRxFifoAlmostFull();
	void handleTxFifoAlmostEmpty();
	void handleFifoError();
	void handlePreambleDetected();

	// --- Si4432-side-state handling
	uint8_t getDeviceStatus();
	void setOperatingMode(uint8_t mode);	// see modes in Si4432_OPERATING_MODE_FUNCTION_CONTROL_1 constant section
	void clearRxTxFIFO();
	void clearRxFIFO();

	// --- internal state handling
	void setStateIdle();
	void setStateRX(boolean force= false);
	void setStateTX(boolean force= false);

	// receiver
	void resetRX();
	void resetRxState();

	// transmitter
	void startTX(boolean force= false);

protected:
	static Si4432_Transceiver *_singleton;

	JustSPI *_spi;
	uint8_t _interruptNumber;

	typedef enum {
//		SHUTDOWN,
		IDLE,
		TX,
		RX,
	} PrimaryState;

	volatile PrimaryState _shadowState;	// redundant to state on Si4432

	uint8_t _address;
	boolean _addressCheckEnabled;

	// receiver related state
	uint8_t _rxBuf[MAX_SI4432_MSG_LEN];
	volatile uint8_t _rxBufLen;
	volatile bool _rxMsgAvailable;


	// transmitter related state
	uint8_t _txBuf[MAX_SI4432_MSG_LEN];
	volatile uint8_t _txBufLen;
	volatile uint8_t _txBufProgressIdx;	// start of data that still needs to be sent
};

#endif 
