#ifndef REGISTERS_SI4432_H_
#define REGISTERS_SI4432_H_


/*
	Si4432 registers (see https://www.silabs.com/documents/public/data-sheets/Si4430-31-32.pdf and
	https://www.silabs.com/documents/public/application-notes/AN440.pdf)
	also see https://www.sparkfun.com/datasheets/Wireless/General/RFM22.PDF - which seems to be using
	the same chip and contains some extra information (RFM22B uses a Si4432 rev B1)

	note: names were derived from description/abbreviations used in the Si4432 spec (e.g. see
	chapter "Si443.. register descriptions"); "Si4432" prefix is used for registers and register
	specific masks use prefix derived from register description.

	unused register/masks are commented out (might be useful at a later time)
*/
                                                                // POR default
#define Si4432_DEVICE_TYPE                                 0x00 // 0x8
	#define DEVICE_TYPE_RX_TRX               0x08 // EZRadioPRO (Si4432 docs are inconsistent here)
	#define DEVICE_TYPE_TX                   0x07 // DEFAULT?

// note: after version >= 0x06 these registers are split in different
// location, before 0x06 – shared at 0x72 reg (in case other SI chips
// were to be supported in the future).
#define Si4432_DEVICE_VERSION                              0x01 // --
	#define VERSION_Si4432B                  0x06 // Si4430/31/32 Rev B1: 00110

#define Si4432_DEVICE_STATUS                               0x02 // --
	//#define STATUS_CPS_IDLE                0x00 // bits 0..1: cps (according to RFM22 doc this is power state )
	//#define STATUS_CPS_RX                  0x01
	//#define STATUS_CPS_TX                  0x02
    // bit 2: unused
	#define STATUS_FREQERR                   0x08 // Frequency Error Status
	//#define STATUS_HEADERR                 0x10 // Header Error Status
	//#define STATUS_RXFFEM                  0x20 // RX FIFO Empty Status
	//#define STATUS_FFUNFL                  0x40 // RX/TX FIFO Underflow Status
	//#define STATUS_FFOVFL                  0x80 // RX/TX FIFO Overflow Status

/**************** Interrupts ********************/

#define Si4432_INTERRUPT_STATUS_1                          0x03 // --
	#define INT_ICRCERROR                     0x01 // CRC Error
	#define INT_IPKVALID                      0x02 // Valid Packet Received
	#define INT_IPKSENT                       0x04 // Packet Sent Interrupt
	//#define INT_IEXT                        0x08 // External Interrupt
	#define INT_IRXFFAFULL                    0x10 // RX FIFO Almost Full
	#define INT_ITXFFAEM                      0x20 // TX FIFO Almost Empty
	//#define INT_ITXFFAFULL                  0x40 // TX FIFO Almost Full
	#define INT_IFFERR                        0x80 // FIFO Underflow/Overflow Error

#define Si4432_INTERRUPT_STATUS_2                          0x04 // --
	//#define INT_IPOR                        0x01 // Power-on-Reset (POR).
	#define INT_ICHIPRDY                      0x02 // Module Ready (XTAL).
	//#define INT_ILBD                        0x04 // Low Battery Detect.
	//#define INT_IWUT                        0x08 // Wake-Up-Timer
	//#define INT_IRSSI                       0x10 // RSSI
	//#define INT_IPREAINVAL                  0x20 // Invalid Preamble Detected.
	#define INT_IPREAVAL                      0x40 // Valid Preamble Detected.
	//#define INT_ISWDET                      0x80 // Sync Word Detected.

#define Si4432_INTERRUPT_ENABLE_1                          0x05 // 0x00
	#define INT_ENCRCERROR                    0x01 // Enable CRC Error
	#define INT_ENPKVALID                     0x02 // Enable Valid Packet Received.
	#define INT_ENPKSENT                      0x04 // Enable Packet Sent.
	//#define INT_ENEXT                       0x08 // Enable External Interrupt.
	#define INT_ENRXFFAFULL                   0x10 // Enable RX FIFO Almost Full.
	#define INT_ENTXFFAEM                     0x20 // Enable TX FIFO Almost Empty
	//#define INT_ENTXFFAFULL                 0x40 // Enable TX FIFO Almost Full.
	#define INT_ENFFERR                       0x80 // Enable FIFO Underflow/Overflow.

#define Si4432_INTERRUPT_ENABLE_2                          0x06 // 0x03
	//#define INT_ENPOR                       0x01 // Enable POR.
	//#define INT_ENCHIPRDY                   0x02 // Enable Chip Ready (XTAL).
	//#define INT_ENLBD                       0x04 // Enable Low Battery Detect.
	//#define INT_ENWUT                       0x08 // Enable Wake-Up Timer.
	//#define INT_ENRSSI                      0x10 // Enable RSSI.
	//#define INT_ENPREAINVAL                 0x20 // Enable Invalid Preamble Detected.
	#define INT_ENPREAVAL                     0x40 // Enable Valid Preamble Detected.
	//#define INT_ENSWDET                     0x80 // Enable Sync Word Detected.

#define Si4432_OPERATING_MODE_FUNCTION_CONTROL_1            0x07 // 0x01
		// 5 modes in idle state:
	#define OMFC_STANDBY_MODE                 0x00 // STANDBY Mode.
	//#define OMFC_SLEEP_MODE                 0x20 // SLEEP Mode (bug in specs).
	//#define OMFC_SENSOR_MODE                0x40 // SENSOR Mode.
	#define OMFC_READY_MODE                   0x01 // READY Mode.
	#define OMFC_TUNE_MODE                    0x02 // TUNE Mode.

	#define OMFC_XTON                         0x01 // Xtal is ON.
	//#define OMFC_PLLON                      0x02 // PLL is ON.
	#define OMFC_RXON                         0x04 // RX on in Manual Receiver Mode.
	#define OMFC_TXON                         0x08 // TX on in Manual Transmit Mode.
	//#define OMFC_X32KSEL                    0x10 // 32,768 kHz Crystal Oscillator Select.
	//#define OMFC_ENWT                       0x20 // Enable Wake-Up-Timer.
	//#define OMFC_ENLBD                      0x40 // Enable Low Battery Detect.
	#define OMFC_SWRES                        0x80 // Software Register Reset Bit.

#define Si4432_OPERATING_MODE_FUNCTION_CONTROL_2            0x08 // 0x00
	#define OMFC_FFCLRTX                      0x01 // TX FIFO Reset/Clear.
	#define OMFC_FFCLRRX                      0x02 // RX FIFO Reset/Clear.
	//#define OMFC_ENLDM                      0x04 // Enable Low Duty Cycle Mode.
	//#define OMFC_AUTOTX                     0x08 // Automatic Transmission.
	//#define OMFC_RXMPK                      0x10 // RX Multi Packet.
	// bits 5..7: Enable Antenna Diversity.

//#define Si4432_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE       0x09 // 0x7f

//#define Si4432_UC_OUTPUT_CLOCK                           0x0a // 0x06
	// bits 0..2: Microcontroller Clock.
	//#define UCO_CLCK_ENLFC                  0x80 // Enable Low Frequency Clock.
	// bits: 4..5: Clock Tail.

/**************** GPIO ********************/

#define Si4432_GPIO_CONFIGURATION_0                        0x0b // 0x00
	#define GPIO_TX_STATE_OUTPUT              0x12 // bits 0..4: GPIO Pin Function Select:
	#define GPIO_RX_STATE_OUTPUT              0x15
	//#define GPIO_PULLUP_ENABLE              0x20 // Pullup Resistor Enable
#define Si4432_GPIO_CONFIGURATION_1                        0x0c // 0x00
//#define Si4432_GPIO_CONFIGURATION_2                      0x0d // 0x00

//#define Si4432_IO_PORT_CONFIGURATION                     0x0e // 0x00
	//#define IO_DIO0                         0x01 // Direct I/O for GPIO0.
	//#define IO_DIO1                         0x02 // Direct I/O for GPIO1.
	//#define IO_DIO2                         0x04 // Direct I/O for GPIO2.
	//#define IO_ITSDO                        0x08 // Interrupt Request Output on the SDO Pin.
	// bits: 4..6: External Interrupt Status.

/**************** ADC - analog digital converter ********************/

//#define Si4432_ADC_CONFIGURATION                         0x0f // 0x00
	// bits 0..1: ADC Sensor Amplifier Gain Selection.
	//#define ADC_ADCREF_BANDGAP              0x00 // bits 2..3: ADC Reference Voltage Selection.
	//#define ADC_ADCREF_VDD_3                0x08
	//#define ADC_ADCREF_VDD_2                0x0c
	//#define ADC_ADCSEL_INTERNAL_TEMP        0x00 // bits 4..6: ] ADC Input Source Selection.
	//#define ADC_ADCSEL_GPIO0_SINGLE_ENDED   0x10
	//#define ADC_ADCSEL_GPIO1_SINGLE_ENDED   0x20
	//#define ADC_ADCSEL_GPIO2_SINGLE_ENDED   0x30
	//#define ADC_ADCSEL_GPIO0_GPIO1_DIFF     0x40
	//#define ADC_ADCSEL_GPIO1_GPIO2_DIFF     0x50
	//#define ADC_ADCSEL_GPIO0_GPIO2_DIFF     0x60
	//#define ADC_ADCSEL_GND                  0x70
	//#define ADC_ADCSTART                    0x80 // bit 7: ADC Measurement Start Bit.
	//#define ADC_ADCDONE                     0x80

//#define Si4432_ADC_SENSOR_AMP_OFFSET                     0x10 // 0x00
//#define Si4432_ADC_VALUE                                 0x11 // 0x00
//#define Si4432_TEMP_SENSOR_CALIB                         0x12 // 0x20
	// bits 0..7: ignore for now.. don't care about temperature
//#define Si4432_TEMPERATURE_VALUE_OFFSET                  0x13 // 0x00

/**************** WUD - wake-up timer ********************/

#define Si4432_WAKEUP_TIMER_PERIOD_1                       0x14 // 0x03
	// bits 0..4: Wake Up Timer Exponent (R) Value (max: 20)
#define Si4432_WAKEUP_TIMER_PERIOD_2                       0x15 // 0x00
	// bits 0..7: Wake Up Timer Mantissa (M) Value - high-byte.
#define Si4432_WAKEUP_TIMER_PERIOD_3                       0x16 // 0x01
	// bits 0..7: Wake Up Timer Mantissa (M) Value - low-byte.
#define Si4432_WAKEUP_TIMER_VALUE_1                        0x17 // --
	// bits 0..7: Wake Up Timer Current Mantissa (M) Value - high-byte.
#define Si4432_WAKEUP_TIMER_VALUE_2                        0x18 // --
	// bits 0..7: Wake Up Timer Current Mantissa (M) Value - low-byte.

/**************** Low-Duty Cycle ********************/

//#define Si4432_LDC_MODE_DURATION                         0x19 // 0x00

/**************** Low Battery Detector ********************/

//#define Si4432_LOW_BATTERY_DETECTOR_THRESHOLD            0x1a // 0x14
//#define Si4432_BATTERY_VOLTAGE_LEVEL                     0x1b // --

/**************** RX Modem settings ********************/

#define Si4432_IF_FILTER_BANDWIDTH                         0x1c // 0x01

	// i.e. AFC enabled by default (and spreadsheet recommends use of AFC_1P5BYPASS )
#define Si4432_AFC_LOOP_GEARSHIFT_OVERRIDE                 0x1d // one doc says 0x40 other doc says 0x44 (01000100)!

	//#define AFC_PH0SIZE                    0x01 // If low, we will reset the Preamble detector
	//#define AFC_MATAP                      0x02 // Number of taps for moving average filter
	//#define AFC_1P5BYPASS                  0x04
	// bits 3..5: AFC High Gear Setting.
	//#define AFC_ENAFC                      0x40 // AFC Enable
	//#define AFC_AFCBD                      0x80 // AFC Wideband Enable (active high)

#define Si4432_AFC_TIMING_CONTROL                          0x1e // 0x0a

#define Si4432_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE           0x1f // 0x03

	// see static configuration derived from spreadsheet
#define Si4432_CLOCK_RECOVERY_OVERSAMPLING_RATIO           0x20 // 0x64
#define Si4432_CLOCK_RECOVERY_OFFSET_2                     0x21 // 0x01
#define Si4432_CLOCK_RECOVERY_OFFSET_1                     0x22 // 0x47
#define Si4432_CLOCK_RECOVERY_OFFSET_0                     0x23 // 0xae
#define Si4432_CLOCK_RECOVERY_TIMING_LOOP_GAIN_1           0x24 // 0x02
#define Si4432_CLOCK_RECOVERY_TIMING_LOOP_GAIN_0           0x25 // 0x8f

#define Si4432_RSSI                                        0x26 // --		// received signal strength indicator
//#define Si4432_RSSI_THRESHOLD                            0x27 // 0x1e
//#define Si4432_ANTENNA_DIVERSITY_1                       0x28 // --
//#define Si4432_ANTENNA_DIVERSITY_2                       0x29 // --
#define Si4432_AFC_LIMITER                                 0x2a // 0x00
//#define Si4432_AFC_CORRECTION_READ                       0x2b // 0x00

/**************** OOK ********************/
	// configured via initial setup
#define Si4432_OOK_COUNTER_VALUE_1                         0x2c // 0x18
	// bits 0..2: MSB of OOK count
	//#define OOKC_MADETEN                   0x08 // madeten: "moving average enable" -> required Manchester data)
	//#define OOKC_PEAKDETEN                 0x10 // peakdeten: "peak detector enable"
	//#define OOKC_OOKFRZEN                  0x20 // ookfrzen: "OOK FReeze"
	// bits 6..7: afc_corr: "AFC correction values - LSBs"
#define Si4432_OOK_COUNTER_VALUE_2                         0x2d // 0xbc	// bits 0..8: LSB of OOK count
#define Si4432_SLICER_PEAK_HOLD                            0x2e // 0x26
// reserved: 2f

#define Si4432_DATA_ACCESS_CONTROL                         0x30 // 0x8d
	#define DAC_CRC_CCITT                    0x00 // bits: 0..1: CRC Polynomial Selection.
	#define DAC_CRC_CRC_16_IBM               0x01
	#define DAC_CRC_IEC_16                   0x02
	#define DAC_CRC_BIACHEVA                 0x03
	#define DAC_ENCRC                        0x04 // CRC Enable.
	#define DAC_ENPACTX                      0x08 // Enable Packet TX Handling.
	//#define DAC_SKIP2PH                    0x10 // Skip 2nd Phase of Preamble Detection.
	//#define DAC_LSBFRST                    0x40 // LSB First Enable.
	//#define DAC_CRCDONLY                   0x20 // CRC Data Only Enable.
	#define DAC_ENPACRX                      0x80 // Enable Packet RX Handling.

//#define Si4432_EZMAC_STATUS                      0x31 // --
	//#define EZMAC_PKSENT                   0x01 // Packet Sent.
	//#define EZMAC_PKTX                     0x02 // Packet Transmitting.
	//#define EZMAC_CRCERROR                 0x04 // CRC Error.
	//#define EZMAC_PKVALID                  0x08 // Valid Packet Received.
	//#define EZMAC_PKRX                     0x10 // Packet Receiving.
	//#define EZMAC_PKSRCH                   0x20 // Packet Searching.
	//#define EZMAC_RXCRC1                   0x40 // If high, it indicates the last CRC received is all ones.

#define Si4432_HEADER_CONTROL_1                            0x32 // 0x0c
	// bits 0..3: Received Header Bytes to be Checked Against the Check Header Bytes.
	//#define HC_HDCH_NONE                   0x00
	//#define HC_HDCH_BYTE_0                 0x01 // Received Header check for byte 0
	//#define HC_HDCH_BYTE_1                 0x02 // Received Header check for byte 1
	//#define HC_HDCH_BYTE_2                 0x04 // Received Header check for byte 2
	#define HC_HDCH_BYTE_3                   0x08 // Received Header check for byte 3
	// bits 4..7: Broadcast Address (FFh) Check Enable.
	//#define HC_BCEN_NONE                   0x00
	//#define HC_BCEN_BYTE_0                 0x10	//  Broadcast address enable for header byte 0
	//#define HC_BCEN_BYTE_1                 0x20 //  Broadcast address enable for header byte 1
	//#define HC_BCEN_BYTE_2                 0x40 //  Broadcast address enable for header byte 2
	#define HC_BCEN_BYTE_3                   0x80	//  Broadcast address enable for header byte 3

#define Si4432_HEADER_CONTROL_2                     0x33 // 0x22
	#define HC_PREALEN_BIT8                  0x01
	// bits 1..2: Synchronization Word Length.
	//#define HC_SYNCLEN_3                   0x00 // Synchronization Word 3
	#define HC_SYNCLEN_3_2                   0x02 // Synchronization Word 3 and 2
	//#define HC_SYNCLEN_3_2_1               0x04 // Synchronization Word 3 and 2 and 1
	//#define HC_SYNCLEN_3_2_1_0             0x06 // Synchronization Word 3 and 2 and 1 and 0
	//#define HC_FIXPKLEN                    0x08
	// bits 4..6: Header Length.
	//#define HC_HDLEN_NOHEAD                0x00
	//#define HC_HDLEN_3                     0x10
	//#define HC_HDLEN_3_2                   0x20
	//#define HC_HDLEN_3_2_1                 0x30
	#define HC_HDLEN_3_2_1_0                 0x40
	//#define HC_SKIPSYN                     0x80

#define Si4432_PREAMBLE_LENGTH                             0x34 // 0x08  // count in nibbles
//#define Si4432_PREAMBLE_DETECTION_CONTROL_1              0x35 // 0x2a
	// bits 0..2: Preamble Detection Threshold
	// bits 3..7: preath

#define Si4432_SYNC_WORD_3                                 0x36 // 0x2d	 // 4th byte
#define Si4432_SYNC_WORD_2                                 0x37 // 0xd4	 // 3th byte
#define Si4432_SYNC_WORD_1                                 0x38 // 0x00	 // 2th byte
#define Si4432_SYNC_WORD_0                                 0x39 // 0x00	 // 1th byte

#define Si4432_TRANSMIT_HEADER_3                           0x3a // 0x00  // 4th byte of header to be transmitted
#define Si4432_TRANSMIT_HEADER_2                           0x3b // 0x00  // ..
#define Si4432_TRANSMIT_HEADER_1                           0x3c // 0x00
#define Si4432_TRANSMIT_HEADER_0                           0x3d // 0x00

#define Si4432_PACKET_LENGTH                               0x3e // 0x00

#define Si4432_CHECK_HEADER_3                              0x3f // 0x00  // 4th byte of the check header.
#define Si4432_CHECK_HEADER_2                              0x40 // 0x00  // ..
#define Si4432_CHECK_HEADER_1                              0x41 // 0x00
#define Si4432_CHECK_HEADER_0                              0x42 // 0x00

#define Si4432_HEADER_ENABLE_3                             0x43 // 0xff
#define Si4432_HEADER_ENABLE_2                             0x44 // 0xff
#define Si4432_HEADER_ENABLE_1                             0x45 // 0xff
#define Si4432_HEADER_ENABLE_0                             0x46 // 0xff

#define Si4432_RECEIVED_HEADER_3                           0x47 // --  // 4th byte of the received header.
#define Si4432_RECEIVED_HEADER_2                           0x48 // --  // ..
#define Si4432_RECEIVED_HEADER_1                           0x49 // --
#define Si4432_RECEIVED_HEADER_0                           0x4a // --

#define Si4432_RECEIVED_PACKET_LENGTH                      0x4b // --

// reserved: 4c-4e
#define Si4432_ADC8_CONTROL                                0x4f // 0x10
// reserved: 50-5f
#define Si4432_RESERVED_58						           0x58 // --  // for some reason the Excel sheet uses this "reserved" register.. WTF?

#define Si4432_CHANNEL_FILTER_COEFFICIENT_ADDRESS          0x60 // 0x00
// reserved: 61
#define Si4432_CRYSTAL_OSCILLATOR_CONTROL_TEST             0x62 // 0x24
// reserved: 63-68
#define Si4432_AGC_OVERRIDE_1                              0x69 // 0x20
// reserved: 6a-6c

/**************** TX Modulation Options ****************/

#define Si4432_TX_POWER                                    0x6d // 0x18
	// bit 0..2: TX Output Power (Si4432: from +20 dBM to –1 dBM in ~3 dB steps; RFM22 doc inconsistency!)
	#define TXOUT_TXPOW_M1DBM                0x00 // -1 dBm
	#define TXOUT_TXPOW_2DBM                 0x01 //  2 dBm
	#define TXOUT_TXPOW_5DBM                 0x02 //  5 dBm
	#define TXOUT_TXPOW_8DBM                 0x03 //  8 dBm
	#define TXOUT_TXPOW_11DBM                0x04 // 11 dBm
	#define TXOUT_TXPOW_14DBM                0x05 // 14 dBm
	#define TXOUT_TXPOW_17DBM                0x06 // 17 dBm
	#define TXOUT_TXPOW_20DBM                0x07 // 20 dBm
	// bit 3: LNA Switch Controller
	//#define TXOUT_LNA_SW                                       0x08
	// bit 4..7: reserved

#define Si4432_TX_DATA_RATE_1                              0x6e // 0x0a		// high-byte
#define Si4432_TX_DATA_RATE_0                              0x6f // 0x3d		// low-byte
//#define Si4432_MODULATION_MODE_CONTROL_1                 0x70 // 0x0c

/*
Data Clock Config.: No TX Data Clock (only for OOK and FSK)
Data Source:        Direct Mode using TX_Data via GPIO pin
Modulation Type:    Unmodulated Carrier
*/
#define Si4432_MODULATION_MODE_CONTROL_2                   0x71 // 0x0 (i.e. unmodulated carrier)
	#define CTRL2_MODTYP_NONE                0x00 // bits 0..1: modulation type
	#define CTRL2_MODTYP_OOK                 0x01
	#define CTRL2_MODTYP_FSK                 0x02
	#define CTRL2_MODTYP_GFSK                0x03 // recommended mode! USE IT!
	#define CTRL2_FD                         0x04 // frequency deviation (bit 8) - see Si4432_FREQUENCY_DEVIATION for bits 0..7
	#define CTRL2_ENINV                      0x08 // inversion of TX data
	#define CTRL2_DTMOD_DIRECT_MODE_GPIO     0x00 // bits 4..5: modulation data source
	#define CTRL2_DTMOD_DIRECT_MODE_SDI      0x10
	#define CTRL2_DTMOD_FIFO                 0x20
	#define CTRL2_DTMOD_PN9                  0x30
	#define CTRL2_TRCLK_NONE                 0x00 // bits 6..7: direct mode clock
	#define CTRL2_TRCLK_GPIO                 0x40
	#define CTRL2_TRCLK_SDO                  0x80
	#define CTRL2_TRCLK_NIRQ                 0xc0

#define Si4432_FREQUENCY_DEVIATION                         0x72 // 0x20
#define Si4432_FREQUENCY_OFFSET_1                          0x73 // 0x00
#define Si4432_FREQUENCY_OFFSET_2                          0x74 // 0x00
#define Si4432_FREQUENCY_BAND_SELECT                       0x75 // 0x75
	// bits 0..4: integer part (N)
	#define FREQBAND_HBSEL                   0x20 // bits 5: high-band select
	#define FREQBAND_SBSEL                   0x40 // bits 6: side band select

#define Si4432_NOMINAL_CARRIER_FREQUENCY_1                 0x76 // 0xbb
#define Si4432_NOMINAL_CARRIER_FREQUENCY_0                 0x77 // 0x80
// reserved: 78
/*
If frequency hopping is used then the step size should
be set first, and then the hopping channel, because the
frequency change occurs when the channel number is set.
*/
//#define Si4432_FREQUENCY_HOPPING_CHANNEL                 0x79 // 0x0
//#define Si4432_FREQUENCY_HOPPING_STEP_SIZE               0x7a // 0x0
// reserved: 7b
//#define Si4432_TX_FIFO_CONTROL_1                         0x7c // 0x37
#define Si4432_TX_FIFO_CONTROL_2                           0x7d // 0x04
#define Si4432_RX_FIFO_CONTROL                             0x7e // 0x37
#define Si4432_FIFO_ACCESS                                 0x7f // --


#endif /* REGISTERS_SI4432_H_ */
