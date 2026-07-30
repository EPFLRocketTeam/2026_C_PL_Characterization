#pragma once

#include "Arduino.h"
#include "SPI.h"

#define DEVID_PRODUCT 0xFA // 372 in octal :)

// Data registers. Each axis data has a 12 bit value. Data is left justified, MSBFIRST.
// Register *_H contains the eight most significant bits (MSBs), and Register *_L contains the four least significant bits (LSBs) of the 12-bit value
#define XDATA_H 0x08
#define XDATA_L 0x09
#define YDATA_H 0x0A
#define YDATA_L 0x0B
#define ZDATA_H 0x0C
#define ZDATA_L 0x0D

// Peak Data registers.
#define MAXPEAK_X_H 0x15
#define MAXPEAK_X_L 0x16
#define MAXPEAK_Y_H 0x17
#define MAXPEAK_Y_L 0x18
#define MAXPEAK_Z_H 0x19
#define MAXPEAK_Z_L 0x1A

// ID registers
#define DEVID_AD 0x00
#define DEVID_MST 0x01
#define PARTID 0x02
#define REVID 0x03

// System registers
#define STATUS 0x04 // Status register

#define OFFSET_X 0x20
#define OFFSET_Y 0x21
#define OFFSET_Z 0x22

#define THRESH_ACT_X_H 0x23 // Activity threshold register
#define THRESH_ACT_X_L 0x24
#define THRESH_ACT_Y_H 0x25
#define THRESH_ACT_Y_L 0x26
#define THRESH_ACT_Z_H 0x27
#define THRESH_ACT_Z_L 0x28

#define TIME_ACT 0x29 // Activity time register

#define THRESH_INACT_X_H 0x2A // Inactivity threshold register
#define THRESH_INACT_X_L 0x2B
#define THRESH_INACT_Y_H 0x2C
#define THRESH_INACT_Y_L 0x2D
#define THRESH_INACT_Z_H 0x2E
#define THRESH_INACT_Z_L 0x2F

#define TIME_INACT_H 0x30 // Inactivity time register
#define TIME_INACT_L 0x31

#define THRESH_ACT2_X_H 0x32 // Motion Warning Threshold register
#define THRESH_ACT2_X_L 0x33
#define THRESH_ACT2_Y_H 0x34
#define THRESH_ACT2_Y_L 0x35
#define THRESH_ACT2_Z_H 0x36
#define THRESH_ACT2_Z_L 0x37

#define FIFO_SAMPLES 0x39 // FIFO samples register
#define FIFO_CTL 0x3A     // FIFO control register

#define INT1_MAP 0x3B // Interrupt 1 & 2 map register
#define INT2_MAP 0x3C

#define TIMING 0x3D    // Timing control register
#define MEASURE 0x3E   // Measurement control register
#define POWER_CTL 0x3F // Power control register

#define SELF_TEST 0x40 // Self test register
#define RESET 0x41     // Reset register
#define FIFO_DATA 0x42 // FIFO data register

// System bitmasks
#define THRESH_ACT_L_MASK 0x1F // Activity detection
#define ACT_EN_MASK 0xFE
#define ACT_EN_BIT 0x01
#define ACT_REF_MASK 0xFD

#define THRESH_INACT_L_MASK 0x1F // Inactivity detection
#define INACT_EN_MASK 0xFE
#define INACT_REF_MASK 0xFD

#define THRESH_ACT2_L_MASK 0x1F // Motion Warning
#define ACT2_EN_MASK 0xFE
#define ACT2_REF_MASK 0xFD

#define FIFO_SAMPLES_8_MASK 0xFE // FIFO control
#define FIFO_MODE_MASK 0xF9
#define FIFO_FORMAT_MASK 0xC7

#define INT_MAP_MASK 0xFF // Interrupt 1 and 2

#define EXT_SYNC_MASK 0xFE // Timing
#define EXT_CLK_MASK 0xFD
#define WAKEUP_RATE_MASK 0xE3
#define ODR_MASK 0x1F

#define BANDWIDTH_MASK 0xF8 // Measure
#define LOW_NOISE_MASK 0xF7
#define LINKLOOP_MASK 0xCF
#define AUTOSLEEP_MASK 0xBF

#define MODE_MASK 0xFC // Power Control
#define HPF_DISABLE_MASK 0xFB
#define LPF_DISABLE_MASK 0xF7
#define FILTER_SETTLE_MASK 0xEF
#define INSTANT_ON_THRESH_MASK 0xDF

#define ST_MASK 0xFE // Self test
#define ST_DONE_MASK 0xFD
#define USER_ST_MASK 0xFB

// Accelerometer Constants
#define SPI_SPEED 10000000 // ADXL372 supports up to 10MHz in SCLK frequency
#define SCALE_FACTOR 100   // mg per LSB
#define MG_TO_G 0.001      // g per mg

#define TIMEOUT_US 200000UL

enum FifoMode
{
    FIFO_DISABLED = 0b00,
    STREAM = 0b01,
    TRIGGER = 0b10,
    OLDEST_SAVED = 0b11
};

enum FifoFormat
{
    XYZ = 0b000,
    X = 0b001,
    Y = 0b010,
    XY = 0b011,
    Z = 0b100,
    XZ = 0b101,
    YZ = 0b110,
    XYZ_PEAK = 0b111
};

enum class FifoAxisOrder : uint8_t
{
    /*
     * The names describe the physical-axis sequence stored in the
     * three successive FIFO slots.
     * Necessary because of silicon issue :(
     */
    XYZ = 0, // slot 0 = X, slot 1 = Y, slot 2 = Z
    YZX = 1, // slot 0 = Y, slot 1 = Z, slot 2 = X
    ZXY = 2, // slot 0 = Z, slot 1 = X, slot 2 = Y

    UNKNOWN = 255
};

enum Odr
{
    ODR_400Hz = 0b000,
    ODR_800Hz = 0b001,
    ODR_1600Hz = 0b010,
    ODR_3200Hz = 0b011,
    ODR_6400Hz = 0b100
};

enum WakeUpRate
{
    WUR_52ms = 0b000,
    WUR_104ms = 0b001,
    WUR_208ms = 0b010,
    WUR_512ms = 0b011,
    WUR_2048ms = 0b100,
    WUR_4096ms = 0b101,
    WUR_8192ms = 0b110,
    WUR_24576ms = 0b111,
};

enum Bandwidth
{
    BW_200Hz = 0b000,
    BW_400Hz = 0b001,
    BW_800Hz = 0b010,
    BW_1600Hz = 0b011,
    BW_3200Hz = 0b100
};

enum LinkLoop
{
    DEFAULT = 0b00,
    LINKED = 0b01,
    LOOPED = 0b10
};

enum OperatingMode
{
    STANDBY = 0b00,
    WAKE_UP = 0b01,
    INSTANT_ON = 0b10,
    FULL_BANDWIDTH = 0b11
};

enum FilterSettlingPeriod
{
    FSP_370ms = 0,
    FSP_16ms = 1
};

enum InstantOnThreshold
{
    IOT_LOW_THRESH = 0,
    IOT_HIGH_THRESH = 1
};

enum InterruptFunction {
    DATA_RDY  = 0x01,
    FIFO_RDY  = 0x02,
    FIFO_FULL = 0x04,
    FIFO_OVR  = 0x08,
    INACT     = 0x10,
    ACT       = 0x20,
    ACT2      = 0x20,
    AWAKE     = 0x40,
    INT_LOW   = 0x80
};

struct TRIPLET {
    int16_t x;
    int16_t y;
    int16_t z;
} __attribute__((packed));

class ADXL372class
{
public:
    ADXL372class(int csPinInput, SPIClass &spi = SPI);
    virtual ~ADXL372class();

    void begin();
    void begin(uint32_t spiClockSpeed);
    bool reset();
    void end();
    void printDevice();
    byte getPartId();
    uint8_t getStatus();
    bool isConnected();
    bool selfTest();

    bool getRawAcceleration(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z);
    void readAcceleration(float &x, float &y, float &z);
    void readPeakAcceleration(float &x, float &y, float &z);

    void setOffsetTrims(float xOffset, float yOffset, float zOffset);

    void setActivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold);
    void enableActivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ);
    void setReferencedActivityProcessing(bool isReferenced);
    void setActivityTimer(uint8_t timerPeriod);
    
    void setInactivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold);
    void enableInactivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ);
    void setReferencedInactivityProcessing(bool isReferenced);
    void setInactivityTimer(uint16_t timerPeriod);

    void setMotionWarningThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold);
    void enableMotionWarningDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ);
    void setReferencedMotionWarningProcessing(bool isReferenced);

    bool readFifoData(TRIPLET *samples);
    void setFifoSamples(int sampleSize);
    void setFifoMode(FifoMode mode);
    void setFifoFormat(FifoFormat format);

    void setFifoReference(int16_t x, int16_t y, int16_t z);
    bool detectFifoAxisOrder(const TRIPLET *samples, uint16_t sample_count);
    bool isFifoAxisOrderDetected() const;
    FifoAxisOrder getFifoAxisOrder() const;

    void selectInt1Function(InterruptFunction function);
    void selectInt1Functions(uint8_t function);
    void selectInt2Function(InterruptFunction function);
    void selectInt2Functions(uint8_t function);

    void setOdr(Odr odr);
    void setWakeUpRate(WakeUpRate wur);
    void enableExternalClock(bool isEnabled);
    void enableExternalTrigger(bool isEnabled);

    void setBandwidth(Bandwidth bandwidth);
    void enableLowNoiseOperation(bool isEnabled);
    void setLinkLoopActivityProcessing(LinkLoop activityProcessing);
    void enableAutosleep(bool isEnabled);

    void setOperatingMode(OperatingMode opMode);
    void disableHighPassFilter(bool isDisabled);
    void disableLowPassFilter(bool isDisabled);
    void setFilterSettling(FilterSettlingPeriod filterSettling);
    void setInstantOnThreshold(InstantOnThreshold threshold);

private:
    int m_csPin;
    SPISettings m_spiSettings;
    SPIClass *m_spi;
    int m_sampleSize;
    FifoAxisOrder m_fifoAxisOrder;
    int16_t m_referenceX;
    int16_t m_referenceY;
    int16_t m_referenceZ;

    uint8_t formatThresholdValue(uint16_t thresholdValue);
    bool checkStandbyMode();
    int convertOffsetValue(float offset);

    uint8_t readRegister(byte regAddress);
    void readMultipleRegisters(byte regAddress, uint8_t *data, uint16_t count);
    void writeRegister(byte regAddress, uint8_t value);
    void updateRegister(byte regAddress, uint8_t value, byte mask);
    inline void select()
    {
        m_spi->beginTransaction(m_spiSettings);
        digitalWrite(m_csPin, LOW);
    }

    inline void deselect()
    {
        digitalWrite(m_csPin, HIGH);
        m_spi->endTransaction();
    }
};