#include "debug.h"
#include "ADXL371.h"

//=================================================
// Constructor
//=================================================
ADXL371class::ADXL371class(int csPinInput, SPIClass &spi)
{
    m_csPin = csPinInput;
    m_spi = &spi;
    m_spiSettings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0);
    m_sampleSize = 0;
}

//=================================================
// Initialization
//=================================================
void ADXL371class::begin()
{
    m_spi->begin();
    //m_spi->beginTransaction(SPISettings(SPI_SPEED)); // CPHA = CPOL = 0

    digitalWrite(m_csPin, HIGH);                                       // Pin ready
    pinMode(m_csPin, OUTPUT);                                          // Setting chip select pin
}

void ADXL371class::begin(uint32_t spiClockSpeed)
{
    m_spiSettings = SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0);
    begin();
}

bool ADXL371class::reset()
{
    setOperatingMode(STANDBY);
    delay(1);

    writeRegister(RESET, 0x52);
    delay(5);

    return isConnected();
}

bool ADXL371class::isConnected() {
    uint8_t devidAd  = readRegister(DEVID_AD);
    uint8_t devidMst = readRegister(DEVID_MST);
    uint8_t partId   = readRegister(PARTID);

    return devidAd  == 0xAD &&
           devidMst == 0x1D &&
           partId   == DEVID_PRODUCT;
}

void ADXL371class::end()
{
    // set some adresses here
    digitalWrite(m_csPin, HIGH);
    m_spi->end();
}

void ADXL371class::printDevice()
{
#if DEBUG_
    byte devidAd = readRegister(DEVID_AD);
    byte devidMst = readRegister(DEVID_MST);
    byte partId = readRegister(PARTID);
    byte revId = readRegister(REVID);
    byte status = readRegister(STATUS);

    Serial.print("DEVID_AD: 0x");
    Serial.println(devidAd, HEX);

    Serial.print("DEVID_MST: 0x");
    Serial.println(devidMst, HEX);

    Serial.print("PARTID: 0x");
    Serial.println(partId, HEX);

    Serial.print("REVID: 0x");
    Serial.println(revId, HEX);

    Serial.print("STATUS: 0x");
    Serial.println(status, HEX);
#endif
}

//=================================================
// Infos/measures getter
//=================================================
byte ADXL371class::getPartId() {
    // Reads and returns the physical PARTID register (0xFA expected)
    return readRegister(PARTID);
}

uint8_t ADXL371class::getStatus()
{
    return readRegister(STATUS);
}

bool ADXL371class::getRawAcceleration(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z)
{
    raw_x = 0;
    raw_y = 0;
    raw_z = 0;


    const uint32_t start = micros();
    while ((readRegister(STATUS) & DATA_RDY) == 0) {
        if (micros() - start >= TIMEOUT_US) {
            return false;
        }
    }
    // Waiting for status register

    uint8_t buf[6];
    readMultipleRegisters(XDATA_H, buf, 6);

    // The register is left justified. *DATA_H has bits 11:4 of the register. *DATA_L has bits 3:0.
    raw_x = (int16_t)((buf[0] << 8) | buf[1]) >> 4;
    raw_y = (int16_t)((buf[2] << 8) | buf[3]) >> 4;
    raw_z = (int16_t)((buf[4] << 8) | buf[5]) >> 4;

    return true;
}

//=================================================
// FIFO configuration
//=================================================
void ADXL371class::setFifoMode(FifoMode mode)
{
    if(!checkStandbyMode()) return;
    byte modeShifted = mode << 1; // starts from bit 1 in register
    updateRegister(FIFO_CTL, modeShifted, FIFO_MODE_MASK);
}

void ADXL371class::setFifoFormat(FifoFormat format)
{
    if(!checkStandbyMode()) return;
    byte formatShifted = format << 3; // starts from bit 3 in register
    updateRegister(FIFO_CTL, formatShifted, FIFO_FORMAT_MASK);
}

void ADXL371class::setFifoSamples(int sampleSize)
{
    if(!checkStandbyMode()) return;
    if (sampleSize < 1) sampleSize = 1;
    if (sampleSize > 511)
    {
#if DEBUG_
        Serial.println("WARNING: this driver supports watermarks 1..511");
#endif
        sampleSize = 511;
    }

    m_sampleSize = sampleSize;
    // Watermark is an entry count, not count minus one.

    writeRegister(FIFO_SAMPLES,(uint8_t)(sampleSize & 0xFF));   // Set watermark level
    updateRegister(FIFO_CTL,(uint8_t)((sampleSize >> 8) & 0x01),FIFO_SAMPLES_8_MASK);   // Associate FIFO_FULL to watermark level
}

//=================================================
// Read FIFO
//=================================================
bool ADXL371class::readFifoWords(uint8_t *buffer, uint16_t words, uint16_t capacity) {
    if (!buffer || words > 512 || capacity < words * 2U) return false;
    if (words == 0) return true;
    select();
    m_spi->transfer((FIFO_DATA << 1) | 1);
    for (uint16_t i = 0; i < words * 2U; ++i) buffer[i] = m_spi->transfer(0);
    deselect();
    return true;
}

AdxlFifoResult ADXL371class::decodeFifoWords(const uint8_t *buffer, uint16_t words,
                                           TRIPLET *samples, uint16_t capacity) const {
    if (!buffer || !samples || words > 510 || words % 3 || capacity < words / 3)
        return AdxlFifoResult::InvalidRequest;
    for (uint16_t i = 0; i < words / 3; ++i) {
        const uint8_t *b = buffer + i * 6;
        if (!(b[1] & 1) || (b[3] & 1) || (b[5] & 1)) return AdxlFifoResult::MarkerMismatch;
        int16_t v[3];
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const uint16_t u = ((uint16_t)b[axis * 2] << 4) | (b[axis * 2 + 1] >> 4);
            v[axis] = (u & 0x800) ? (int16_t)((int32_t)u - 4096) : (int16_t)u;
        }
        samples[i].x = v[0]; samples[i].y = v[1]; samples[i].z = v[2];
    }
    return AdxlFifoResult::Ok;
}

AdxlFifoResult ADXL371class::readFifoRawData(uint8_t *buffer, uint16_t capacity,
                                                uint16_t &entries)
{
    entries = getFifoEntries();
    if (buffer == nullptr || m_sampleSize < 3 || m_sampleSize > 510 ||
        m_sampleSize % 3 != 0 || capacity < (uint16_t)(2 * m_sampleSize)) {
        return AdxlFifoResult::InvalidRequest;
    }
    if (entries < (uint16_t)m_sampleSize) return AdxlFifoResult::TooFewEntries;
    select();
    m_spi->transfer((FIFO_DATA << 1) | 1);
    for (uint16_t i = 0; i < (uint16_t)(2 * m_sampleSize); ++i) {
        buffer[i] = m_spi->transfer(0x00);
    }
    deselect();
    return AdxlFifoResult::Ok;
}

AdxlFifoResult ADXL371class::decodeFifoData(const uint8_t *buffer, TRIPLET *samples,
                                          uint16_t sampleCapacity) const
{
    if (buffer == nullptr || samples == nullptr || m_sampleSize < 3 ||
        m_sampleSize > 510 || m_sampleSize % 3 != 0 ||
        sampleCapacity < (uint16_t)(m_sampleSize / 3)) {
        return AdxlFifoResult::InvalidRequest;
    }
    for (uint16_t i = 0; i < (uint16_t)(m_sampleSize / 3); ++i) {
        const uint8_t *b = buffer + 6 * i;
        if (!(b[1] & 1) || (b[3] & 1) || (b[5] & 1)) {
            return AdxlFifoResult::MarkerMismatch;
        }
        // Explicit 12-bit sign extension, without shifting signed negatives.
        int16_t values[3];
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const uint16_t value = ((uint16_t)b[2 * axis] << 4) | (b[2 * axis + 1] >> 4);
            values[axis] = (value & 0x800) ? (int16_t)((int32_t)value - 4096) : (int16_t)value;
        }
        samples[i].x = values[0];
        samples[i].y = values[1];
        samples[i].z = values[2];
    }
    return AdxlFifoResult::Ok;
}

bool ADXL371class::readFifoData(TRIPLET *samples)
{
    // Compatibility wrapper; the logger uses the split read/decode API.
    uint8_t buffer[1024];
    uint16_t entries;
    if (readFifoRawData(buffer, sizeof(buffer), entries) != AdxlFifoResult::Ok) return false;
    return decodeFifoData(buffer, samples, m_sampleSize / 3) == AdxlFifoResult::Ok;
}

uint16_t ADXL371class::getFifoEntries()
{
    uint8_t entries[2];

    // Registers 0x06 and 0x07 are consecutive.
    readMultipleRegisters(FIFO_ENTRIES2, entries, 2);

    return
        ((uint16_t)(entries[0] & 0x03) << 8) |
        (uint16_t)entries[1];
}

bool ADXL371class::drainFifo(uint32_t *discardedWords)
{
    // Normally only one pass should be necessary because the ADC
    // is already stopped using EXT_SYNC.
    //
    // A few passes make this robust against an in-flight conversion
    // completing just as EXT_SYNC is changed.
    for (uint8_t pass = 0; pass < 4; pass++) {

        const uint16_t entries = getFifoEntries();

        if (entries == 0) {
            return true;
        }

        select();

        // Start FIFO_DATA read.
        m_spi->transfer((FIFO_DATA << 1) | 1);

        if (discardedWords) *discardedWords += entries;

        // One FIFO entry is one 16-bit axis word.
        const uint32_t bytes_to_discard =
            (uint32_t)entries * 2U;

        for (uint32_t i = 0; i < bytes_to_discard; i++) {
            m_spi->transfer(0x00);
        }

        deselect();
    }

    // It should definitely be empty by now.
    return getFifoEntries() == 0;
}

//=================================================
// Interrupt mapping
//=================================================
void ADXL371class::selectInt1Function(InterruptFunction function)
{
    writeRegister(INT1_MAP, function);
}

void ADXL371class::selectInt1Functions(uint8_t function)
{
    writeRegister(INT1_MAP, function);
}

void ADXL371class::selectInt2Function(InterruptFunction function)
{
    writeRegister(INT2_MAP, function);
}

void ADXL371class::selectInt2Functions(uint8_t function)
{
    writeRegister(INT2_MAP, function);
}

//=================================================
// Measurement config
//=================================================
void ADXL371class::setOdr(Odr odr)
{
#if DEBUG_
    int currentBandwidth = readRegister(MEASURE) & 0b00000111; // Warning-only check.
    if ((int)odr < currentBandwidth) Serial.println("WARNING: ODR must be at least double the bandwidth, to not violate the Nyquist criteria. Otherwise signal integrity will not be maintained");
#endif

    byte odrShifted = odr << 5; // odr bits start from bit 5
    updateRegister(TIMING, odrShifted, ODR_MASK);
}

void ADXL371class::setBandwidth(Bandwidth bandwidth)
{
    int currentOdr = (readRegister(TIMING) & 0b11100000) >> 5; // Get only the ODR bits
    if ((int)bandwidth > currentOdr)
    {
#if DEBUG_
        Serial.println("WARNING: Bandwidth must be no greater than half the ODR, to not violate the Nyquist criteria. Otherwise signal integrity will not be maintained");
#endif
    }
    updateRegister(MEASURE, bandwidth, BANDWIDTH_MASK);
}

void ADXL371class::setFilterSettling(FilterSettlingPeriod filterSettling)
{
    byte valueShifted = filterSettling << 4; // bit 4 in register
    updateRegister(POWER_CTL, valueShifted, FILTER_SETTLE_MASK);
}

void ADXL371class::disableHighPassFilter(bool isDisabled)
{
    byte valueShifted = isDisabled << 2; // bit 2 in register
    updateRegister(POWER_CTL, valueShifted, HPF_DISABLE_MASK);
}

void ADXL371class::disableLowPassFilter(bool isDisabled)
{
    byte valueShifted = isDisabled << 3; // bit 3 in register
    updateRegister(POWER_CTL, valueShifted, LPF_DISABLE_MASK);
}

//=================================================
// Operating mode
//=================================================
void ADXL371class::setOperatingMode(OperatingMode opMode)
{
    updateRegister(POWER_CTL, opMode, MODE_MASK);
}

bool ADXL371class::checkStandbyMode()
{
    byte mode = readRegister(POWER_CTL);
    mode &= 0x03;

    if (mode != STANDBY)
    {
#if DEBUG_
        Serial.println("WARNING: Activity, Inactivity and FIFO can only be set while in standy mode");
#endif
        return false;
    }
    return true;
}

void ADXL371class::enableLowNoiseOperation(bool isEnabled)
{
    byte valueShifted = isEnabled << 3; // bit 3 in register
    updateRegister(MEASURE, valueShifted, LOW_NOISE_MASK);
}

//=================================================
// Register management
//=================================================
uint8_t ADXL371class::readRegister(byte regAddress)
{
    select();

    regAddress = regAddress << 1 | 1; // Reading from a register
    m_spi->transfer(regAddress);
    uint8_t value = m_spi->transfer(0x00); // Transfering dummy byte to recieve data from accelerometer

    deselect();
    return value;
}

void ADXL371class::readMultipleRegisters(byte regAddress, uint8_t *data, uint16_t count)
{
    select();

    regAddress = regAddress << 1 | 1; // Reading from a register
    m_spi->transfer(regAddress);

    for (uint16_t idx = 0; idx < count; idx++) {
        data[idx] = m_spi->transfer(0x00); // Transfering dummy byte to recieve data from accelerometer
    }

    deselect();
}

void ADXL371class::writeRegister(byte regAddress, uint8_t value)
{
    select();

    regAddress = regAddress << 1; // Writing to a register
    m_spi->transfer(regAddress);
    m_spi->transfer(value);
    
    deselect();
}

void ADXL371class::updateRegister(byte regAddress, uint8_t value, byte preserveMask)
{
    // Need to use bitmasks to only change the desired bits in the registers
    uint8_t current = readRegister(regAddress);
    current &= preserveMask;
    current |= value;
    writeRegister(regAddress, current);
}

//=================================================
// Test function
//=================================================
bool ADXL371class::selfTest()
{
    constexpr uint8_t ST_DONE_BIT = 0x02;
    constexpr uint32_t SELF_TEST_TIMEOUT_US = 1000000UL;
    constexpr uint16_t MAX_WINDOW_SAMPLES = 256;

    // Datasheet procedure:
    // - low-pass filter enabled
    // - measurement mode
    // - wait for full filter settling
    setOperatingMode(STANDBY);
    disableLowPassFilter(false);
    setFilterSettling(FSP_462_5ms);
    setOperatingMode(FULL_BANDWIDTH);

    // Recommended settling time is 462.5 ms when the filter is enabled.
    delay(470);

    // Determine the current ODR so that we know how many samples
    // correspond to 50 ms.
    uint8_t timing = readRegister(TIMING);
    uint8_t odr_code = (timing >> 5) & 0x07;

    uint16_t odr_hz;

    switch (odr_code) {
        case 0b000: odr_hz = 320;  break;
        case 0b001: odr_hz = 640;  break;
        case 0b010: odr_hz = 1280; break;
        case 0b011: odr_hz = 2560; break;
        case 0b100: odr_hz = 5120; break;

        default:
            setOperatingMode(STANDBY);
            return false;
    }

    // Number of samples in 50 ms.
    const uint16_t window_samples = odr_hz / 20;

    if (window_samples == 0 ||
        window_samples > MAX_WINDOW_SAMPLES) {
        setOperatingMode(STANDBY);
        return false;
    }

    // Sum of samples from the first 50 ms.
    int32_t first_sum = 0;
    uint16_t first_count = 0;

    // Rolling buffer containing the most recent 50 ms of samples.
    int16_t last_window[MAX_WINDOW_SAMPLES];
    uint16_t last_count = 0;
    uint16_t last_index = 0;
    int32_t last_sum = 0;

    // Make sure ST is clear first.
    updateRegister(SELF_TEST, 0x00, ST_MASK);

    // Start self test.
    updateRegister(SELF_TEST, 0x01, ST_MASK);

    const uint32_t start_time = micros();

    while ((readRegister(SELF_TEST) & ST_DONE_BIT) == 0) {

        if ((uint32_t)(micros() - start_time) >= SELF_TEST_TIMEOUT_US) {
            updateRegister(SELF_TEST, 0x00, ST_MASK);
            setOperatingMode(STANDBY);
            return false;
        }

        int16_t x, y, z;

        // This waits for DATA_RDY, so every value used here is a
        // new acceleration sample rather than repeated reads.
        if (!getRawAcceleration(x, y, z)) {
            updateRegister(SELF_TEST, 0x00, ST_MASK);
            setOperatingMode(STANDBY);
            return false;
        }

        // First 50 ms.
        if (first_count < window_samples) {
            first_sum += z;
            first_count++;
        }

        // Maintain the last 50 ms as a circular buffer.
        if (last_count < window_samples) {
            last_window[last_index] = z;
            last_sum += z;
            last_count++;
        }
        else {
            last_sum -= last_window[last_index];
            last_window[last_index] = z;
            last_sum += z;
        }

        last_index++;

        if (last_index >= window_samples) {
            last_index = 0;
        }
    }

    // Clear self test.
    updateRegister(SELF_TEST, 0x00, ST_MASK);

    // Leave the part in a predictable state.
    setOperatingMode(STANDBY);

    if (first_count != window_samples ||
        last_count != window_samples) {
        return false;
    }

    const float first_average =
        (float)first_sum / (float)first_count;

    const float last_average =
        (float)last_sum / (float)last_count;

    float delta = last_average - first_average;

    if (delta < 0.0f) {
        delta = -delta;
    }

    #if DEBUG_
#if DEBUG_
    Serial.print("ADXL371 self-test first average: ");
#endif
#if DEBUG_
    Serial.println(first_average);
#endif

#if DEBUG_
    Serial.print("ADXL371 self-test last average: ");
#endif
#if DEBUG_
    Serial.println(last_average);
#endif

#if DEBUG_
    Serial.print("ADXL371 self-test delta: ");
#endif
#if DEBUG_
    Serial.println(delta);
#endif
    #endif

    return delta > 5.0f;
}

//=================================================
// g Converted reader
//=================================================
void ADXL371class::readAcceleration(float &x, float &y, float &z)
{
    int16_t raw_x, raw_y, raw_z;
    if(!getRawAcceleration(raw_x, raw_y, raw_z)){
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        return;
    }

    // Converting buf axis data to acceleration in g unit
    x = raw_x * SCALE_FACTOR * MG_TO_G;
    y = raw_y * SCALE_FACTOR * MG_TO_G;
    z = raw_z * SCALE_FACTOR * MG_TO_G;
}

void ADXL371class::readPeakAcceleration(float &xPeak, float &yPeak, float &zPeak)
{
    const uint32_t start = micros();
    while ((readRegister(STATUS) & DATA_RDY) == 0) {
        if (micros() - start >= TIMEOUT_US) {
            xPeak = 0.0f;
            yPeak = 0.0f;
            zPeak = 0.0f;
            return;
        }
    }

    uint8_t buf[6];
    readMultipleRegisters(MAXPEAK_X_H, buf, 6);

    short rawX = buf[0] << 8 | buf[1];
    short rawY = buf[2] << 8 | buf[3];
    short rawZ = buf[4] << 8 | buf[5];

    rawX = rawX >> 4;
    rawY = rawY >> 4;
    rawZ = rawZ >> 4;

    // Converting buf axis data to acceleration in g unit
    xPeak = rawX * SCALE_FACTOR * MG_TO_G;
    yPeak = rawY * SCALE_FACTOR * MG_TO_G;
    zPeak = rawZ * SCALE_FACTOR * MG_TO_G;
}

//=================================================
// Useless functions for FH2 payload
//=================================================
void ADXL371class::setOffsetTrims(float xOffset, float yOffset, float zOffset)
{
    int xOffsetConverted = convertOffsetValue(xOffset);
    int yOffsetConverted = convertOffsetValue(yOffset);
    int zOffsetConverted = convertOffsetValue(zOffset);
    // No need to mask or bitshift for the offset registers
    writeRegister(OFFSET_X, xOffsetConverted);
    writeRegister(OFFSET_Y, yOffsetConverted);
    writeRegister(OFFSET_Z, zOffsetConverted);
}

uint8_t ADXL371class::formatThresholdValue(uint16_t thresholdValue)
{
    if (thresholdValue > 0x7FF) // The threshold value is an 11-bit value. So the max limit is 0x7FF.
    {
#if DEBUG_
        Serial.println("WARNING: Threshold value limit is 2047, going beyond this value will have unintended effects.");
#endif
    }
    return thresholdValue = thresholdValue >> 3; // Get 8 MSB
}

void ADXL371class::setActivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
{
    checkStandbyMode();
    uint8_t xThresh8Msb = formatThresholdValue(xThreshold);
    writeRegister(THRESH_ACT_X_H, xThresh8Msb);
    updateRegister(THRESH_ACT_X_L, (xThreshold << 5), THRESH_ACT_L_MASK);

    uint8_t yThresh8Msb = formatThresholdValue(yThreshold);
    writeRegister(THRESH_ACT_Y_H, yThresh8Msb);
    updateRegister(THRESH_ACT_Y_L, (yThreshold << 5), THRESH_ACT_L_MASK);

    uint8_t zThresh8Msb = formatThresholdValue(zThreshold);
    writeRegister(THRESH_ACT_Z_H, zThresh8Msb);
    updateRegister(THRESH_ACT_Z_L, (zThreshold << 5), THRESH_ACT_L_MASK);
}

void ADXL371class::enableActivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    checkStandbyMode();
    updateRegister(THRESH_ACT_X_L, isEnabledX, ACT_EN_MASK); // bit 1 in register
    updateRegister(THRESH_ACT_Y_L, isEnabledY, ACT_EN_MASK);
    updateRegister(THRESH_ACT_Z_L, isEnabledZ, ACT_EN_MASK);
}

void ADXL371class::setReferencedActivityProcessing(bool isReferenced)
{
    checkStandbyMode();
    updateRegister(THRESH_ACT_X_L, isReferenced << 1, ACT_REF_MASK); // bit 1 in register
}

void ADXL371class::setActivityTimer(uint8_t timerPeriod)
{
    checkStandbyMode();
    uint8_t currentOpMode = readRegister(POWER_CTL);
    currentOpMode &= 0b11; // Get only the MODE bits
    if (currentOpMode != FULL_BANDWIDTH)
    {
#if DEBUG_
        Serial.println("WARNING: The activity timer is operational in measurement mode only");
#endif
    }
    writeRegister(TIME_ACT, timerPeriod);
}

void ADXL371class::setInactivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
{
    checkStandbyMode();
    uint8_t xThresh8Msb = formatThresholdValue(xThreshold);
    writeRegister(THRESH_INACT_X_H, xThresh8Msb);
    updateRegister(THRESH_INACT_X_L, (xThreshold << 5), THRESH_INACT_L_MASK);

    uint8_t yThresh8Msb = formatThresholdValue(yThreshold);
    writeRegister(THRESH_INACT_Y_H, yThresh8Msb);
    updateRegister(THRESH_INACT_Y_L, (yThreshold << 5), THRESH_INACT_L_MASK);

    uint8_t zThresh8Msb = formatThresholdValue(zThreshold);
    writeRegister(THRESH_INACT_Z_H, zThresh8Msb);
    updateRegister(THRESH_INACT_Z_L, (zThreshold << 5), THRESH_INACT_L_MASK);
}

void ADXL371class::enableInactivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    checkStandbyMode();
    updateRegister(THRESH_INACT_X_L, isEnabledX, INACT_EN_MASK);
    updateRegister(THRESH_INACT_Y_L, isEnabledY, INACT_EN_MASK);
    updateRegister(THRESH_INACT_Z_L, isEnabledZ, INACT_EN_MASK);
}

void ADXL371class::setReferencedInactivityProcessing(bool isReferenced)
{
    checkStandbyMode();
    updateRegister(THRESH_INACT_X_L, isReferenced << 1, INACT_REF_MASK);
}

void ADXL371class::setInactivityTimer(uint16_t timerPeriod)
{
    checkStandbyMode();
    uint8_t timerPeriodH = timerPeriod >> 8;
    uint8_t timerPeriodL = timerPeriod;

    writeRegister(TIME_INACT_H, timerPeriodH);
    writeRegister(TIME_INACT_L, timerPeriodL);
}

void ADXL371class::setMotionWarningThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
{
    uint8_t xThresh8Msb = formatThresholdValue(xThreshold);
    writeRegister(THRESH_ACT2_X_H, xThresh8Msb);
    updateRegister(THRESH_ACT2_X_L, (xThreshold << 5), THRESH_ACT2_L_MASK);

    uint8_t yThresh8Msb = formatThresholdValue(yThreshold);
    writeRegister(THRESH_ACT2_Y_H, yThresh8Msb);
    updateRegister(THRESH_ACT2_Y_L, (yThreshold << 5), THRESH_ACT2_L_MASK);

    uint8_t zThresh8Msb = formatThresholdValue(zThreshold);
    writeRegister(THRESH_ACT2_Z_H, zThresh8Msb);
    updateRegister(THRESH_ACT2_Z_L, (zThreshold << 5), THRESH_ACT2_L_MASK);
}
void ADXL371class::enableMotionWarningDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    updateRegister(THRESH_ACT2_X_L, isEnabledX, ACT_EN_MASK); // bit 1 in register
    updateRegister(THRESH_ACT2_Y_L, isEnabledY, ACT_EN_MASK);
    updateRegister(THRESH_ACT2_Z_L, isEnabledZ, ACT_EN_MASK);
}
void ADXL371class::setReferencedMotionWarningProcessing(bool isReferenced)
{
    updateRegister(THRESH_ACT2_X_L, isReferenced << 1, ACT2_REF_MASK);
}

void ADXL371class::setInstantOnThreshold(InstantOnThreshold threshold)
{
    byte valueShifted = threshold << 5; // bit 5 in register
    updateRegister(POWER_CTL, valueShifted, INSTANT_ON_THRESH_MASK);
}

void ADXL371class::enableExternalClock(bool isEnabled)
{
    byte valueShifted = isEnabled << 1; // bit 1 in register
    updateRegister(TIMING, valueShifted, EXT_CLK_MASK);
}

void ADXL371class::enableExternalTrigger(bool isEnabled)
{
    updateRegister(TIMING, isEnabled, EXT_SYNC_MASK);
}

int ADXL371class::convertOffsetValue(float offset) {
    if(offset < -60.0f || offset > 52.5f){
#if DEBUG_
        Serial.println("WARNING: Offset value can only be set to between -60 and 52.5. Try again");
#endif
        return 0;
    }

    int mappedOffset;

    if(offset >= 0){
        mappedOffset = static_cast<int>((offset / 52.5f) * 7);
    }
    else {
        mappedOffset = static_cast<int>(((offset +60 ) / 52.5f) * 7) + 8;
    }
    return mappedOffset;
}

void ADXL371class::setLinkLoopActivityProcessing(LinkLoop activityProcessing)
{
    checkStandbyMode();
    if (activityProcessing == LINKED || activityProcessing == LOOPED)
    {
        // Check if Activity and Inactivity detection is enabled
        if ((readRegister(THRESH_ACT_X_L) & ACT_EN_BIT) == false || (readRegister(THRESH_ACT_Y_L) & ACT_EN_BIT) == false || (readRegister(THRESH_ACT_Z_L) & ACT_EN_BIT) == false ||
            (readRegister(THRESH_INACT_X_L) & ACT_EN_BIT) == false || (readRegister(THRESH_INACT_Y_L) & ACT_EN_BIT) == false || (readRegister(THRESH_INACT_Z_L) & ACT_EN_BIT) == false)
        {
#if DEBUG_
            Serial.println("WARNING: Activity and Inactivity detection must be enabled");
#endif
        }
    }
    byte valueShifted = activityProcessing << 4; // bit 4 in register
    updateRegister(MEASURE, valueShifted, LINKLOOP_MASK);
}

void ADXL371class::enableAutosleep(bool isEnabled)
{
    byte valueShifted = isEnabled << 6; // bit 6 in register
    updateRegister(MEASURE, valueShifted, AUTOSLEEP_MASK);
}

void ADXL371class::setWakeUpRate(WakeUpRate wur)
{
    byte wurShifted = wur << 2; // wur bits start from bit 2
    updateRegister(TIMING, wurShifted, WAKEUP_RATE_MASK);
}