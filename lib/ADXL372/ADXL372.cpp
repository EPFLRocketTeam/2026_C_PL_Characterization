#include "ADXL372.h"

ADXL372class::ADXL372class(int csPinInput, SPIClass &spi)
{
    m_csPin = csPinInput;
    m_spi = &spi;
    m_spiSettings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0);
    m_sampleSize = 0;
    m_fifoAxisOrder = FifoAxisOrder::XYZ;
    m_referenceX = 0;
    m_referenceY = 0;
    m_referenceZ = 0;
}

ADXL372class::~ADXL372class()
{
}

void ADXL372class::begin()
{
    m_spi->begin();
    // m_spi->beginTransaction(SPISettings(SPI_SPEED)); // CPHA = CPOL = 0

    pinMode(m_csPin, OUTPUT);                                          // Setting chip select pin
    digitalWrite(m_csPin, HIGH);                                       // Pin ready
}

void ADXL372class::begin(uint32_t spiClockSpeed)
{
    m_spi->begin();
    
    m_spiSettings = SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0);

    pinMode(m_csPin, OUTPUT);                                              // Setting chip select pin
    digitalWrite(m_csPin, HIGH);                                           // Pin ready
}

bool ADXL372class::reset()
{
    setOperatingMode(STANDBY);
    delay(1);

    writeRegister(RESET, 0x52);
    delay(5);

    m_fifoAxisOrder = FifoAxisOrder::XYZ;

    return isConnected();
}

void ADXL372class::end()
{
    // set some adresses here

    m_spi->end();
}

void ADXL372class::printDevice()
{
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
}

byte ADXL372class::getPartId() {
    // Reads and returns the physical PARTID register (0xFA expected)
    return readRegister(PARTID);
}

uint8_t ADXL372class::getStatus()
{
    return readRegister(STATUS);
}

bool ADXL372class::isConnected() {
    uint8_t devidAd  = readRegister(DEVID_AD);
    uint8_t devidMst = readRegister(DEVID_MST);
    uint8_t partId   = readRegister(PARTID);

    return devidAd  == 0xAD &&
           devidMst == 0x1D &&
           partId   == DEVID_PRODUCT;
}

bool ADXL372class::getRawAcceleration(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z)
{
    raw_x = 0;
    raw_y = 0;
    raw_z = 0;


    const uint32_t start = micros();
    while ((readRegister(STATUS) & 0x01) == 0) {
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

void ADXL372class::readAcceleration(float &x, float &y, float &z)
{
    int16_t raw_x, raw_y, raw_z;
    getRawAcceleration(raw_x, raw_y, raw_z);

    // Converting buf axis data to acceleration in g unit
    x = raw_x * SCALE_FACTOR * MG_TO_G;
    y = raw_y * SCALE_FACTOR * MG_TO_G;
    z = raw_z * SCALE_FACTOR * MG_TO_G;
}

void ADXL372class::readPeakAcceleration(float &xPeak, float &yPeak, float &zPeak)
{
    const uint32_t start = micros();
    while ((readRegister(STATUS) & 0x01) == 0) {
        if (micros() - start >= TIMEOUT_US) {
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

void ADXL372class::setOffsetTrims(float xOffset, float yOffset, float zOffset)
{
    int xOffsetConverted = convertOffsetValue(xOffset);
    int yOffsetConverted = convertOffsetValue(yOffset);
    int zOffsetConverted = convertOffsetValue(zOffset);
    // No need to mask or bitshift for the offset registers
    writeRegister(OFFSET_X, xOffsetConverted);
    writeRegister(OFFSET_Y, yOffsetConverted);
    writeRegister(OFFSET_Z, zOffsetConverted);
}

uint8_t ADXL372class::formatThresholdValue(uint16_t thresholdValue)
{
    if (thresholdValue > 0x7FF) // The threshold value is an 11-bit value. So the max limit is 0x7FF.
    {
        Serial.println("WARNING: Threshold value limit is 2047, going beyond this value will have unintended effects.");
    }
    return thresholdValue = thresholdValue >> 3; // Get 8 MSB
}

void ADXL372class::setActivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
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

void ADXL372class::enableActivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    checkStandbyMode();
    updateRegister(THRESH_ACT_X_L, isEnabledX, ACT_EN_MASK); // bit 1 in register
    updateRegister(THRESH_ACT_Y_L, isEnabledY, ACT_EN_MASK);
    updateRegister(THRESH_ACT_Z_L, isEnabledZ, ACT_EN_MASK);
}

void ADXL372class::setReferencedActivityProcessing(bool isReferenced)
{
    checkStandbyMode();
    updateRegister(THRESH_ACT_X_L, isReferenced << 1, ACT_REF_MASK); // bit 1 in register
}

void ADXL372class::setActivityTimer(uint8_t timerPeriod)
{
    checkStandbyMode();
    uint8_t currentOpMode = readRegister(POWER_CTL);
    currentOpMode &= 0b11; // Get only the MODE bits
    if (currentOpMode != FULL_BANDWIDTH)
    {
        Serial.println("WARNING: The activity timer is operational in measurement mode only");
    }
    writeRegister(TIME_ACT, timerPeriod);
}

void ADXL372class::setInactivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
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

void ADXL372class::enableInactivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    checkStandbyMode();
    updateRegister(THRESH_INACT_X_L, isEnabledX, INACT_EN_MASK);
    updateRegister(THRESH_INACT_Y_L, isEnabledY, INACT_EN_MASK);
    updateRegister(THRESH_INACT_Z_L, isEnabledZ, INACT_EN_MASK);
}

void ADXL372class::setReferencedInactivityProcessing(bool isReferenced)
{
    checkStandbyMode();
    updateRegister(THRESH_INACT_X_L, isReferenced << 1, INACT_REF_MASK);
}

void ADXL372class::setInactivityTimer(uint16_t timerPeriod)
{
    checkStandbyMode();
    uint8_t timerPeriodH = timerPeriod >> 8;
    uint8_t timerPeriodL = timerPeriod;

    writeRegister(TIME_INACT_H, timerPeriodH);
    writeRegister(TIME_INACT_L, timerPeriodL);
}

void ADXL372class::setMotionWarningThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
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
void ADXL372class::enableMotionWarningDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    updateRegister(THRESH_ACT2_X_L, isEnabledX, ACT_EN_MASK); // bit 1 in register
    updateRegister(THRESH_ACT2_Y_L, isEnabledY, ACT_EN_MASK);
    updateRegister(THRESH_ACT2_Z_L, isEnabledZ, ACT_EN_MASK);
}
void ADXL372class::setReferencedMotionWarningProcessing(bool isReferenced)
{
    updateRegister(THRESH_ACT2_X_L, isReferenced << 1, ACT2_REF_MASK);
}

void ADXL372class::setFifoReference(int16_t x, int16_t y, int16_t z)
{
    m_referenceX = x;
    m_referenceY = y;
    m_referenceZ = z;
    m_fifoAxisOrder = FifoAxisOrder::UNKNOWN;
}

bool ADXL372class::detectFifoAxisOrder(const TRIPLET *samples, uint16_t sample_count)
{
    if (samples == nullptr || sample_count == 0) {
        return false;
    }

    int32_t sum_0 = 0;
    int32_t sum_1 = 0;
    int32_t sum_2 = 0;

    for (uint16_t i = 0; i < sample_count; i++) {
        sum_0 += samples[i].x;
        sum_1 += samples[i].y;
        sum_2 += samples[i].z;
    }

    float slot_0 = (float)sum_0 / sample_count;
    float slot_1 = (float)sum_1 / sample_count;
    float slot_2 = (float)sum_2 / sample_count;

    float dx, dy, dz;

    dx = slot_0 - m_referenceX;
    dy = slot_1 - m_referenceY;
    dz = slot_2 - m_referenceZ;
    float error_xyz = dx * dx + dy * dy + dz * dz;

    dx = slot_2 - m_referenceX;
    dy = slot_0 - m_referenceY;
    dz = slot_1 - m_referenceZ;
    float error_yzx = dx * dx + dy * dy + dz * dz;

    dx = slot_1 - m_referenceX;
    dy = slot_2 - m_referenceY;
    dz = slot_0 - m_referenceZ;
    float error_zxy = dx * dx + dy * dy + dz * dz;

    m_fifoAxisOrder = FifoAxisOrder::XYZ;

    if (error_yzx < error_xyz && error_yzx <= error_zxy) {
        m_fifoAxisOrder = FifoAxisOrder::YZX;
    }
    else if (error_zxy < error_xyz && error_zxy < error_yzx) {
        m_fifoAxisOrder = FifoAxisOrder::ZXY;
    }

    return true;
}

bool ADXL372class::isFifoAxisOrderDetected() const
{
    return m_fifoAxisOrder != FifoAxisOrder::UNKNOWN;
}

FifoAxisOrder ADXL372class::getFifoAxisOrder() const
{
    return m_fifoAxisOrder;
}

bool ADXL372class::readFifoData(TRIPLET *samples)
{
    if (samples == nullptr || m_sampleSize < 6) {
        return false;
    }

    uint16_t entries_to_read = m_sampleSize - 6;

    if ((entries_to_read % 3) != 0) {
        return false;
    }

    uint16_t bytes_to_read = entries_to_read * 2;

    if (bytes_to_read > 1024) {
        return false;
    }

    uint8_t buf[1024];

    select();
    m_spi->transfer((FIFO_DATA << 1) | 1);

    for (uint16_t i = 0; i < bytes_to_read; i++) {
        buf[i] = m_spi->transfer(0x00);
    }

    deselect();

    uint8_t x_slot = 0;
    uint8_t y_slot = 1;
    uint8_t z_slot = 2;

    if (m_fifoAxisOrder == FifoAxisOrder::YZX) {
        x_slot = 2;
        y_slot = 0;
        z_slot = 1;
    }
    else if (m_fifoAxisOrder == FifoAxisOrder::ZXY) {
        x_slot = 1;
        y_slot = 2;
        z_slot = 0;
    }

    uint16_t sample_count = entries_to_read / 3;
    uint8_t *buf_ptr = buf;

    for (uint16_t i = 0; i < sample_count; i++) {
        bool first_marker = (buf_ptr[1] & 0x01) != 0;
        bool second_marker = (buf_ptr[3] & 0x01) != 0;
        bool third_marker = (buf_ptr[5] & 0x01) != 0;

        if (!first_marker || second_marker || third_marker) {
            return false;
        }

        int16_t slot[3];

        slot[0] = (int16_t)((buf_ptr[0] << 8) | buf_ptr[1]) >> 4;
        slot[1] = (int16_t)((buf_ptr[2] << 8) | buf_ptr[3]) >> 4;
        slot[2] = (int16_t)((buf_ptr[4] << 8) | buf_ptr[5]) >> 4;

        samples[i].x = slot[x_slot];
        samples[i].y = slot[y_slot];
        samples[i].z = slot[z_slot];

        buf_ptr += 6;
    }

    return true;
}

void ADXL372class::setFifoSamples(int sampleSize)
{
    checkStandbyMode();
    if (sampleSize < 0) {
        sampleSize = 0;
    }

    if (sampleSize > 512)
    {
        Serial.println("WARNING: FIFO samples limit is 512");
        sampleSize = 512;
    }

    m_sampleSize = sampleSize;
    // sampleSize -= 1;

    writeRegister(
        FIFO_SAMPLES,
        static_cast<uint8_t>(sampleSize & 0xFF)
    );

    updateRegister(
        FIFO_CTL,
        static_cast<uint8_t>((sampleSize >> 8) & 0x01),
        FIFO_SAMPLES_8_MASK
    );
}

void ADXL372class::setFifoMode(FifoMode mode)
{
    checkStandbyMode();
    byte modeShifted = mode << 1; // starts from bit 1 in register
    updateRegister(FIFO_CTL, modeShifted, FIFO_MODE_MASK);
}

void ADXL372class::setFifoFormat(FifoFormat format)
{
    checkStandbyMode();
    byte formatShifted = format << 3; // starts from bit 3 in register
    updateRegister(FIFO_CTL, formatShifted, FIFO_FORMAT_MASK);
}

void ADXL372class::selectInt1Function(InterruptFunction function)
{
    writeRegister(INT1_MAP, function);
}

void ADXL372class::selectInt1Functions(uint8_t function)
{
    writeRegister(INT1_MAP, function);
}

void ADXL372class::selectInt2Function(InterruptFunction function)
{
    writeRegister(INT2_MAP, function);
}

void ADXL372class::selectInt2Functions(uint8_t function)
{
    writeRegister(INT2_MAP, function);
}

void ADXL372class::setOdr(Odr odr)
{
    int currentBandwidth = readRegister(MEASURE) & 0b00000111; // Get only the bandwidth bits
    if ((int)odr < currentBandwidth)
    {
        Serial.println("WARNING: ODR must be at least double the bandwidth, to not violate the Nyquist criteria. Otherwise signal integrity will not be maintained");
    }
    byte odrShifted = odr << 5; // odr bits start from bit 5
    updateRegister(TIMING, odrShifted, ODR_MASK);
}

void ADXL372class::setWakeUpRate(WakeUpRate wur)
{
    byte wurShifted = wur << 2; // wur bits start from bit 2
    updateRegister(TIMING, wurShifted, WAKEUP_RATE_MASK);
}

void ADXL372class::enableExternalClock(bool isEnabled)
{
    byte valueShifted = isEnabled << 1; // bit 1 in register
    updateRegister(TIMING, valueShifted, EXT_CLK_MASK);
}

void ADXL372class::enableExternalTrigger(bool isEnabled)
{
    updateRegister(TIMING, isEnabled, EXT_SYNC_MASK);
}

void ADXL372class::setBandwidth(Bandwidth bandwidth)
{
    int currentOdr = (readRegister(TIMING) & 0b11100000) >> 5; // Get only the ODR bits
    if ((int)bandwidth > currentOdr)
    {
        Serial.println("WARNING: Bandwidth must be no greater than half the ODR, to not violate the Nyquist criteria. Otherwise signal integrity will not be maintained");
    }
    updateRegister(MEASURE, bandwidth, BANDWIDTH_MASK);
}

void ADXL372class::enableLowNoiseOperation(bool isEnabled)
{
    byte valueShifted = isEnabled << 3; // bit 3 in register
    updateRegister(MEASURE, valueShifted, LOW_NOISE_MASK);
}

void ADXL372class::setLinkLoopActivityProcessing(LinkLoop activityProcessing)
{
    checkStandbyMode();
    if (activityProcessing == LINKED || activityProcessing == LOOPED)
    {
        // Check if Activity and Inactivity detection is enabled
        if ((readRegister(THRESH_ACT_X_L) & ACT_EN_BIT) == false || (readRegister(THRESH_ACT_Y_L) & ACT_EN_BIT) == false || (readRegister(THRESH_ACT_Z_L) & ACT_EN_BIT) == false ||
            (readRegister(THRESH_INACT_X_L) & ACT_EN_BIT) == false || (readRegister(THRESH_INACT_Y_L) & ACT_EN_BIT) == false || (readRegister(THRESH_INACT_Z_L) & ACT_EN_BIT) == false)
        {
            Serial.println("WARNING: Activity and Inactivity detection must be enabled");
        }
    }
    byte valueShifted = activityProcessing << 4; // bit 4 in register
    updateRegister(MEASURE, valueShifted, LINKLOOP_MASK);
}

void ADXL372class::enableAutosleep(bool isEnabled)
{
    byte valueShifted = isEnabled << 6; // bit 6 in register
    updateRegister(MEASURE, valueShifted, AUTOSLEEP_MASK);
}

void ADXL372class::setOperatingMode(OperatingMode opMode)
{
    updateRegister(POWER_CTL, opMode, MODE_MASK);
}

void ADXL372class::disableHighPassFilter(bool isDisabled)
{
    byte valueShifted = isDisabled << 2; // bit 2 in register
    updateRegister(POWER_CTL, valueShifted, HPF_DISABLE_MASK);
}

void ADXL372class::disableLowPassFilter(bool isDisabled)
{
    byte valueShifted = isDisabled << 3; // bit 3 in register
    updateRegister(POWER_CTL, valueShifted, LPF_DISABLE_MASK);
}

void ADXL372class::setFilterSettling(FilterSettlingPeriod filterSettling)
{
    byte valueShifted = filterSettling << 4; // bit 4 in register
    updateRegister(POWER_CTL, valueShifted, FILTER_SETTLE_MASK);
}

void ADXL372class::setInstantOnThreshold(InstantOnThreshold threshold)
{
    byte valueShifted = threshold << 5; // bit 5 in register
    updateRegister(POWER_CTL, valueShifted, INSTANT_ON_THRESH_MASK);
}

bool ADXL372class::checkStandbyMode()
{
    byte mode = readRegister(POWER_CTL);
    mode &= 0x03;

    if (mode != STANDBY)
    {
        Serial.println("WARNING: Activity, Inactivity and FIFO can only be set while in standy mode");
        return false;
    }
    return true;
}

int ADXL372class::convertOffsetValue(float offset) {
    if(offset < -60.0f || offset > 52.5f){
        Serial.println("WARNING: Offset value can only be set to between -60 and 52.5. Try again");
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

uint8_t ADXL372class::readRegister(byte regAddress)
{
    select();

    regAddress = regAddress << 1 | 1; // Reading from a register
    m_spi->transfer(regAddress);
    uint8_t value = m_spi->transfer(0x00); // Transfering dummy byte to recieve data from accelerometer

    deselect();
    return value;
}

void ADXL372class::readMultipleRegisters(byte regAddress, uint8_t *data, uint16_t count)
{
    select();

    regAddress = regAddress << 1 | 1; // Reading from a register
    m_spi->transfer(regAddress);

    for (uint16_t idx = 0; idx < count; idx++) {
        data[idx] = m_spi->transfer(0x00); // Transfering dummy byte to recieve data from accelerometer
    }

    deselect();
}

void ADXL372class::writeRegister(byte regAddress, uint8_t value)
{
    select();

    regAddress = regAddress << 1; // Writing to a register
    m_spi->transfer(regAddress);
    m_spi->transfer(value);
    
    deselect();
}

void ADXL372class::updateRegister(byte regAddress, uint8_t value, byte preserveMask)
{
    // Need to use bitmasks to only change the desired bits in the registers
    uint8_t current = readRegister(regAddress);
    current &= preserveMask;
    current |= value;
    writeRegister(regAddress, current);
}

bool ADXL372class::selfTest()
{
    // const uint32_t settling_ms = 1000 / currentODR * 4;
    // Self test procedure (Page 27 in datasheet)
    setOperatingMode(FULL_BANDWIDTH);
    setFilterSettling(FSP_370ms);
    updateRegister(SELF_TEST, true, ST_MASK);
    delay(400);

    if ((readRegister(SELF_TEST) & ~ST_DONE_MASK) == false)
    {
        Serial.println("Self Test was not finished");
        return false;
    }

    bool isTestPassed = readRegister(SELF_TEST) & ~USER_ST_MASK;

    updateRegister(SELF_TEST, false, ST_MASK); //Clear self test
    
    return isTestPassed;
}