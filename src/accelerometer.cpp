#include "accelerometer.h"

volatile bool adxl_main_int = false;
volatile bool adxl_sat_int = false;

volatile uint32_t adxl_main_timestamp = 0;
volatile uint32_t adxl_sat_timestamp = 0;

volatile bool lsm_main_int = false;
volatile bool lsm_sat_int = false;

volatile uint32_t lsm_main_timestamp = 0;
volatile uint32_t lsm_sat_timestamp = 0;

ADXL371_PACKET adxl371_packet;
LSM_PACKET lsm_packet;

uint32_t adxl371_overruns[3] = {0, 0, 0};
uint32_t adxl371_invalid_blocks[3] = {0, 0, 0};

const char *fifo_order_name(FifoAxisOrder order) {
    if (order == FifoAxisOrder::YZX) return "YZX";
    if (order == FifoAxisOrder::ZXY) return "ZXY";
    return "XYZ";
}

int sensor_index(uint8_t sensor_id) {
    if (sensor_id < ID_ADXL371_MAIN || sensor_id > ID_ADXL371_SAT) {
        return -1;
    }

    return sensor_id - ID_ADXL371_MAIN;
}

//=================================================
// ISRs
//=================================================
FASTRUN void adxl_main_ISR() {
    adxl_main_timestamp = micros();
    adxl_main_int = true;
}

FASTRUN void adxl_sat_ISR() {
    adxl_sat_timestamp = micros();
    adxl_sat_int = true;
}

FASTRUN void lsm_main_ISR() {
    lsm_main_timestamp = micros();
    lsm_main_int = true;
}

FASTRUN void lsm_sat_ISR() {
    lsm_sat_timestamp = micros();
    lsm_sat_int = true;
}

//=================================================
// ADXL371
//=================================================
bool setup_adxl371(ADXL371class *accel) {
    if (accel == nullptr) return false;

    // Set ADXL packet constants
    adxl371_packet.header.sync_word = 0xAA;
    adxl371_packet.header.payload_len = sizeof(adxl371_packet.data);

    accel->begin();
    
    if (!accel->isConnected()) {
        #ifdef DEBUG_
        accel->printDevice();
        Serial.println("ADXL371 is not connected");
        #endif
        return false; // Sensor is missing or SPI is dead
    }
    
    if (!accel->reset()) {
        #ifdef DEBUG_
        accel->printDevice();
        Serial.println("ADXL371 reset or post-reset ID check failed.");
        #endif
        return false;
    }

    // Silicon anomaly makes this unsafe
    // if (!accel->selfTest()) {
    //     #ifdef DEBUG_
    //     Serial.println("Self Test Accel Error.");
    //     accel->printDevice();
    //     #endif
    //     return false;
    // }

    #ifdef DEBUG_
    accel->printDevice();
    #endif

    accel->setOperatingMode(STANDBY);

    accel->setOdr(ODR_5120Hz);                  //output data rate
    accel->setBandwidth(BW_1280Hz);             //low-pass filter cutoff frequency
    accel->enableLowNoiseOperation(true);

    accel->disableHighPassFilter(true);         //keep low acceleration in signal
    accel->disableLowPassFilter(false);         //filter to higher frequency (Nyquist-Shanon) 
    accel->setFilterSettling(FSP_4_OVER_ODR);   //short settling time

    accel->setFifoSamples(300);                 //set FIFO watermarks
    accel->setFifoFormat(XYZ);                  //set data format
    accel->setFifoMode(STREAM);                 //continious writing

    accel->selectInt1Function(FIFO_FULL);       //int1 setup (when watermarks reached)
    accel->setOperatingMode(FULL_BANDWIDTH);     

    delay(2*4/5120); // Allow filter settling period to complete before collecting direct-register and FIFO means.

    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    for (uint8_t i = 0; i < 64; i++) {
        int16_t x, y, z;

        if (!accel->getRawAcceleration(x, y, z)) {
            return false;
        }

        sum_x += x;
        sum_y += y;
        sum_z += z;
    }

    accel->setFifoReference(sum_x / 64, sum_y / 64, sum_z / 64);

    // Leave the final FIFO configuration ready, but do not start it yet.
    accel->setOperatingMode(STANDBY);
    accel->setFifoMode(STREAM);

    adxl371_packet.header.sync_word = 0xAA;
    adxl371_packet.header.payload_len = sizeof(adxl371_packet.data);

    return true;
}

void start_adxl371(ADXL371class *accel, uint8_t interrupt_pin, void (*isr)()) {
    pinMode(interrupt_pin, INPUT);

    // Clear any old status before enabling the interrupt.
    accel->getStatus();

    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr, RISING);
    accel->setOperatingMode(FULL_BANDWIDTH);
}


void print_adxl371_accel(ADXL371class *accel) {
    /*
    * Reads latest measurement and prints to Serial
    */
    float x, y, z;
    accel->readAcceleration(x, y, z);

    #ifdef DEBUG_
    char line[50];
    sprintf(line, "x: %f, y: %f, z: %f", x, y, z);
    Serial.println(line);
    #endif
}

void log_adxl371_fifo(ADXL371class *accel, uint32_t timestamp, uint8_t sensor_id) {
    int index = sensor_index(sensor_id);

    uint8_t status_before = accel->getStatus();
    bool valid = accel->readFifoData(adxl371_packet.data);
    uint8_t status_after = accel->getStatus();

    if (index >= 0 && ((status_before | status_after) & FIFO_OVR)) {
        adxl371_overruns[index]++;
    }

    if (!valid) {
        if (index >= 0) {
            adxl371_invalid_blocks[index]++;
        }
        return;
    }

    // The first FIFO packet is used only to determine the axis order.
    if (!accel->isFifoAxisOrderDetected()) {
        accel->detectFifoAxisOrder(
            adxl371_packet.data,
            ADXL371_PACKET_SAMPLES
        );

        #ifdef DEBUG_
        Serial.print("ADXL371 FIFO order detected: ");
        Serial.println(fifo_order_name(accel->getFifoAxisOrder()));
        #endif

        return;
    }

    adxl371_packet.header.sensor_type = sensor_id;
    adxl371_packet.header.timestamp = timestamp;
    
    ring_buffer_push((uint8_t*)&adxl371_packet, sizeof(adxl371_packet));
}

void print_adxl371_diagnostics() {
    #ifdef DEBUG_
    const char *names[3] = {"Main", "Sat"};

    for (uint8_t i = 0; i < 2; i++) {
        Serial.print("ADXL371 ");
        Serial.print(names[i]);
        Serial.print(": overruns=");
        Serial.print(adxl371_overruns[i]);
        Serial.print(", invalid blocks=");
        Serial.println(adxl371_invalid_blocks[i]);
    }
    #endif
}

//=================================================
// LSM6DOS32
//=================================================
bool setup_lsm(LSM6DSO32Sensor *accel) {
    if (accel == nullptr) return false;

    // Set LSM packet constant
    lsm_packet.header.sync_word = 0xAA;
    lsm_packet.header.payload_len = sizeof(lsm_packet.data) + sizeof(lsm_packet.tmp);

    // Initialize interface
    if (accel->begin() != LSM6DSO32_OK) {
        #ifdef DEBUG_
        Serial.println("LSM6DSO32 begin() failed.");
        #endif
        return false;
    }
    // Explicitly check physical sensor presence
    uint8_t id = 0;
    if (accel->ReadID(&id) != LSM6DSO32_OK) {
        #ifdef DEBUG_
        Serial.println("WHO_AM_I read ERROR");
        #endif
        return false;
    }
    Serial.print("WHO_AM_I = 0x");
    Serial.println(id, HEX);

    if (id != 0x6C) {
        #ifdef DEBUG_
        Serial.println("LSM6DSO32 NOT DETECTED");
        #endif
        return false;
    }
    #ifdef DEBUG_
    Serial.println("LSM6DSO32 initialized successfully."); 
    #endif

    // Sensor reset, erase previous config
    accel->Write_Reg(0x12, 0x01); 
    delay(15); 

    // Setup LSM with highest sensibility & measure logging
    // Accel ODR=6667Hz 1010b, FS=+/-32g 01b LFP2_EN 0b + 0b -> 0xA4
    if (accel->Write_Reg(0x10, 0xA4) != LSM6DSO32_OK) return false;
    uint8_t ctr10=0;
    accel->Read_Reg(0x10, &ctr10);
    Serial.print("0x10 = 0b");
    Serial.println(ctr10, BIN);
    // Gyro ODR=6667Hz 1010b FS=+/-2000dps 11b LFP2 0b + 0b -> 0xAC
    if (accel->Write_Reg(0x11, 0xAC) != LSM6DSO32_OK) return false;

    // Disable I2C interface
    uint8_t ctrl4 = 0;
    if (accel->Read_Reg(0x13, &ctrl4) == LSM6DSO32_OK) {
        ctrl4 |= (1 << 2); 
        accel->Write_Reg(0x13, ctrl4);
    }
    // Disable I3C interface
    uint8_t val = 0;
    accel->Read_Reg(0x18, &val);
    val |= (1 << 1); 
    accel->Write_Reg(0x18, val);

    // FIFO config
    accel->Write_Reg(0x12, 0x44);   // Enable multiregister access
    // Bypass mode for configuration 
    if (accel->Write_Reg(0x0A, 0x00) != LSM6DSO32_OK) return false;     
    // Set watermark à 300
    if (accel->Write_Reg(0x07, 0x2C) != LSM6DSO32_OK
     || accel->Write_Reg(0x08, 0x81) != LSM6DSO32_OK) {                 
        #ifdef DEBUG_
        Serial.println("Set_FIFO_Watermark_Level FAILED");
        #endif
        return false;
    }
    // Set X/G FIFO BDR
    if (accel->Write_Reg(0x09, 0xAA) != LSM6DSO32_OK) {
        #ifdef DEBUG_
        Serial.println("Set_FIFO_X_BDR FAILED");
        Serial.println("Set_FIFO_G_BDR FAILED");
        #endif
        return false;
    }

    // Set int1 to trigger on watermark threshold 
    uint8_t int1_ctrl = 0;
    if (accel->Read_Reg(0x0D, &int1_ctrl) != LSM6DSO32_OK) {
        #ifdef DEBUG_
        Serial.println("INT1_CTRL read ERROR");
        #endif
        return false;
    }
    int1_ctrl |= (1 << 3);
    if (accel->Write_Reg(0x0D, int1_ctrl) != LSM6DSO32_OK) {
        #ifdef DEBUG_
        Serial.println("INT1_CTRL write ERROR");
        #endif
        return false;
    }

    // Check int1 mapping
    #ifdef DEBUG_
    uint8_t check = 0;
    if (accel->Read_Reg(0x0D, &check) != LSM6DSO32_OK) {
        Serial.println("INT1_CTRL readback ERROR");
        return false;
    }
    Serial.print("INT1_CTRL after = 0x");
    Serial.println(check, HEX);
    #endif

    return true;
}

void start_lsm(LSM6DSO32Sensor *accel, uint8_t interrupt_pin, void (*isr)()) {
    if (accel == nullptr || isr == nullptr) return;

    // Attach interrupt pin
    pinMode(interrupt_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr, RISING);

    // Start data acquisition in FIFO
    if (accel->Write_Reg(0x0A, 0x01) != LSM6DSO32_OK) {
        #ifdef DEBUG_
        Serial.println("FIFO MODE fail !");
        #endif
        return;
    }

    #ifdef DEBUG_
    Serial.println("LSM6DSO32 start !");
    #endif
}

int16_t getRawTmp(LSM6DSO32Sensor *accel) {
    uint8_t temp[2];

    if (accel->IO_Read(temp, 0x20, 2) != 0)
        return 0;

    return (int16_t)(((uint16_t)temp[1] << 8) | temp[0]);
}

void print_lsm_accel(LSM6DSO32Sensor *accel) {
    int32_t data[3];

    if (accel->Get_X_Axes(data) != LSM6DSO32_OK) {
        Serial.println("LSM6DSO32 acceleration read error");
        return;
    }

    #ifdef DEBUG_
    char line[64];

    sprintf(line, "x: %f, y: %f, z: %f", 
            data[0], 
            data[1], 
            data[2]);
    Serial.println(line);
    #endif
}

void print_lsm_gyro(LSM6DSO32Sensor *accel) {
    int32_t data[3];

    if (accel->Get_G_Axes(data) != LSM6DSO32_OK){
        Serial.println("LSM6DSO32 gyroscope read error");
        return;
    }

    #ifdef DEBUG_
    char line[64];
    // Note: Adafruit_LSM6DS returns values in m/s^2
    sprintf(line, "Wx: %f, Wy: %f, Wz: %f", 
            data[0], 
            data[1], 
            data[2]);
    Serial.println(line);
    #endif
}

void print_lsm_temperature(LSM6DSO32Sensor *accel) {

    #ifdef DEBUG_
    int16_t raw = getRawTmp(accel);
    float temperature = 25.0f + ((float)raw / 256.0f);

    Serial.print("LSM Temperature [C] : ");
    Serial.println(temperature);
    #endif
}

void log_lsm_fifo(LSM6DSO32Sensor *accel, uint32_t timestamp, uint8_t sensor_id) {
    if (accel == nullptr) return;
    lsm_packet.header.sensor_type = sensor_id;     // Add sensor id
    lsm_packet.header.timestamp = timestamp;       // Add timestamp

    lsm_packet.tmp = getRawTmp(accel);             // Add raw temp measure

    uint16_t number_samples;
    accel->Get_FIFO_Num_Samples(&number_samples);  // Get unread measure number (300 expected)

    uint8_t fifo_raw[7];                           // Buffer for fifo word
    uint8_t gyr_idx = 0;  
    uint8_t acc_idx = 0;

    for(size_t i = 0; i < number_samples; i++) {
        // Continious reading of 7 register starting from tag
        if (accel->IO_Read(fifo_raw, 0x78, 7) != LSM6DSO32_OK) {
            break; 
        }

        // Read tag bits
        uint8_t tag = (fifo_raw[0] >> 3) & 0x1F;
        
        if (tag == 1 && gyr_idx < LSM_PACKET_SAMPLES) {
            // Gyro
            lsm_packet.data[gyr_idx].Wx = (int16_t)((uint16_t)fifo_raw[2] << 8 | fifo_raw[1]);
            lsm_packet.data[gyr_idx].Wy = (int16_t)((uint16_t)fifo_raw[4] << 8 | fifo_raw[3]);
            lsm_packet.data[gyr_idx].Wz = (int16_t)((uint16_t)fifo_raw[6] << 8 | fifo_raw[5]);
            gyr_idx++;
        } 
        else if (tag == 2 && acc_idx < LSM_PACKET_SAMPLES) {
            // Accel
            lsm_packet.data[acc_idx].Ax = (int16_t)((uint16_t)fifo_raw[2] << 8 | fifo_raw[1]);
            lsm_packet.data[acc_idx].Ay = (int16_t)((uint16_t)fifo_raw[4] << 8 | fifo_raw[3]);
            lsm_packet.data[acc_idx].Az = (int16_t)((uint16_t)fifo_raw[6] << 8 | fifo_raw[5]);
            acc_idx++;
        }
    }

    ring_buffer_push((uint8_t*)&lsm_packet, sizeof(lsm_packet));

    accel->Write_Reg(0x0A, 0x00);   // Flush fifo content
    accel->Write_Reg(0x0A, 0x01);   // Restart data logging
}

void print_lsm_diagnostics(LSM6DSO32Sensor *accel) {

    uint8_t id;
    float accel_odr;
    float gyro_odr;
    int32_t accel_fs;
    int32_t gyro_fs;
    uint16_t fifo_samples;
    uint8_t fifo_full;

    #ifdef DEBUG_
    Serial.println("----- LSM6DSO32 diagnostics -----");

    if (accel->ReadID(&id) == LSM6DSO32_OK) {
        Serial.print("WHO_AM_I : 0x");
        Serial.println(id, HEX);
    }

    if (accel->Get_X_ODR(&accel_odr) == LSM6DSO32_OK) {
        Serial.print("Accel ODR : ");
        Serial.print(accel_odr);
        Serial.println(" Hz");
    }

    if (accel->Get_G_ODR(&gyro_odr) == LSM6DSO32_OK) {
        Serial.print("Gyro ODR : ");
        Serial.print(gyro_odr);
        Serial.println(" Hz");
    }

    if (accel->Get_X_FS(&accel_fs) == LSM6DSO32_OK) {
        Serial.print("Accel FS : ±");
        Serial.print(accel_fs);
        Serial.println(" g");
    }

    if (accel->Get_G_FS(&gyro_fs) == LSM6DSO32_OK) {
        Serial.print("Gyro FS : ±");
        Serial.print(gyro_fs);
        Serial.println(" dps");
    }

    if (accel->Get_FIFO_Num_Samples(&fifo_samples) == LSM6DSO32_OK) {
        Serial.print("FIFO samples : ");
        Serial.println(fifo_samples);
    }

    if (accel->Get_FIFO_Full_Status(&fifo_full) == LSM6DSO32_OK) {
        Serial.print("FIFO full : ");
        Serial.println(fifo_full);
    }

    Serial.println("---------------------------------");
    #endif
}