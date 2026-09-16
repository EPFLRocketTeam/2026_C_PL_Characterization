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

    lsm_packet.header.sync_word = 0xAA;
    lsm_packet.header.payload_len = sizeof(lsm_packet.tmp) + sizeof(lsm_packet.data);

    if (accel->begin() != LSM6DSO32_OK) {
        #ifdef DEBUG_
        Serial.println("LSM6DSO32 initialization failed or sensor missing.");
        #endif
        return false;
    }

    #ifdef DEBUG_
    Serial.println("LSM6DSO32 initialized successfully."); 
    #endif
    
    // Setup LSM with highest sensibility & measure logging
    // Accel
    if (accel->Set_X_FS(32) != LSM6DSO32_OK) return false;
    if (accel->Set_X_ODR(6667.0) != LSM6DSO32_OK) return false;

    // Gyro
    if (accel->Set_G_FS(2000) != LSM6DSO32_OK) return false;
    if (accel->Set_G_ODR(6667.0) != LSM6DSO32_OK) return false;

    // FIFO config
    accel->Set_FIFO_X_BDR(6667.0);              // Logging rate
    accel->Set_FIFO_G_BDR(6667.0);
    accel->Set_FIFO_Watermark_Level(300);       // Watermark
    accel->Set_FIFO_Stop_On_Fth(1);             // Stop writting when watermark reached
    accel->Set_FIFO_Mode(LSM6DSO32_FIFO_MODE);  // Set FIFO mode 

    // Set int1 to trigger on watermark threshold 
    uint8_t int1_ctrl;
    accel->Read_Reg(0x0D, &int1_ctrl);  
    int1_ctrl |= (1 << 3);              // modify INT1_FIFO_TH
    accel->Write_Reg(0x0D, int1_ctrl);

    return true;
}

void start_lsm(LSM6DSO32Sensor *accel, uint8_t interrupt_pin, void (*isr)()) {
    if (accel == nullptr || isr == nullptr) return;

    // Start accelerometer & gyroscope
    accel->Enable_X();
    accel->Enable_G();

    // Attach interrupt pin
    pinMode(interrupt_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr, RISING);
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
    lsm_packet.header.sensor_type = sensor_id;  // Add sensor id
    lsm_packet.header.timestamp = timestamp;    // Add timestamp

    lsm_packet.tmp = getRawTmp(accel);          // Add raw temp measure

    uint16_t number_samples;
    accel->Get_FIFO_Num_Samples(&number_samples);  // Get unread measure number (300 expected)

    #ifdef DEBUG_
    Serial.println("FIFO samples number : ");
    Serial.print(number_samples);
    #endif

    uint8_t tag;        // sensor tag
    uint8_t data[6];    // FIFO payload buffer

    uint8_t gyr_idx = 0;  
    uint8_t acc_idx = 0;

    for(size_t i=0; i<number_samples; i++) {
        accel->Get_FIFO_Tag(&tag);      // Get last FIFO tag
        accel->Get_FIFO_Data(data);     // Get last FIFO data 
        
        // Save data into lsm_packet
        switch (tag) {
        case 1:
            if(gyr_idx < LSM_PACKET_SAMPLES) {
                lsm_packet.data[gyr_idx].Wx = (int16_t)((uint16_t)data[1] << 8 | data[0]);
                lsm_packet.data[gyr_idx].Wy = (int16_t)((uint16_t)data[3] << 8 | data[2]);
                lsm_packet.data[gyr_idx].Wz = (int16_t)((uint16_t)data[5] << 8 | data[4]);
                gyr_idx++;
            }
            break;
        
        case 2:
            if(acc_idx < LSM_PACKET_SAMPLES) {
                lsm_packet.data[acc_idx].Ax = (int16_t)((uint16_t)data[1] << 8 | data[0]);
                lsm_packet.data[acc_idx].Ay = (int16_t)((uint16_t)data[3] << 8 | data[2]);
                lsm_packet.data[acc_idx].Az = (int16_t)((uint16_t)data[5] << 8 | data[4]);
                acc_idx++;
            }
            break;
        }
    }

    #ifdef DEBUG_
    Serial.println("Acc samples : ");
    Serial.print(acc_idx);
    Serial.println("Gyr samples : ");
    Serial.print(gyr_idx);
    #endif

    ring_buffer_push((uint8_t*)&lsm_packet, sizeof(lsm_packet));
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