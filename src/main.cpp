#include "main.h"

#include "buzzer.h"
#include "packets.h"
#include "sd_card.h"
#include "microphone.h"
#include "environment.h"
#include "accelerometer.h"

ADXL371class adxl_main(CS_PIN_ADXL_Main, SPI);		// Constructor different for each lib...
Adafruit_BME280 bme_main(CS_PIN_BME_Main, &SPI);
LSM6DSO32Sensor lsm_main(&SPI, CS_PIN_LSM_Main, 10000000);

ADXL371class adxl_sat(CS_PIN_ADXL_Sat, SPI1);
Adafruit_BME280 bme_sat(CS_PIN_BME_Sat, &SPI1);
LSM6DSO32Sensor lsm_sat(&SPI1, CS_PIN_LSM_Sat, 10000000);

uint32_t last_bme_read = 0;

bool is_logging = true;

// Sensors connection flags
bool has_adxl_main = false;		
bool has_lsm_main  = false;
bool has_bme_main  = false;

bool has_adxl_sat  = false;
bool has_lsm_sat   = false;
bool has_bme_sat   = false;

volatile bool pfm_triggered = false;	// Power fail flag

uint32_t jumper_high_start = 0;
bool jumper_bouncing = false;

FASTRUN void power_fail_ISR() {
	pfm_triggered = true;
}

void test_miso1_idle_drive()
{
    // Every device on SPI1 must be deselected.
	pinMode(CS_PIN_ADXL_Sat, OUTPUT);
    digitalWriteFast(CS_PIN_ADXL_Sat, HIGH);

	pinMode(CS_PIN_LSM_Sat, OUTPUT);
    digitalWriteFast(CS_PIN_LSM_Sat, HIGH);

	pinMode(CS_PIN_BME_Sat, OUTPUT);
    digitalWriteFast(CS_PIN_BME_Sat, HIGH);

    delay(10);

    pinMode(SPI1_MISO, INPUT_PULLDOWN);
    delay(20);
    const int with_pulldown = digitalReadFast(SPI1_MISO);

    pinMode(SPI1_MISO, INPUT_PULLUP);
    delay(20);
    const int with_pullup = digitalReadFast(SPI1_MISO);

    pinMode(SPI1_MISO, INPUT);

    Serial.printf(
        "MISO1: pulldown=%d, pullup=%d\n",
        with_pulldown,
        with_pullup
    );
}

void setup() {
	SPI.begin();
	SPI1.begin();
	#ifdef DEBUG_
	Serial.begin(115200);
	while (!Serial) {}
	Serial.println("----- Program Start -----");
	#endif

	delay(100);

	// Setup buzzer
	setup_buzzer();

	// Setup SPI CS pins (deselect all sensors)
	digitalWrite(CS_PIN_ADXL_Main, HIGH); pinMode(CS_PIN_ADXL_Main, OUTPUT);	// SPI (main)
	digitalWrite(CS_PIN_LSM_Main, HIGH); pinMode(CS_PIN_LSM_Main, OUTPUT);
	digitalWrite(CS_PIN_BME_Main, HIGH); pinMode(CS_PIN_BME_Main, OUTPUT); 

    digitalWrite(CS_PIN_ADXL_Sat, HIGH); pinMode(CS_PIN_ADXL_Sat, OUTPUT);		// SPI1 (sat)
    digitalWrite(CS_PIN_LSM_Sat, HIGH); pinMode(CS_PIN_LSM_Sat, OUTPUT);
    digitalWrite(CS_PIN_BME_Sat, HIGH); pinMode(CS_PIN_BME_Sat, OUTPUT); 
	
	delay(5);

	// Activate all sensors SPI mode
	digitalWrite(CS_PIN_ADXL_Main, LOW); delay(1); digitalWrite(CS_PIN_ADXL_Main, HIGH);	// SPI (main)
    digitalWrite(CS_PIN_LSM_Main, LOW);  delay(1); digitalWrite(CS_PIN_LSM_Main, HIGH);
    digitalWrite(CS_PIN_BME_Main, LOW);  delay(1); digitalWrite(CS_PIN_BME_Main, HIGH);
    
    digitalWrite(CS_PIN_ADXL_Sat, LOW);  delay(1); digitalWrite(CS_PIN_ADXL_Sat, HIGH);		// SPI1 (sat)
    digitalWrite(CS_PIN_LSM_Sat, LOW);   delay(1); digitalWrite(CS_PIN_LSM_Sat, HIGH);
    digitalWrite(CS_PIN_BME_Sat, LOW);   delay(1); digitalWrite(CS_PIN_BME_Sat, HIGH);
    
    delay(5);

	// Initialization start beep
	beep(1000);

	// Setup main sensors
	#ifdef DEBUG_
	Serial.println();
	Serial.println("Connecting ADXL371 Main");
	#endif
	has_adxl_main = setup_adxl371(&adxl_main);
	conditional_beeps(has_adxl_main, 100, 2, 200, 1);
	delay(5);

	#ifdef DEBUG_
	Serial.println();
	Serial.println("Connecting BME280 Main");
	#endif
	has_bme_main = setup_bme(&bme_main);
	conditional_beeps(has_bme_main, 100, 2, 200, 1);
	delay(5);

	#ifdef DEBUG_
	Serial.println();
	Serial.println("Connecting LSM6DSO32 Main");
	#endif
	has_lsm_main = setup_lsm(&lsm_main);
	conditional_beeps(has_lsm_main, 100, 2, 200, 1);
	delay(5);

	// Setup main sensors
	#ifdef DEBUG_
	Serial.println();
	Serial.println("Connecting ADXL371 Sat");
	#endif
	has_adxl_sat = setup_adxl371(&adxl_sat);
	conditional_beeps(has_adxl_sat, 100, 2, 200, 1);
	delay(5);

	#ifdef DEBUG_
	Serial.println();
	Serial.println("Connecting BME280 Sat");
	#endif
	has_bme_sat = setup_bme(&bme_sat);
	conditional_beeps(has_bme_sat, 100, 2, 200, 1);
	delay(5);

	#ifdef DEBUG_
	Serial.println();
	Serial.println("Connecting LSM6DSO32 Sat");
	#endif
	has_lsm_sat = setup_lsm(&lsm_sat);
	conditional_beeps(has_lsm_sat, 100, 2, 200, 1);
	delay(5);

	#ifdef DEBUG_
	Serial.println();
	if (has_adxl_main) Serial.println("ADXL371 Main Connected");
	if (has_bme_main) Serial.println("BME280 Main Connected");
	if (has_lsm_main) Serial.println("LSM6DSO32 Main Connected");
	
	if (has_adxl_sat) Serial.println("ADXL371 Sat Connected");
	if (has_bme_sat) Serial.println("BME280 Sat Connected");
	if (has_lsm_sat) Serial.println("LSM6DSO32 Sat Connected");
	Serial.println();
	delay(200);
	#endif

	// Setup microphones
	setup_microphones();

	if (!setup_sd() || !setup_file()) {
		#ifdef DEBUG_
		Serial.println("CRITICAL SD ERROR: Halting system.");
		#endif

		beep(100);
        while (1) {}
    }

	// Setup Power Failure Monitor
	pinMode(PFM_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(PFM_PIN), power_fail_ISR, FALLING);

	// Setup User Jumper
	pinMode(JUMPER_PIN, INPUT);
	while(digitalRead(JUMPER_PIN) == HIGH) {}	// Wait for jumper removing to start measure

	#ifdef DEBUG_
	Serial.println("----- Measure Start -----");
	#endif
	
	// Setup success beeps
	beeps(100, 3);
	delay(5);

	// Keep this start order !!! ADXL first makes lsm crash !!!
	if (has_lsm_main) start_lsm(&lsm_main, INT_PIN_LSM_Main, lsm_main_ISR);
	delay(1);
    if (has_lsm_sat) start_lsm(&lsm_sat, INT_PIN_LSM_Sat, lsm_sat_ISR);
	delay(1);
	if (has_adxl_main) start_adxl371(&adxl_main, INT_PIN_ADXL_Main, adxl_main_ISR); 
	delay(1);
    if (has_adxl_sat) start_adxl371(&adxl_sat, INT_PIN_ADXL_Sat, adxl_sat_ISR);
	delay(1);
}

void loop() {

	// Case of unexpected power shutdown (PFM trigger)
	if (pfm_triggered && is_logging) {
        uint32_t poll_start = millis();
        is_logging = false;

		#ifdef DEBUG_
        Serial.println("[CRITICAL] Power Failure Detected! Executing Panic Save.");
		#endif

        // Drain the absolute last bytes of the ring buffer
        write_buffer_to_sd();

		#ifdef DEBUG_
		Serial.print("Buffer saved in ");Serial.print(millis() - poll_start);Serial.println("ms");
		#endif

        // Inject EOF and safely close the file system
        close_sd(PFM_END);
		#ifdef DEBUG_
        Serial.print("SD File Secured in ");Serial.print(millis() - poll_start);Serial.println("ms");
		Serial.println("Entering monitoring phase...");
		#endif

        // "Lazarus" Polling Phase
        bool power_restored = false;
        while (millis() - poll_start < PFM_MONITOR_TIME) {
            // Check if the PFM pin goes back to 2.5V (HIGH)
            if (digitalRead(PFM_PIN) == HIGH) {
                delay(10); // Final software debounce
                if (digitalRead(PFM_PIN) == HIGH) {
                    power_restored = true;
                    break;
                }
            }
        }

		// Sound long buzzer beep to confirm save
		beep(2000);

        // Post-Monitoring Action
        if (power_restored) {
			#ifdef DEBUG_
            Serial.println("Power restored! Executing Cortex-M7 hardware reset...");
            delay(100); // Allow serial buffer to flush
			#endif
            
            // Triggers a complete system reset on the Teensy
            SCB_AIRCR = 0x05FA0004; 
        } else {
			#ifdef DEBUG_
            Serial.println("System dead. Awaiting supercapacitor depletion.");
			#endif

            while (1) {
                // Safe state. Processor will brown out peacefully.
            }
        }
    }

	// Normal operation (continuous logging)
	if (is_logging) {
		// Check Accelerometer ISR Flags
		if (has_adxl_main && adxl_main_int) {
			log_adxl371_fifo(&adxl_main, adxl_main_timestamp, ID_ADXL371_MAIN);
			adxl_main_int = false;
		}
		if (has_adxl_sat && adxl_sat_int) {
			log_adxl371_fifo(&adxl_sat, adxl_sat_timestamp, ID_ADXL371_SAT);
			adxl_sat_int = false;
		}

		if (has_lsm_main && lsm_main_int) {
			log_lsm_fifo(&lsm_main, lsm_main_timestamp, ID_LSM_MAIN);
			lsm_main_int = false;
		}
		if (has_lsm_sat && lsm_sat_int) {
			log_lsm_fifo(&lsm_sat, lsm_sat_timestamp, ID_LSM_SAT);
			lsm_sat_int = false;
		}

		// Read Microphones
		read_microphone();

		// Read BME280s at 10 Hz (every 100ms) without blocking
		if (millis() - last_bme_read >= 100) {
			if (has_bme_main) log_bme_values(&bme_main, ID_BME280_MAIN);
			if (has_bme_sat) log_bme_values(&bme_sat, ID_BME280_SAT);
			last_bme_read = millis();
		}

		// Drain Ring Buffer to SD Card
		write_buffer_to_sd();
	}

	// Jumper debounce
	bool jumper_placed = false;
    if (digitalRead(JUMPER_PIN) == HIGH) {
        if (!jumper_bouncing) {
            // First time we see it HIGH, start the stopwatch
            jumper_bouncing = true;
            jumper_high_start = millis();
        } else if (millis() - jumper_high_start >= JUMPER_DEBOUNCE_TIME) {
            jumper_placed = true;
        }
    } else {
        // Pin went LOW. It was glitch or not pulled yet. Reset stopwatch.
        jumper_bouncing = false;
    }

	// Stop Logging (User provoked)
	if (jumper_placed || reached_file_end()) {
		is_logging = false;

		print_adxl371_diagnostics();

		#ifdef DEBUG_
		Serial.println("Safe shutdown triggered.");
		#endif

		write_buffer_to_sd();
		close_sd(JUMPER_END);

		#ifdef DEBUG_
		Serial.println("SD File Truncated and Closed. Safe to power off.");
		#endif

		// Sound long buzzer beep to confirm save
		beep(2000);

		while(1) {
			// System is safe to power down
		}
    }

}

// #include <Arduino.h>
// #include <SPI.h>
// #include <main.h>

// constexpr uint8_t CS_MAIN =
//     CS_PIN_Accel_Main;

// constexpr uint8_t CS_SAT2 =
//     CS_PIN_Accel_Sat_2;

// uint8_t readMainRegister(
//     uint8_t address,
//     uint32_t spi_frequency)
// {
//     SPI1.beginTransaction(
//         SPISettings(
//             spi_frequency,
//             MSBFIRST,
//             SPI_MODE0
//         )
//     );

//     digitalWriteFast(CS_MAIN, LOW);

//     // Conservative CS-to-clock delay for the test.
//     delayMicroseconds(1);

//     SPI1.transfer(
//         static_cast<uint8_t>(
//             (address << 1) | 0x01U
//         )
//     );

//     const uint8_t value =
//         SPI1.transfer(0x00);

//     digitalWriteFast(CS_MAIN, HIGH);

//     SPI1.endTransaction();

//     return value;
// }

// struct TestResults {
//     uint32_t total_register_reads = 0;
//     uint32_t bad_register_reads = 0;

//     uint32_t zero_to_one_bits = 0;
//     uint32_t one_to_zero_bits = 0;

//     uint8_t last_expected = 0;
//     uint8_t last_actual = 0;
//     uint8_t last_register = 0;
// };

// TestResults results;

// void recordResult(
//     uint8_t address,
//     uint8_t expected,
//     uint8_t actual)
// {
//     results.total_register_reads++;

//     if (actual == expected) {
//         return;
//     }

//     results.bad_register_reads++;

//     results.last_register = address;
//     results.last_expected = expected;
//     results.last_actual = actual;

//     const uint8_t changed =
//         static_cast<uint8_t>(
//             expected ^ actual
//         );

//     const uint8_t zero_to_one =
//         static_cast<uint8_t>(
//             changed & actual
//         );

//     const uint8_t one_to_zero =
//         static_cast<uint8_t>(
//             changed & expected
//         );

//     results.zero_to_one_bits +=
//         __builtin_popcount(
//             static_cast<unsigned int>(
//                 zero_to_one
//             )
//         );

//     results.one_to_zero_bits +=
//         __builtin_popcount(
//             static_cast<unsigned int>(
//                 one_to_zero
//             )
//         );
// }

// void runIdTest(
//     uint32_t spi_frequency,
//     uint32_t repetitions)
// {
//     static constexpr uint8_t addresses[] = {
//         0x00,
//         0x01,
//         0x02,
//         0x03
//     };

//     static constexpr uint8_t expected[] = {
//         0xAD,
//         0x1D,
//         0xFA,
//         0x03
//     };

//     results = {};

//     for (uint32_t test = 0;
//          test < repetitions;
//          test++)
//     {
//         for (uint8_t i = 0; i < 4; i++) {
//             const uint8_t actual =
//                 readMainRegister(
//                     addresses[i],
//                     spi_frequency
//                 );

//             recordResult(
//                 addresses[i],
//                 expected[i],
//                 actual
//             );
//         }
//     }

//     Serial.printf(
//         "\nSPI frequency: %lu Hz\n",
//         spi_frequency
//     );

//     Serial.printf(
//         "Register reads: %lu\n",
//         results.total_register_reads
//     );

//     Serial.printf(
//         "Bad reads: %lu\n",
//         results.bad_register_reads
//     );

//     Serial.printf(
//         "0 -> 1 errors: %lu\n",
//         results.zero_to_one_bits
//     );

//     Serial.printf(
//         "1 -> 0 errors: %lu\n",
//         results.one_to_zero_bits
//     );

//     Serial.printf(
//         "Last error: reg=0x%02X expected=0x%02X actual=0x%02X\n",
//         results.last_register,
//         results.last_expected,
//         results.last_actual
//     );
// }

// void setup()
// {
//     Serial.begin(115200);

//     while (!Serial && millis() < 3000) {
//     }

//     pinMode(CS_MAIN, OUTPUT);
//     pinMode(CS_SAT2, OUTPUT);

//     digitalWriteFast(CS_MAIN, HIGH);
//     digitalWriteFast(CS_SAT2, HIGH);

//     SPI1.begin();

//     Serial.printf(
//         "CS Main=%d, CS Sat2=%d\n",
//         digitalReadFast(CS_MAIN),
//         digitalReadFast(CS_SAT2)
//     );

//     runIdTest(100000, 10000);
//     runIdTest(1000000, 10000);
//     runIdTest(10000000, 10000);
// }

// void loop()
// {
// }