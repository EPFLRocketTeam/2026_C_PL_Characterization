#include "main.h"

#include "buzzer.h"
#include "packets.h"
#include "sd_card.h"
#include "microphone.h"
#include "environment.h"
#include "accelerometer.h"
#include "diagnostics.h"
#include "acquisition_scheduler.h"

ADXL371class adxl_main(CS_PIN_ADXL_Main, SPI);		// Constructor different for each lib...
Adafruit_BME280 bme_main(CS_PIN_BME_Main, &SPI);
LSM6DSO32Sensor lsm_main(&SPI, CS_PIN_LSM_Main, 10000000);

ADXL371class adxl_sat(CS_PIN_ADXL_Sat, SPI1);
Adafruit_BME280 bme_sat(CS_PIN_BME_Sat, &SPI1);
LSM6DSO32Sensor lsm_sat(&SPI1, CS_PIN_LSM_Sat, 10000000);

uint32_t last_bme_read = 0; // ms 

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

void setup() {
	delay(60000);
	SPI.begin();
	SPI1.begin();
	#if DEBUG_
	Serial.begin(115200);
	while (!Serial) {}
	Serial.println("----- ACQUISITION DIAGNOSTICS -----");
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
	#if DEBUG_
	Serial.println("\nConnecting ADXL371 Main");
	#endif
	has_adxl_main = setup_adxl371(&adxl_main);
	conditional_beeps(has_adxl_main, 100, 2, 200, 1);
	delay(2000);

	#if DEBUG_
	Serial.println();
	Serial.println("Connecting BME280 Main");
	#endif
	has_bme_main = setup_bme(&bme_main);
	conditional_beeps(has_bme_main, 100, 2, 200, 1);
	delay(2000);

	#if DEBUG_
	Serial.println();
	Serial.println("Connecting LSM6DSO32 Main");
	#endif
	has_lsm_main = setup_lsm(&lsm_main);
	conditional_beeps(has_lsm_main, 100, 2, 200, 1);
	delay(2000);

	// Setup main sensors
	#if DEBUG_
	Serial.println();
	Serial.println("Connecting ADXL371 Sat");
	#endif
	has_adxl_sat = setup_adxl371(&adxl_sat);
	conditional_beeps(has_adxl_sat, 100, 2, 200, 1);
	delay(2000);

	#if DEBUG_
	Serial.println();
	Serial.println("Connecting BME280 Sat");
	#endif
	has_bme_sat = setup_bme(&bme_sat);
	conditional_beeps(has_bme_sat, 100, 2, 200, 1);
	delay(2000);

	#if DEBUG_
	Serial.println();
	Serial.println("Connecting LSM6DSO32 Sat");
	#endif
	has_lsm_sat = setup_lsm(&lsm_sat);
	conditional_beeps(has_lsm_sat, 100, 2, 200, 1);
	delay(2000);

	#if DEBUG_
	Serial.println();
	if (has_adxl_main) Serial.println("ADXL371 Main Connected");
	if (has_bme_main) Serial.println("BME280 Main Connected");
	if (has_lsm_main) Serial.println("LSM6DSO32 Main Connected");
	
	if (has_adxl_sat) Serial.println("ADXL371 Sat Connected");
	if (has_bme_sat) Serial.println("BME280 Sat Connected");
	if (has_lsm_sat) Serial.println("LSM6DSO32 Sat Connected");
	Serial.println();
	#endif

	// Setup microphones
	setup_microphones();

	if (!setup_sd() || !setup_file()) {
		#if DEBUG_
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

	#if DEBUG_
	Serial.println("----- Measure Start -----");
	#endif
	
	// Setup success beeps
	beeps(100, 5);
	delay(5);

	start_microphones(); // Start queues only when the recording is about to begin.

	// Keep this start order !!! ADXL first makes lsm crash !!!
	if (has_lsm_main) { start_lsm(&lsm_main, INT_PIN_LSM_Main, lsm_main_ISR); capture_lsm_registers(&lsm_main, ID_LSM_MAIN); }
	delay(1);
    if (has_lsm_sat) { start_lsm(&lsm_sat, INT_PIN_LSM_Sat, lsm_sat_ISR); capture_lsm_registers(&lsm_sat, ID_LSM_SAT); }
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

		#if DEBUG_
        Serial.println("[CRITICAL] Power Failure Detected! Executing Panic Save.");
		#endif

        // Save recoverable health state before flushing the SD queue.
        finalize_diagnostic_logging(2);
        const bool panic_saved = close_sd(PFM_END);
        (void)panic_saved; // Used only by optional verbose Serial output.
		#if DEBUG_
        Serial.print(panic_saved ? "SD file closed in " : "SD CLOSE FAILED after ");Serial.print(millis() - poll_start);Serial.println("ms");
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

		// Sound long buzzer beep to confirm stop
		beep(2000);

        // Post-Monitoring Action
        if (power_restored) {
			#if DEBUG_
            Serial.println("Power restored! Executing Cortex-M7 hardware reset...");
            delay(100); // Allow serial buffer to flush
			#endif
            
            // Triggers a complete system reset on the Teensy
            SCB_AIRCR = 0x05FA0004; 
        } else {
			#if DEBUG_
            Serial.println("System dead. Awaiting supercapacitor depletion.");
			#endif

            while (1) {
                // Safe state. Processor will brown out peacefully.
            }
        }
    }

	// Normal operation (continuous logging)
	if (is_logging) {
        service_acquisition_pass();

		// Read Microphones
		read_microphone();

		// Read BME280s at 10 Hz (every 100ms) without blocking
		if (millis() - last_bme_read >= 100) {
			if (has_bme_main) log_bme_values(&bme_main, ID_BME280_MAIN);
			if (has_bme_sat) log_bme_values(&bme_sat, ID_BME280_SAT);
			last_bme_read = millis();
		}

        service_diagnostic_logging();
        service_sd_if_ready();
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

		#if DEBUG_
		Serial.println("Safe shutdown triggered.");
		#endif

        // Final cumulative snapshot, including bounded traces, before EOF.
        finalize_diagnostic_logging(1);
        const bool saved = close_sd(JUMPER_END);
        (void)saved;
        detachInterrupt(digitalPinToInterrupt(INT_PIN_ADXL_Main));
        detachInterrupt(digitalPinToInterrupt(INT_PIN_ADXL_Sat));
        detachInterrupt(digitalPinToInterrupt(INT_PIN_LSM_Main));
        detachInterrupt(digitalPinToInterrupt(INT_PIN_LSM_Sat));

		#if DEBUG_
		Serial.println(saved ? "SD file truncated and closed." : "SD SAVE FAILED: inspect card and diagnostics.");
		#endif

		print_acquisition_diagnostics();
		// Sound long buzzer beep to confirm stop
		beep(2000);

		while(1) {
			// System is safe to power down
		}
    }

}
