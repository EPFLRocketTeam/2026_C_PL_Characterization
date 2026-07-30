import struct
import csv
import wave
import sys
import os

# Sensor IDs matching the C++ enum
ID_ADXL372_MAIN  = 0x01
ID_ADXL372_SAT_1 = 0x02
ID_ADXL372_SAT_2 = 0x03
ID_BME280_MAIN   = 0x04
ID_BME280_SAT_1  = 0x05
ID_MIC           = 0x06
PFM              = 0x07
EOF_             = 0xFF

# Constants
ADXL_SAMPLES_PER_PACKET = 83
ADXL_SAMPLE_RATE_HZ = 6400
# Calculate the time delta between individual FIFO samples in microseconds
DELTA_T_US = 1000000.0 / ADXL_SAMPLE_RATE_HZ 

def create_csv_writer(file_handle, header_row):
    writer = csv.writer(file_handle)
    writer.writerow(header_row)
    return writer

def parse_binary(input_file):
    print(f"Parsing {input_file}...")
    name = input_file.split('.b')[0]

    os.mkdir(name)

    # Open all output files
    with open(f'{name}\\adxl372_main.csv', 'w', newline='') as f_adxl_main, \
         open(f'{name}\\adxl372_sat_1.csv', 'w', newline='') as f_adxl_s1, \
         open(f'{name}\\adxl372_sat_2.csv', 'w', newline='') as f_adxl_s2, \
         open(f'{name}\\bme280_main.csv', 'w', newline='') as f_bme_main, \
         open(f'{name}\\bme280_sat_1.csv', 'w', newline='') as f_bme_s1, \
         wave.open(f'{name}\\flight_audio.wav', 'wb') as wav_file, \
         open(input_file, 'rb') as f_in:

        # Setup CSV Headers
        adxl_header = ['timestamp_us', 'raw_x', 'raw_y', 'raw_z', 'g_x', 'g_y', 'g_z']
        bme_header = ['timestamp_us', 'temperature_c', 'pressure_pa', 'humidity_rh']

        csv_adxl_main = create_csv_writer(f_adxl_main, adxl_header)
        csv_adxl_s1   = create_csv_writer(f_adxl_s1, adxl_header)
        csv_adxl_s2   = create_csv_writer(f_adxl_s2, adxl_header)
        csv_bme_main  = create_csv_writer(f_bme_main, bme_header)
        csv_bme_s1    = create_csv_writer(f_bme_s1, bme_header)

        # Configure WAV file (Mono, 16-bit, 44.1kHz)
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2) 
        wav_file.setframerate(44100)

        # Pre-compile struct unpackers for extreme speed
        # Header: <HBII = Little Endian, uint16, uint8, uint32, uint32
        header_struct = struct.Struct('<HBII') 
        # BME280 Data: <fff = Little Endian, 3 floats
        bme_struct = struct.Struct('<fff')
        # ADXL372 Data: <510h = Little Endian, 510 int16s (170 triplets * 3 axes)
        adxl_struct = struct.Struct(f'<{ADXL_SAMPLES_PER_PACKET*3}h')

        bytes_read = 0
        file_size = os.path.getsize(input_file)

        previous_timestamps = {}
        previous_raw_timestamps = {}
        timestamp_offsets = {}
        UINT32_MAX = 4294967296  # 2^32
        while True:
            header_bytes = f_in.read(11)
            if len(header_bytes) < 11:
                break # End of file
            
            bytes_read += 11
            sync_word, sensor_type, timestamp, payload_len = header_struct.unpack(header_bytes)

            # Robust SD Card glitch recovery
            if sync_word != 0xAAAA:
                f_in.seek(-10, 1) # Step back 10 bytes and try again
                bytes_read -= 10
                continue

            # --- EOF DETECTION ---
            if sensor_type == EOF_:
                print("\n[SUCCESS] Clean EOF packet detected.")
                print("Flight computer executed a safe shutdown.")
                break # Cleanly exit the while True loop

            # --- PFM DETECTION ----
            if sensor_type == PFM:
                print("\n[WARNING] Clean PFM packet detected.")
                print("Flight computer executed a power failure shutdown.")
                break # Cleanly exit the while True loop

            # Read the exact payload length specified by the header
            payload_bytes = f_in.read(payload_len)
            bytes_read += payload_len

            if len(payload_bytes) < payload_len:
                break # Unexpected EOF

            # --- MICROS() OVERFLOW UNWRAPPING ---
            if sensor_type not in previous_raw_timestamps:
                previous_raw_timestamps[sensor_type] = timestamp
                timestamp_offsets[sensor_type] = 0
            else:
                # If the current timestamp is smaller than the previous one, 
                # a 32-bit overflow just occurred.
                if timestamp < previous_raw_timestamps[sensor_type]:
                    timestamp_offsets[sensor_type] += UINT32_MAX
                
                previous_raw_timestamps[sensor_type] = timestamp
            
            # The true, continuous flight time in microseconds
            true_timestamp = timestamp + timestamp_offsets[sensor_type]

            # --- ROUTE THE DATA ---
            if sensor_type == ID_MIC:
                # Bypass unpacking entirely. Write raw PCM bytes straight to WAV.
                wav_file.writeframesraw(payload_bytes)

            elif sensor_type in (ID_BME280_MAIN, ID_BME280_SAT_1):
                temp, press, hum = bme_struct.unpack(payload_bytes)
                row = [true_timestamp, temp, press, hum]
                
                if sensor_type == ID_BME280_MAIN:
                    csv_bme_main.writerow(row)
                else:
                    csv_bme_s1.writerow(row)

            elif sensor_type in (ID_ADXL372_MAIN, ID_ADXL372_SAT_1, ID_ADXL372_SAT_2):
                raw_axes = adxl_struct.unpack(payload_bytes[:adxl_struct.size])
                
                # --- NEW DYNAMIC TIMING LOGIC ---
                # Track the previous timestamp to calculate the exact hardware clock speed
                if sensor_type not in previous_timestamps:
                    previous_timestamps[sensor_type] = true_timestamp - (ADXL_SAMPLES_PER_PACKET * DELTA_T_US)
                
                actual_delta_t = (true_timestamp - previous_timestamps[sensor_type]) / ADXL_SAMPLES_PER_PACKET
                previous_timestamps[sensor_type] = true_timestamp
                # --------------------------------
                
                rows = []
                for i in range(ADXL_SAMPLES_PER_PACKET):
                    idx = i * 3
                    rx = raw_axes[idx]
                    ry = raw_axes[idx+1]
                    rz = raw_axes[idx+2]
                    
                    gx = rx * 0.1
                    gy = ry * 0.1
                    gz = rz * 0.1

                    # Use the dynamically calculated delta_t to prevent overlaps
                    sample_time = true_timestamp - ((ADXL_SAMPLES_PER_PACKET - 1 - i) * actual_delta_t)
                    
                    rows.append([sample_time, rx, ry, rz, gx, gy, gz])

                if sensor_type == ID_ADXL372_MAIN:
                    csv_adxl_main.writerows(rows)
                elif sensor_type == ID_ADXL372_SAT_1:
                    csv_adxl_s1.writerows(rows)
                elif sensor_type == ID_ADXL372_SAT_2:
                    csv_adxl_s2.writerows(rows)

            # Simple progress tracker
            if bytes_read % 10485760 < 11: # Print every ~10 MB
                print(f"Processed {bytes_read / 1048576:.1f} MB of {file_size / 1048576:.1f} MB...")

    print("Parsing Complete! CSVs and WAV file generated successfully.")

if __name__ == '__main__':
    # Replace with your actual file name
    parse_binary("data_0003.bin")