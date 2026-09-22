import struct
import csv
import wave
import sys
import os

# Sensor IDs matching the C++ enum
ID_ADXL371_MAIN   = 0x01 
ID_ADXL371_SAT    = 0x02 
ID_LSM_MAIN       = 0x03 
ID_LSM_SAT        = 0x04 
ID_BME280_MAIN    = 0x05 
ID_BME280_SAT     = 0x06 
ID_MIC            = 0x07 
PFM               = 0x08 
EOF_              = 0xFF

# Constants
ADXL_SAMPLES_PER_PACKET = 100
ADXL_SAMPLE_RATE_HZ = 5120

LSM_SAMPLES_PER_PACKET = 150
LSM_SAMPLE_RATE_HZ = 6670
# Calculate the time delta between individual FIFO samples in microseconds
ADXL_DELTA_T_US = 1000000.0 / ADXL_SAMPLE_RATE_HZ 
LSM_DELTA_T_US = 1000000.0 / LSM_SAMPLE_RATE_HZ 

def create_csv_writer(file_handle, header_row):
    writer = csv.writer(file_handle)
    writer.writerow(header_row)
    return writer

def parse_binary(input_file):
    print(f"Parsing {input_file}...")
    name = input_file.split('.b')[0]

    os.mkdir(name)

    # Open all output files
    with open(f'{name}\\adxl371_main.csv', 'w', newline='') as f_adxl_main, \
         open(f'{name}\\adxl371_sat.csv', 'w', newline='') as f_adxl_s, \
         open(f'{name}\\lsm_main.csv', 'w', newline='') as f_lsm_main, \
         open(f'{name}\\lsm_sat.csv', 'w', newline='') as f_lsm_s, \
         open(f'{name}\\bme280_main.csv', 'w', newline='') as f_bme_main, \
         open(f'{name}\\bme280_sat.csv', 'w', newline='') as f_bme_s, \
         wave.open(f'{name}\\flight_audio.wav', 'wb') as wav_file, \
         open(input_file, 'rb') as f_in:

        # Setup CSV Headers
        adxl_header = ['timestamp_us', 'raw_x', 'raw_y', 'raw_z', 'g_x', 'g_y', 'g_z']
        lsm_header  = ['timestamp_us', 'raw_x', 'raw_y', 'raw_z', 'acc_x', 'acc_y', 'acc_z',\
                        'raw_wx', 'raw_wy', 'raw_wz', 'gyr_x', 'gyr_y', 'gyr_z', 'tmp']
        bme_header  = ['timestamp_us', 'temperature_c', 'pressure_pa', 'humidity_rh']

        csv_adxl_main  = create_csv_writer(f_adxl_main, adxl_header)
        csv_adxl_s     = create_csv_writer(f_adxl_s, adxl_header)
        csv_lsm_main   = create_csv_writer(f_lsm_main, lsm_header)
        csv_lsm_s      = create_csv_writer(f_lsm_s, lsm_header)
        csv_bme_main   = create_csv_writer(f_bme_main, bme_header)
        csv_bme_s      = create_csv_writer(f_bme_s, bme_header)

        # Configure WAV file (Mono, 16-bit, 44.1kHz)
        wav_file.setnchannels(2)
        wav_file.setsampwidth(2) 
        wav_file.setframerate(44100)

        # Pre-compile struct unpackers for extreme speed
        # Header: <HBII = Little Endian, uint8, uint8, uint32, uint32
        header_struct = struct.Struct('<BBII') 
        # BME280 Data: <fff = Little Endian, 3 floats
        bme_struct = struct.Struct('<fff')
        # ADXL371 Data: <510h = Little Endian, 510 int16s (170 triplets * 3 axes)
        adxl_struct = struct.Struct(f'<{ADXL_SAMPLES_PER_PACKET*3}h')
        # LSM Data: <510h = Little Endian, 510 int16s (170 triplets * 3 axes) + 1 tmp
        lsm_struct = struct.Struct(f'<{1+LSM_SAMPLES_PER_PACKET*6}h')

        bytes_read = 0
        file_size = os.path.getsize(input_file)

        previous_timestamps = {}
        previous_raw_timestamps = {}
        timestamp_offsets = {}
        UINT32_MAX = 4294967296  # 2^32
        while True:
            header_bytes = f_in.read(10)
            if len(header_bytes) < 10:
                break # End of file
            
            bytes_read += 10
            sync_word, sensor_type, timestamp, payload_len = header_struct.unpack(header_bytes)

            # Robust SD Card glitch recovery
            if sync_word != 0xAA:
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
                left = payload_bytes[:128]
                right = payload_bytes[128:]

                stereo_data = bytearray(256)
                offset = 0 
                for i in range(128): 
                    # Left sample 
                    stereo_data[offset:offset + 2] = struct.pack( '<h', left[i] ) 
                    offset += 2 

                    # Right sample 
                    stereo_data[offset:offset + 2] = struct.pack( '<h', right[i] ) 
                    offset += 2 

                wav_file.writeframesraw(stereo_data)

            elif sensor_type in (ID_BME280_MAIN, ID_BME280_SAT):
                temp, press, hum = bme_struct.unpack(payload_bytes)
                row = [true_timestamp, temp, press, hum]
                
                if sensor_type == ID_BME280_MAIN:
                    csv_bme_main.writerow(row)
                else:
                    csv_bme_s.writerow(row)

            elif sensor_type in (ID_ADXL371_MAIN, ID_ADXL371_SAT):
                raw_axes = adxl_struct.unpack(payload_bytes[:adxl_struct.size])
                
                # --- NEW DYNAMIC TIMING LOGIC ---
                # Track the previous timestamp to calculate the exact hardware clock speed
                if sensor_type not in previous_timestamps:
                    previous_timestamps[sensor_type] = true_timestamp - (ADXL_SAMPLES_PER_PACKET * ADXL_DELTA_T_US)
                
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

                if sensor_type == ID_ADXL371_MAIN:
                    csv_adxl_main.writerows(rows)
                elif sensor_type == ID_ADXL371_SAT:
                    csv_adxl_s.writerows(rows)

            elif sensor_type in (ID_LSM_MAIN, ID_LSM_SAT):
                raw_axes = lsm_struct.unpack(payload_bytes[:lsm_struct.size])
                
                # --- NEW DYNAMIC TIMING LOGIC ---
                # Track the previous timestamp to calculate the exact hardware clock speed
                if sensor_type not in previous_timestamps:
                    previous_timestamps[sensor_type] = true_timestamp - (LSM_SAMPLES_PER_PACKET * LSM_DELTA_T_US)
                
                actual_delta_t = (true_timestamp - previous_timestamps[sensor_type]) / LSM_SAMPLES_PER_PACKET
                previous_timestamps[sensor_type] = true_timestamp
                # --------------------------------
                
                # Payload packet is temperature
                raw_temp = raw_axes[0]
                temp_celsius = 25.0 + (raw_temp / 256.0)

                rows = []
                for i in range(LSM_SAMPLES_PER_PACKET):
                    idx = 1 + i * 6
                    rx = raw_axes[idx]
                    ry = raw_axes[idx+1]
                    rz = raw_axes[idx+2]
                    gx = raw_axes[idx+3]
                    gy = raw_axes[idx+4]
                    gz = raw_axes[idx+5]
                    
                    Accx, Accy, Accz = rx*0.00976, ry*0.00976, rz*0.00976
                    Wx, Wy, Wz = gx*0.07, gy*0.07, gz*0.07

                    # Use the dynamically calculated delta_t to prevent overlaps
                    sample_time = true_timestamp - ((LSM_SAMPLES_PER_PACKET - 1 - i) * actual_delta_t)
                    
                    rows.append([sample_time, rx, ry, rz, Accx, Accy, Accz, gx, gy, gz, Wx, Wy, Wz, temp_celsius])

                if sensor_type == ID_LSM_MAIN:
                    csv_lsm_main.writerows(rows)
                elif sensor_type == ID_LSM_SAT:
                    csv_lsm_s.writerows(rows)
               

            # Simple progress tracker
            if bytes_read % 10485760 < 11: # Print every ~10 MB
                print(f"Processed {bytes_read / 1048576:.1f} MB of {file_size / 1048576:.1f} MB...")

    print("Parsing Complete! CSVs and WAV file generated successfully.")

if __name__ == '__main__':
    # Replace with your actual file name
    parse_binary("data_0008.bin")