import os
import glob
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def process_flight_folder(target_folder):
    print(f"Scanning folder: {target_folder}")
    
    # Ensure directory exists
    if not os.path.isdir(target_folder):
        print(f"Error: Directory '{target_folder}' not found.")
        return

    # Find all CSV files in the folder
    csv_files = glob.glob(os.path.join(target_folder, '*.csv'))
    
    if not csv_files:
        print("No CSV files found in the specified folder.")
        return

    for file_path in csv_files:
        filename = os.path.basename(file_path)
        file_base = os.path.splitext(filename)[0]
        
        try:
            df = pd.read_csv(file_path)
            
            # Skip completely empty dataframes
            if df.empty:
                print(f"Skipping {filename}: File is empty.")
                continue
                
            # Setup time axis (normalized to start at 0 seconds)
            time_s = (df['timestamp_us'] - df['timestamp_us'].iloc[0]) / 1e6
            
            # --- ADXL372 ACCELEROMETER PLOTTING ---
            if 'adxl372' in filename.lower():
                print(f"Plotting Accelerometer Data: {filename}")
                
                # Calculate the magnitude (Norm) of the acceleration vector
                g_norm = np.sqrt(df['g_x']**2 + df['g_y']**2 + df['g_z']**2)
                
                plt.figure(figsize=(12, 6))
                
                # Plot individual axes with slight transparency
                plt.plot(time_s, df['g_x'], label='X-Axis', alpha=0.5, linewidth=0.5)
                plt.plot(time_s, df['g_y'], label='Y-Axis', alpha=0.5, linewidth=0.5)
                plt.plot(time_s, df['g_z'], label='Z-Axis', alpha=0.5, linewidth=0.5)
                
                # Plot the Norm distinctly over the top
                plt.plot(time_s, g_norm, label='Norm (Magnitude)', color='black', alpha=0.9, linewidth=1.0)
                
                plt.title(f'ADXL372 Acceleration Data - {file_base}')
                plt.xlabel('Time (seconds)')
                plt.ylabel('Acceleration (G)')
                plt.legend(loc='upper right')
                plt.grid(True)
                plt.tight_layout()
                
                out_path = os.path.join(target_folder, f"{file_base}_plot.png")
                plt.savefig(out_path, dpi=300) # 300 DPI for crisp zooming
                plt.close()
                print(f"  -> Saved {out_path}")
                
            # --- BME280 ENVIRONMENTAL PLOTTING ---
            elif 'bme280' in filename.lower():
                print(f"Plotting Environmental Data: {filename}")
                fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
                
                # Temperature
                axs[0].plot(time_s, df['temperature_c'], color='red')
                axs[0].set_ylabel('Temp (°C)')
                axs[0].set_title(f'BME280 Environmental Data - {file_base}')
                axs[0].grid(True)
                
                # Pressure (Converted from Pa to hPa for readability)
                axs[1].plot(time_s, df['pressure_pa'] / 100.0, color='green')
                axs[1].set_ylabel('Pressure (hPa)')
                axs[1].grid(True)
                
                # Humidity
                axs[2].plot(time_s, df['humidity_rh'], color='blue')
                axs[2].set_ylabel('Humidity (%RH)')
                axs[2].set_xlabel('Time (seconds)')
                axs[2].grid(True)
                
                plt.tight_layout()
                out_path = os.path.join(target_folder, f"{file_base}_plot.png")
                plt.savefig(out_path, dpi=300)
                plt.close()
                print(f"  -> Saved {out_path}")
                
            else:
                print(f"Skipping {filename}: Unknown sensor format.")
                
        except Exception as e:
            print(f"Error processing {filename}: {e}")

    print("\nBatch plotting complete!")

if __name__ == '__main__':        
    process_flight_folder("data_0032")