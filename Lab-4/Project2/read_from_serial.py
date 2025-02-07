
import serial
import time
import csv
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

# File path for data logging
DATA_PATH = './sensor_data.csv'
#COM = '/dev/cu.usbmodem160464801'  # Update this to match your Arduino's port
BAUD = 115200

def read_from_serial(path=DATA_PATH):
    # Adding Header Row with Columns of CSV
    with open(path, mode='w') as sensor_file:
        sensor_writer = csv.writer(
            sensor_file, 
            delimiter=',', 
            quotechar='"', 
            quoting=csv.QUOTE_MINIMAL
        )
        sensor_writer.writerow([
            'Time(ms)',
            'Distance(cm)',
            'Heading',
            'Roll',
            'Pitch',
            'Obstacle_Detected',
            'Sys_Cal',
            'Gyro_Cal',
            'Accel_Cal',
            'Mag_Cal'
        ])
    
    # Creating Serial Object for COM Port and Selected Baud Rate
    x = serial.Serial(COM, BAUD, timeout=0.1)
    print("Starting data collection...")
    
    # Reading Serial Values and Storing into CSV
    while x.isOpen() is True:
        try:
            data = x.readline().decode('utf-8').rstrip()
            print(data)  # Print raw data for monitoring
            
            if data:
                with open(path, mode='a') as sensor_file:
                    line = data.split(',')
                    if len(line) == 10:  # Verify we have all expected columns
                        sensor_writer = csv.writer(
                            sensor_file, 
                            delimiter=',', 
                            quotechar='"', 
                            quoting=csv.QUOTE_MINIMAL
                        )
                        sensor_writer.writerow(line)
                    
        except KeyboardInterrupt:
            print("\nData collection stopped by user")
            x.close()
            break
        except Exception as e:
            print(f"Error: {e}")
            continue

def plot_data(path=DATA_PATH):
    df = pd.read_csv(path)
    
    # Convert time from milliseconds to seconds
    df['Time(ms)'] = df['Time(ms)'] / 1000.0
    
    # Creating subplots
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    
    axes[0].plot(df['Time(ms)'], df['Heading'], label='Yaw (Heading)', color='b')
    axes[0].set_ylabel('Yaw (degrees)')
    axes[0].legend()
    axes[0].grid()
    
    axes[1].plot(df['Time(ms)'], df['Distance(cm)'], label='Distance', color='r')
    axes[1].set_ylabel('Distance (cm)')
    axes[1].legend()
    axes[1].grid()
    
    axes[2].plot(df['Time(ms)'], df['Roll'], label='Roll', color='g')
    axes[2].set_ylabel('Roll (degrees)')
    axes[2].legend()
    axes[2].grid()
    
    axes[3].plot(df['Time(ms)'], df['Pitch'], label='Pitch', color='m')
    axes[3].set_ylabel('Pitch (degrees)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend()
    axes[3].grid()
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    #read_from_serial()
    plot_data()
