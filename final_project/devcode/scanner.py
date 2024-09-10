import serial
import time

# Use the correct USB serial port for your ESP32-S3
serial_port = '/dev/cu.wchusbserial54E20179371'  # Replace with your actual port
baud_rate = 115200  # Must match the baud rate used in your Arduino code

# Open the serial connection
bt_serial = serial.Serial(serial_port, baud_rate, timeout=1)

time.sleep(2)  # Wait for the serial connection to stabilize

# Open a file to log the data
with open("accel_data_log.txt", "a") as log_file:
    # Continuously read data from the serial port
    while True:
        if bt_serial.in_waiting > 0:
            data = bt_serial.readline().decode('utf-8').strip()
            
            # Split the received data into components: time, accel_x, accel_y, accel_z
            try:
                accel_x, accel_y, accel_z = data.split(',')
                print(f"Acceleration: x={accel_x}, y={accel_y}, z={accel_z}")
                
                # Example of real-time analysis: calculating the magnitude of the acceleration vector
                accel_magnitude = (float(accel_x)**2 + float(accel_y)**2 + float(accel_z)**2)**0.5
                print(f"Acceleration Magnitude: {accel_magnitude} units")
                
                # Log data to the file
                log_file.write(f"{accel_x},{accel_y},{accel_z},{accel_magnitude}\n")
                log_file.flush()

            except ValueError:
                # If the data format is incorrect, skip this iteration
                continue
