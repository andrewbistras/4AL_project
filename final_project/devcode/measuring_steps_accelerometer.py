import serial
import time
import numpy as np

# Use the correct USB serial port for your ESP32-S3
serial_port = '/dev/cu.wchusbserial54E20179371'  # Replace with your actual port
baud_rate = 115200  # Must match the baud rate used in your Arduino code

# Open the serial connection
bt_serial = serial.Serial(serial_port, baud_rate, timeout=1)

time.sleep(2)  # Wait for the serial connection to stabilize

# Number of readings to collect for averaging
num_readings = 100

def flush_input_buffer():
    """Reads and discards all data in the buffer to start fresh."""
    while bt_serial.in_waiting > 0:
        bt_serial.readline()  # Read and discard old data

def collect_data_for_axis(axis_name):
    print(f"Please align the accelerometer so that the {axis_name}-axis is pointing down.")
    input("Press Enter when ready...")  # Wait for user confirmation to start collecting data
    print(f"Collecting {num_readings} readings for the {axis_name}-axis...")

    accel_data = []

    flush_input_buffer()  # Clear out any old data in the buffer
    
    # Start collecting the 100 readings after user presses Enter
    for _ in range(num_readings):
        # Wait for incoming data
        while bt_serial.in_waiting == 0:
            time.sleep(0.01)  # Small delay to allow data to arrive

        # Read and decode the data
        data = bt_serial.readline().decode('utf-8').strip()

        try:
            # Split the data by commas
            accel_x_str, accel_y_str, accel_z_str = data.split(',')

            # Convert the strings to floats
            accel_x = float(accel_x_str)
            accel_y = float(accel_y_str)
            accel_z = float(accel_z_str)

            print(accel_x, accel_y, accel_z)

            parsed_data = [accel_x, accel_y, accel_z]

            print(f"Raw data for {axis_name}-axis: {parsed_data}")  # Debugging line to print raw data

            accel_data.append(parsed_data)
        
        except ValueError:
            print("Data format error, skipping...")
            continue
    
    # Check if we have collected enough data
    if len(accel_data) == 0:
        print(f"No valid data collected for {axis_name}-axis.")
        return np.array([0, 0, 0])

    print(f"Collected {len(accel_data)} readings for {axis_name}-axis.")  # Debugging line to check data length

    # Check the data before averaging
    print(f"Data before averaging for {axis_name}-axis: {accel_data}")  # Debugging line to check data content

    # Calculate the average for this axis
    avg_data = np.mean(accel_data, axis=0)
    print(f"Average data for {axis_name}-axis: {avg_data}")
    return avg_data


# Collect data for X, Y, and Z axes
x_avg = collect_data_for_axis("X")
y_avg = collect_data_for_axis("Y")
z_avg = collect_data_for_axis("Z")

# Measured values matrix (from the accelerometer data)
measured_values = np.array([x_avg, y_avg, z_avg])
measured_values /= 16384  # MPU6050

# Expected gravity values matrix (ideal values for each axis)
expected_gravity = np.array([
    [9.81, 0, 0],  # When X-axis is down
    [0, 9.81, 0],  # When Y-axis is down
    [0, 0, 9.81]   # When Z-axis is down
])

# Calculate the calibration matrix
calibration_matrix = np.dot(expected_gravity, np.linalg.inv(measured_values))
print("Calibration matrix:")
print(calibration_matrix)

# Function to apply the calibration matrix to new data
def apply_calibration(raw_data):
    calibrated_data = np.dot(calibration_matrix, raw_data)
    return calibrated_data

print("\nCalibration complete! Now printing corrected accelerometer data in m/s².\n")

# Continuously read data from the accelerometer, apply the calibration matrix, and print results
while True:
    if bt_serial.in_waiting > 0:
        data = bt_serial.readline().decode('utf-8').strip()
        print(data)

        # Assuming the data is split by commas
        try:
            accel_x_str, accel_y_str, accel_z_str = data.split(',')
            raw_data = np.array([float(accel_x_str), float(accel_y_str), float(accel_z_str)])

            # Apply the calibration matrix
            calibrated_data = apply_calibration(raw_data)

            # Print the corrected acceleration data in m/s²
            print(f"Calibrated Acceleration (m/s²): X={calibrated_data[0]:.2f}, Y={calibrated_data[1]:.2f}, Z={calibrated_data[2]:.2f}")

        except ValueError:
            print("Data format error, skipping...")
