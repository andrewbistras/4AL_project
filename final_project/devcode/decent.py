import serial
import time
import numpy as np
import threading

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

            parsed_data = [accel_x, accel_y, accel_z]
            accel_data.append(parsed_data)
        
        except ValueError:
            print("Data format error, skipping...")
            continue
    
    # Check if we have collected enough data
    if len(accel_data) == 0:
        print(f"No valid data collected for {axis_name}-axis.")
        return np.array([0, 0, 0])

    # Calculate the average for this axis
    avg_data = np.mean(accel_data, axis=0)
    return avg_data

# Collect data for X, Y, and Z axes
x_avg = collect_data_for_axis("X")
y_avg = collect_data_for_axis("Y")
z_avg = collect_data_for_axis("Z")

# Measured values matrix (from the accelerometer data)
measured_values = np.array([x_avg, y_avg, z_avg])
measured_values /= 16384 / 9.81  # scaling factor for MPU6050

print(measured_values)

# Expected gravity values matrix (ideal values for each axis)
expected_gravity = np.array([
    [9.81, 0, 0],  # When X-axis is down
    [0, 9.81, 0],  # When Y-axis is down
    [0, 0, 9.81]   # When Z-axis is down
])

# Calculate the calibration matrix
# Reshape the measured values to align with expected gravity
# Measured values is an array of shape (3, 3), corresponding to each axis
measured_values = np.vstack([x_avg, y_avg, z_avg])

# Now we solve for the calibration matrix using the least-squares method
# This solves the system: calibration_matrix * measured_values.T = expected_gravity.T
calibration_matrix, _, _, _ = np.linalg.lstsq(measured_values, expected_gravity, rcond=None)

print("Calibration matrix:")
print(calibration_matrix)


# Function to apply the calibration matrix to new data
def apply_calibration(raw_data):
    calibrated_data = np.dot(calibration_matrix, raw_data)
    return calibrated_data

# Initialize velocities and displacements
velocity = np.array([0.0, 0.0, 0.0])  # Velocity in m/s (X, Y, Z)
displacement = np.array([0.0, 0.0, 0.0])  # Displacement in meters (X, Y, Z)

# Variable to indicate if velocity should be reset to 0
reset_velocity = True

# Function to continuously listen for Enter key press
def listen_for_enter():
    global reset_velocity
    while True:
        input("Press Enter to signal stop/go at any time...\n")
        reset_velocity = not reset_velocity
        print(f"Velocity reset state toggled: {reset_velocity}")

# Start the Enter key listening thread
enter_thread = threading.Thread(target=listen_for_enter, daemon=True)
enter_thread.start()

# Continuously read data from the accelerometer, apply the calibration matrix, and calculate displacement
last_time = time.time()

print("\nCalibration complete! Now printing corrected accelerometer data in m/s² and calculating displacement.\n")

while True:
    if bt_serial.in_waiting > 0:
        data = bt_serial.readline().decode('utf-8').strip()

        try:
            accel_x_str, accel_y_str, accel_z_str = data.split(',')
            raw_data = np.array([float(accel_x_str), float(accel_y_str), float(accel_z_str)])

            # Apply the calibration matrix
            calibrated_data = apply_calibration(raw_data)

            # Remove the effect of gravity from the x?-axis reading
            calibrated_data[2] -= 9.81  # Subtract gravity from Z-axis

            # Print the corrected acceleration data in m/s²
            print(f"Calibrated Acceleration (m/s²): X={calibrated_data[0]:.2f}, Y={calibrated_data[1]:.2f}, Z={calibrated_data[2]:.2f}")

            # Calculate elapsed time since last reading
            current_time = time.time()
            elapsed_time = current_time - last_time
            last_time = current_time

            if reset_velocity:
                velocity = np.array([0.0, 0.0, 0.0])


            # Update velocity by integrating acceleration (v = v0 + a * dt)
            velocity += calibrated_data * elapsed_time

            # Update displacement by integrating velocity (s = s0 + v * dt)
            displacement += velocity * elapsed_time

            # Print the estimated displacement
            print(f"Estimated Displacement (m): X={displacement[0]:.2f}, Y={displacement[1]:.2f}, Z={displacement[2]:.2f}")

        except ValueError:
            print("Data format error, skipping...")
