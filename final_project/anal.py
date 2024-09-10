'''
There are two functions here: scrape_displacement_data and make_plot.

scrape_displacement_data gets the displacement that's estimated in the code that ran the experiment, test.py
It's not accurate and shouldn't be used for much EXCEPT to get the start and stop times of the experiment.
This is because in the experiment I manually set the velocity to 0 when we weren't actively running it, which means
you can see very clearly when the velocity changes and so when the experiment starts. This is very important to get right
because the more inaccurate our start times the more that the noise of the accelerometer ruins our analysis.

make_plot draws any plot for you based on the acceleration data. Use this once you get the start and stop times.
I commented the function so check out the args
'''

import csv
import matplotlib.pyplot as plt


def scrape_displacement_data(filename):
    '''
    reads out the not very accurate displacement data used by the experiment code.

    Args: .csv file

    Prints out a guess for the start and stop times
    '''

    # Initialize lists to store time and displacement data
    time_data = []
    time_order = [0]
    displacement = []

    # Open and read the CSV file
    with open(filename, newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        
        # Skip the header
        next(csvreader)
        next(csvreader)
        
        time_data.append(float(next(csvreader)[3]))
        
        # Iterate over the rows to extract displacement and time data
        for i, row in enumerate(csvreader):
            if i % 2 == 0:  # Even-indexed rows contain displacement data
                displacement.append(float(row[0]))  # Displacement X is the first value
            else:
                time_data.append(float(row[3])+time_data[-1])  # Elapsed time is the fourth value
                time_order.append((int)(i/2))

    relative_velocity = displacement[:30]  # where relative velocity will be the mean of the displacements
    found_start = False
    for i in range(30, len(displacement)):
        relative_velocity.append(displacement[i])
        relative_velocity.pop(0)
        if (not found_start):
            if (abs(relative_velocity[-1] - relative_velocity[0]) > 0.2):
                start = time_data[i]
                found_start = True
                print("experiment starts at around", start, "seconds")
                i += 300
                relative_velocity = displacement[i:i+30]
        else:
            if (abs(relative_velocity[-1] - relative_velocity[0]) < 0.2):
                print("experiment ends at around", time_data[i], "seconds")
                break

    
    time_data = time_data[:len(displacement)]

    # Plot the displacement in X direction over time
    plt.plot(time_data, displacement, marker='o')
    plt.title('Displacement over Time')
    plt.xlabel('Elapsed Time (s)')
    plt.ylabel('Displacement (m)')
    plt.grid(True)
    plt.show()


def make_plot(filename, type="displacement", direction="x", what_to_graph_on_x_axis="time", start_time=0, stop_time=999999):
    '''
    uses the raw acceleration data and deos the math to get velocity and displacement over time

    Args:
    filename: name of .csv file
    type: what you're measuring. pass: "acceleration", "velocity", or "displacement"
    direction: pass: "x" "y" or "z"
    what_to_graph_on_x_axis: you can either graph over time or over the index of each measurement. Takes: "time" or "index"
    start_time: The start time for the experiment that you derived from scrape_displacement_data()
    stop_time: The stop time for the experiment
    '''

    # Initialize lists to store time and displacement data
    elapsed_time = 0  # this is for the time elapsed in the .csv file, no matter what
    time_data = []  # this is for time data within the start and stop times
    index = []
    acceleration = []
    velocity = []
    displacement = []

    #Figure out how to scrape the right direction
    if (direction == "x"):
        direction_index = 0
    if (direction == "y"):
        direction_index = 1
    if (direction == "z"):
        direction_index = 2

    # Open and read the CSV file
    with open(filename, newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        
        # Skip the header
        next(csvreader)
        next(csvreader)
        
        a, v, x, dt, t = 0, 0, 0, 0, 0
        # Iterate over the rows to extract displacement and time data
        for i, row in enumerate(csvreader):
            if i % 2 == 1:  # if we're looking at displacement data, skip
                i+=1
                continue
            
            try:
                elapsed_time += float(row[3])
            except:
                pass

            if (start_time <= elapsed_time and elapsed_time < stop_time):
                try:
                    a = float(row[direction_index])
                    dt = float(row[3])
                    t = dt + time_data[-1]
                    v = a * dt + velocity[-1]  # riemann sum: acceeleration * dt + previous velocity
                    x = v * dt + displacement[-1]
                    time_data.append(t) 
                    acceleration.append(a)
                    velocity.append(v)
                    displacement.append(x)
                    index.append(i/2)
                except:
                    acceleration.append(0)
                    velocity.append(0)
                    time_data.append(0)
                    displacement.append(0)
                    print("Row number", i, "couldn't be read properly", row,)

    
    if (type == "displacement"):
        kinematic_data = displacement
        y_label_units = "m"
    elif (type == "velocity"):
        kinematic_data = velocity
        y_label_units = "m/s"
    elif (type == "acceleration"):
        kinematic_data = acceleration
        y_label_units = "m/s/s"

    if (what_to_graph_on_x_axis == "time"):
        x_axis_data = time_data
        x_label_units = "(s)"
    if (what_to_graph_on_x_axis == "index"):
        x_axis_data = index
        x_label_units = ""

    plt.plot(x_axis_data, kinematic_data, marker='o')
    plt.title("{} in {} direction over {}".format(type, direction, what_to_graph_on_x_axis))
    plt.xlabel("{} {}".format(what_to_graph_on_x_axis, x_label_units))
    plt.ylabel("{} in {} direction ({})".format(type, direction, y_label_units))
    plt.grid(True)
    plt.show()


# examples

scrape_displacement_data('y1.csv')
make_plot('x1.csv', "displacement", "x", "time", 12, 20)
make_plot('x1.csv', "displacement", "x", "time")
make_plot('y1.csv', "displacement", "y", "time")
