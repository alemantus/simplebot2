import serial
import matplotlib.pyplot as plt

# Open serial port
ser = serial.Serial('/dev/ttyACM0', 115200)  # Replace 'COM1' with your serial port and 9600 with baud rate

#  Initialize empty lists for data
data1 = []
data2 = []
data3 = []
data4 = []

try:
    while True:
        # Read line from serial port
        line = ser.readline().decode().strip()
        
        # Split the line into 4 parts
        parts = line.split()
        
        if len(parts) == 4:
            # Convert each part to float and append to respective lists
            data1.append(float(parts[0]))
            data2.append(float(parts[1]))
            data3.append(float(parts[2]))
            data4.append(float(parts[3]))

except KeyboardInterrupt:
    # Close serial port on keyboard interrupt
    ser.close()

# Plot the data
plt.plot(data1, label='Data 1')
plt.plot(data2, label='Data 2')
plt.plot(data3, label='Data 3')
plt.plot(data4, label='Data 4')
plt.xlabel('Samples')
plt.ylabel('Values')
plt.title('Received Data')
plt.legend()

# Save the plot as an image
plt.savefig('received_data_plot.png')

# Show the plot
plt.show()