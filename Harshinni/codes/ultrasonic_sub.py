import serial

# Open a serial connection to the Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace '/dev/ttyUSB0' with the correct port name

while True:
    # Read a line of data from the Arduino
    data = ser.readline().decode('utf-8').strip()
    
    # Check if the received data starts with 'Distances:'
    if data.startswith('Distances:'):
        # Extract the distance values from the data
        distances_str = data[len('Distances:'):]
        distances = [int(val) for val in distances_str.split(',')]
        
        # Print the distances
        print("Distance 1: {} cm".format(distances[0]))
        print("Distance 2: {} cm".format(distances[1]))
        print("Distance 3: {} cm".format(distances[2]))
        print("Distance 4: {} cm".format(distances[3]))
