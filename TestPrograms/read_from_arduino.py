import serial
import time

# Define serial port and baud rate
serial_port = '/dev/ttyACM1'  # Need to setup udev rules on the Pi 
baud_rate = 9600

def main():
    # Open a new serial connection
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print("Serial connection established")

    while True:
        # Read data from Arduino
        data = ser.readline().decode().strip()
        
        # Check if data is not empty
        if data:
            print("Data received from Arduino:", data)
	
	# TODO: Handle errors and do strict error checking

if __name__ == '__main__':
    main()
