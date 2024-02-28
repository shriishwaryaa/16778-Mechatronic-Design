import serial
import time

# Open serial connection to Arduino
arduino_acc = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

def main():
    data_acc = arduino_acc.readline().decode().strip()

    while True:
        	# Send commands to Arduinos
            data_acc = arduino_acc.readline().decode().strip()
            print("Accelerometer data : ", data_acc)

if __name__ == '__main__':
	 main()
