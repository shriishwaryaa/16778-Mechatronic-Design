import serial
import time

# Open serial connection to Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

commands = ['rotate_left', 'rotate_right', 'stop']

def main():
	data = arduino.readline().decode().strip()
	while (data != "OK"):
		data = arduino.readline().decode().strip()
	while True:
        	# Send commands to Arduino
        	for command in commands:
            		print("Sending command to Arduino:", command)
            		arduino.write(command.encode())
            		time.sleep(5)  # Wait for 2 seconds between commands

if __name__ == '__main__':
	 main()
