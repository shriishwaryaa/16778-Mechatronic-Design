import serial

def validate_angle(angle):
    try:
        angle = int(angle)
        if 0 <= angle <= 90:
            return True
    except ValueError:
        pass
    return False

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    

    while True:
        # Validate the input
        angle = input("Please enter an angle: ")
        
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("read line from arduino", line)
        except UnicodeDecodeError:
            print("Received bytes couldn't be decoded as UTF-8. Ensure the Arduino is sending valid UTF-8 strings.")
        
        if not validate_angle(angle):
            print("Invalid angle. Please enter a number between 0 and 90.")
        else:
            print("You entered:", angle)
    
            print("Sending number " + angle + " to Arduino.")
            ser.write((angle + '\n').encode('utf-8'))  # Ensure Arduino knows the end of the data
            
        print("DONE")
        
            
    ser.close()

