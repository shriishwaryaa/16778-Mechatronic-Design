import threading
import serial
import time
import csv
import math
threads = []
filename = 'output_joint_data.csv'
ard1 = "/dev/ttyACM0"
# ard2 = "/dev/arduino_1"

def find_row_by_first_column(filename, search_value):
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Check if the first column matches the search value
            if row[0] == search_value:
                # Convert the entire row to float where possible
                float_row = []
                for item in row:
                    try:
                        # Attempt to convert each item to a float
                        float_row.append(float(item))
                    except ValueError:
                        # If conversion fails, keep the original item
                        float_row.append(item)
                return float_row
    return None

result_row0 = find_row_by_first_column(filename, '6')
result_row1 = find_row_by_first_column(filename, '7')

if result_row0 and result_row1:
    print("Found row with float conversion where possible:", result_row0, result_row1)
else:
    print("No row found with that value.")


ser1 = None
ser2 = None

def calculate_delay(angle):
  # 5 degrees to 2 seconds
  # angle is in degrees
  # delay = (abs(angle) // 10) * 2
  return 1

def l1_thread():
  global ser1
  global ser2
  time.sleep(2)
  print("Inside L1")
  angle = 5
  ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(calculate_delay(angle))
  print("Maga L1 pitch reached ", angle, "degrees")

  angle = 10
  ser1.write((str(angle) + "$" + str(1) + '\n').encode('utf-8'))
  time.sleep(calculate_delay(angle))
  print("Maga L1 pitch reached ", angle, "degrees")

  angle = -5
  ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(calculate_delay(angle))
  print("Maga L1 pitch reached ", angle, "degrees")

def l2_thread():
  global ser1
  global ser2
  time.sleep(2)
  print("Inside L2")
  angle = 5
  ser2.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(calculate_delay(angle))
  print("Maga L2 pitch reached ", angle, "degrees")

  angle = 10
  ser2.write((str(angle) + "$" + str(1) + '\n').encode('utf-8'))
  time.sleep(calculate_delay(angle))

  angle = -5
  print("Maga L1 pitch reached ", angle, "degrees")
  ser2.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(calculate_delay(angle))
  print("Maga L2 pitch reached ", angle, "degrees")

def main():
  global ser1
  global ser2
  ser1 = serial.Serial(ard1, 9600, timeout=1)
  ser1.reset_input_buffer()

  for i in range(len(result_row0)):
    angle=math.degrees(result_row0[i])
    ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))

    # send one line for all motors 
    # angle1$0$angle2$1$angle3$2$angle4

    time.sleep(0.1)
    angle=math.degrees(result_row0[i])
    ser1.write((str(angle) + "$" + str(1) + '\n').encode('utf-8'))
    time.sleep(0.)

  print("BYE maga")

if __name__ == "__main__":
  main()
