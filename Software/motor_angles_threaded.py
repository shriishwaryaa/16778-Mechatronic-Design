import threading
import serial
import time
import csv
import math
threads = []
filename = 'output_joint_data.csv'
ard1 = "/dev/ttyACM1"
ard2 = "/dev/arduino_0"

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
result_row2 = find_row_by_first_column(filename, '0')
result_row3 = find_row_by_first_column(filename, '1')


# if result_row0 and result_row1:
#     print("Found row with float conversion where possible:", result_row0, result_row1)
# else:
#     print("No row found with that value.")


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

  cmd=""

  for i in range(len(result_row0)):
    # angle=math.degrees(result_row0[i])
    # 0$5$1$10$2$15$3$20'
    cmd=""
    # cmd += str(0) + "$" + str(math.degrees(result_row0[i])) + "$" + str(1) + "$" + str(math.degrees(result_row0[i])) + "$" + str(2) + "$" + str(math.degrees(result_row1[i])) + "$" + str(3) + "$" + str(math.degrees(result_row1[i]))
    cmd += str(0) + "$" + str(int(math.degrees(result_row0[i]))) + "$" + str(1) + "$" + str(int(math.degrees(result_row1[i])))
    # print("Command is ", cmd)
    ser1.write((cmd+'\n').encode('utf-8'))
    time.sleep(0.5)


def l2_thread():
  global ser1
  global ser2
  time.sleep(2)
  print("Inside L2")

  cmd=""

  for i in range(len(result_row2)):
    # angle=math.degrees(result_row2[i])
    cmd=""
    # 0$5$1$10$2$15$3$20'
    # cmd += str(0) + "$" + str(math.degrees(result_row2[i])) + "$" + str(1) + "$" + str(math.degrees(result_row2[i])) + "$" + str(2) + "$" + str(math.degrees(result_row3[i])) + "$" + str(3) + "$" + str(math.degrees(result_row3[i]))
    cmd += str(0) + "$" + str(int(math.degrees(result_row2[i]))) + "$" + str(1) + "$" + str(int(math.degrees(result_row3[i])))
    # print("Command is ", cmd)
    ser2.write((cmd+'\n').encode('utf-8'))
    time.sleep(0.5)

def main():
  global ser1
  global ser2
  global threads

  ser1 = serial.Serial(ard1, 9600, timeout=1)
  ser1.reset_input_buffer()

  ser2 = serial.Serial(ard2, 9600, timeout=1)
  ser2.reset_input_buffer()

  time.sleep(2)

  thread1 = threading.Thread(target=l1_thread)
  thread1.start()
  threads.append(thread1)

  thread2 = threading.Thread(target=l2_thread)
  thread2.start()
  threads.append(thread2)

  for t in threads:
    t.join()

  print("BYE maga")

if __name__ == "__main__":
  main()
