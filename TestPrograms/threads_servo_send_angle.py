import threading
import serial
import time

threads = []

ard1 = "/dev/arduino_0"
ard2 = "/dev/arduino_1"
ard3 = "/dev/arduino_2"

ser1 = None
ser2 = None
ser3 = None

def calculate_delay(angle):
  # 5 degrees to 2 seconds
  # angle is in degrees
  delay = (abs(angle) // 10) * 2
  return delay

def l1_thread():
  global ser1
  global ser2
  global ser3

  time.sleep(2)
  print("Inside L1")
  angle = 5
  # ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  ser1.write((str(angle)).encode('utf-8'))
  time.sleep(calculate_delay(angle))

  angle = 150
  # ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  ser1.write((str(angle)).encode('utf-8'))
  time.sleep(calculate_delay(angle))
  print("Maga L1 pitch reached ", angle, "degrees")

def l2_thread():
  global ser1
  global ser2
  global ser3

  time.sleep(2)
  print("Inside L2")
  angle = 5
  # ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  ser2.write((str(angle)).encode('utf-8'))
  time.sleep(calculate_delay(angle))

  angle = 150
  # ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  ser2.write((str(angle)).encode('utf-8'))
  time.sleep(calculate_delay(angle))

  print("Maga L2 pitch reached ", angle, "degrees")

def main():
  global ser1
  global ser2
  global ser3
  ser1 = serial.Serial(ard1, 9600, timeout=1)
  ser1.reset_input_buffer()
  ser2 = serial.Serial(ard2, 9600, timeout=1)
  ser2.reset_input_buffer()
  ser3 = serial.Serial(ard3, 9600, timeout=1)
  ser3.reset_input_buffer()

  time.sleep(5)

  l1 = threading.Thread(target=l1_thread)
  l1.start()
  threads.append(l1)

  l2 = threading.Thread(target=l2_thread)
  l2.start()
  threads.append(l2)

  for t in threads:
    t.join()

  ser1.close()
  ser2.close()
  ser3.close()

  print("BYE maga")

if __name__ == "__main__":
  main()
