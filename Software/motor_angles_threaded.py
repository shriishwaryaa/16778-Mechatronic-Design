import threading
import serial
import time

threads = []

ard1 = "/dev/ttyACM0"
ard2 = "/dev/ttyACM1"

ser1 = None
ser2 = None

def l1_thread():
  global ser1
  global ser2
  time.sleep(2)
  print("Inside L1")
  angle = 5
  ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(2)
  print("Maga L1 pitch reached ", angle, "degrees")

  angle = 10
  ser1.write((str(angle) + "$" + str(1) + '\n').encode('utf-8'))
  time.sleep(2)
  print("Maga L1 pitch reached ", angle, "degrees")

  angle = -5
  ser1.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(2)
  print("Maga L1 pitch reached ", angle, "degrees")

def l2_thread():
  global ser1
  global ser2
  time.sleep(2)
  print("Inside L2")
  angle = 5
  ser2.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(2)
  print("Maga L2 pitch reached ", angle, "degrees")

  angle = 10
  ser2.write((str(angle) + "$" + str(1) + '\n').encode('utf-8'))
  time.sleep(2)

  angle = -5
  print("Maga L1 pitch reached ", angle, "degrees")
  ser2.write((str(angle) + "$" + str(0) + '\n').encode('utf-8'))
  time.sleep(2)
  print("Maga L2 pitch reached ", angle, "degrees")

def main():
  global ser1
  global ser2
  ser1 = serial.Serial(ard1, 9600, timeout=1)
  ser1.reset_input_buffer()
  ser2 = serial.Serial(ard2, 9600, timeout=1)
  ser2.reset_input_buffer()

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

  print("BYE maga")

if __name__ == "__main__":
  main()
