import threading
import time

threads=[]

def send_angle(angle, leg):
  print("sending ", angle, "to ", leg)
  time.sleep(2)
  

def main():
  legs = [0, 2, 4]

  for leg in legs:
    angle = 10
    thread = threading.Thread(target=send_angle, args=[angle, leg])
    thread.start()
    threads.append(thread)

  for t in threads:
    t.join()

if __name__=="__main__":
  main()