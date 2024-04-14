import threading

threads = []

def busy_wait():
  while True:
    continue

def main():
  t1 = threading.Thread(target=busy_wait)
  t1.start()
  threads.append(t1)

  t2 = threading.Thread(target=busy_wait)
  t2.start()
  threads.append(t2)

  t3 = threading.Thread(target=busy_wait)
  t3.start()
  threads.append(t3)

  # t4 = threading.Thread(target=busy_wait)
  # t4.start()
  # threads.append(t4)

  for t in threads:
    t.join()

if __name__=="__main__":
  main()