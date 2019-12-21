import serial
import time

s = serial.Serial('/dev/ttyUSB1', 115200)

def send(value,endvalue=1234,waitack=False):
  message = bytes("BBB", encoding='ascii')
  checksum = 0
  for i in range(10):
    message += (value).to_bytes(2,byteorder='little')
    checksum += value
  message += (checksum % 65536).to_bytes(2,byteorder='little')
  s.write(message)
  if waitack:
    result = s.readline()
    if result.decode() == "OK\r\n": return True
    else: return result
  else:
    return True

def loop(dt=0.005):
  a = 0
  lastTime = time.time()
  while True:
    currentTime = time.time()
    ret = send(a,a+1)
    if ret is not True: 
      print("Erro de checksum, retornou: " + str(ret))
    else: print(currentTime-lastTime, end='\r')
    lastTime = currentTime
    time.sleep(dt)
    a = a+1
