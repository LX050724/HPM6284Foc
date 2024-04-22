import serial
import time
com = serial.Serial('COM9')

total_len = 0

try:
    t1 = time.monotonic()
    while data := com.read(65536):
        total_len += len(data)
except KeyboardInterrupt:
    t2 = time.monotonic()

t = t2-t1
speed = total_len/t
print(f"{t:g}s {total_len/1024:.1f}KB {speed/1024:.2f}KB/s {speed*8/1024/1024:.2f}Mbps")

