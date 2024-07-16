import serial
import time
com = serial.Serial('COM15', timeout=5)

com.dtr = 0
com.dtr = 1

print('open success')

total_len = 0
pack_len = 1024 * 1024
receive_len = 1000 * 1024 * 1024

try:
    t1 = time.monotonic()
    while data := com.read(pack_len):
        total_len += len(data)
        if total_len >= receive_len:
            break
except KeyboardInterrupt:
    ...
finally:
    t2 = time.monotonic()

t = t2-t1
speed = total_len/t
print(f"{t:g}s {total_len/1024:.1f}KB {speed/1024:.2f}KB/s {speed*8/1024/1024:.2f}Mbps")

