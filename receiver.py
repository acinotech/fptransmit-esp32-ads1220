import time
from time import sleep
import serial
from serial.tools.list_ports import comports
import numpy as np

reads = 0
SERIAL_MSG_LENGTH = 18 # 4 bytes x 4 sensors + 2 EOL chars
PGA = 128                   # Internal gain = 128
SENSITIVITY = 0.002         # Ratio of excitation to output at full scale = 2 mV/V
FSF = 981/2                 # Force at full-scale (will result in 1 mV output at 1V excitation)
FSR = pow(2, 23) - 1        # Full-scale reading
NEWTONS_PER_COUNT = FSF/(FSR*PGA*SENSITIVITY)

if comports:
    serialDeviceList = list(comports())
    # print("Available serial devices:")
    for device in serialDeviceList:
        print(device.description)
        if device.description.startswith("Silicon Labs CP210x USB to UART Bridge"): # ESP32 UART-USB convertor
            port = device.device

# port = input("Copy and paste the name of the port to read from here:\n") # '/dev/cu.usbmodem346C306032311'#'COM6'#input("Enter com port: ")
baud = 500000
ser = serial.Serial(port, baud)
ser.flushInput

t0 = time.time()

while True:
    msg = ser.read_until(b'\r\n')
    # # print(msg)
    if len(msg) != SERIAL_MSG_LENGTH: continue
    msg = msg[:-2] # remove \r\n characters at message end
    data = np.frombuffer(msg, dtype='<i4') # little-endian int32_t
    # print(data)

    readings = data*NEWTONS_PER_COUNT # in N
    # readings = (data*VFSR*1000)/FSR # in mV
    print("N:", readings);  # directly print float value

    reads += 1
    print("Rate:", round(1.0 / ((time.time() - t0) / reads)), "Hz")