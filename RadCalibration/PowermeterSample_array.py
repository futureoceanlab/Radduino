from datetime import datetime
from ctypes import cdll,c_long, c_ulong, c_uint32,byref,create_string_buffer,c_bool,c_char_p,c_int,c_int16,c_uint16,c_double, sizeof, c_voidp
from TLPM import TLPM
import time
import os

os.add_dll_directory(os.path.dirname(os.path.realpath(__file__)))
tlPM = TLPM()
deviceCount = c_uint32()
tlPM.findRsrc(byref(deviceCount))

print("devices found: " + str(deviceCount.value))

resourceName = create_string_buffer(1024)

for i in range(0, deviceCount.value):
    tlPM.getRsrcName(c_int(i), resourceName)
    print(c_char_p(resourceName.raw).value)
    break

tlPM.close()

tlPM = TLPM()
#resourceName = create_string_buffer(b"COM1::115200")
#print(c_char_p(resourceName.raw).value)
tlPM.open(resourceName, c_bool(True), c_bool(True))

message = create_string_buffer(1024)
tlPM.getCalibrationMsg(message)
print(c_char_p(message.raw).value)

time.sleep(0.5)

power_measurements = []
times = []
count = 0
powers =  (c_double*20)()
timestamps = (c_uint16*20)()
flag = c_uint16(1)
print(tlPM.setArrMeasurement(flag))
tlPM.getArrMeasurement(byref(flag))
print(flag)
count = c_uint16(20)
#interval = c_uint16(10000)
print(count.value)
tlPM.getPowerArrayMeasurement(byref(count),byref(timestamps),byref(powers))
#tlPM.getPowerMeasurementSequence(count,interval,powers)
print(count.value)
for i in range(20):
    print(timestamps[i],powers[i])
    #print(powers.value)


tlPM.close()
print('End program')
