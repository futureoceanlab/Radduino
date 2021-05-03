from datetime import datetime
from ctypes import cdll,c_long, c_ulong, c_uint32,byref,create_string_buffer,c_bool,c_char_p,c_int,c_int16,c_double, sizeof, c_voidp
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

#print(tlPM.setAvgTime(c_double(0.1)))
print(tlPM.setAvgCnt(c_int16(32)))
avgcnt = c_int16(0);
tlPM.getAvgCnt(byref(avgcnt))
print(avgcnt.value)
wavelength = c_double(470);
print(wavelength.value)
print(tlPM.setWavelength(wavelength))
time.sleep(0.1)
tlPM.getWavelength(c_int16(0),byref(wavelength))
time.sleep(0.1)
print(wavelength.value)
power_measurements = []
times = []
count = 0
while count < 20:
    power =  c_double()
    tlPM.measPower(byref(power))
    power_measurements.append(power.value)
    times.append(datetime.now())
    print(power.value)
    count+=1
    time.sleep(0.2)

tlPM.close()
print('End program')
