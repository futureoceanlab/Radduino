from datetime import datetime
from ctypes import cdll,c_long, c_ulong,c_uint16, c_uint32,byref,create_string_buffer,c_bool,c_char_p,c_int,c_int16,c_double, sizeof, c_voidp
from TLPM import TLPM
import time
import os

#os.add_dll_directory(os.path.dirname(os.path.realpath(__file__)))
tlPM = TLPM()
deviceCount = c_uint32()
tlPM.findRsrc(byref(deviceCount))

print("devices found: " + str(deviceCount.value))

resourceName = create_string_buffer(1024)

for i in range(0, deviceCount.value):
    tlPM.getRsrcName(c_int(i), resourceName)
    print(c_char_p(resourceName.raw).value)

tlPM.close()

tlPM = TLPM()
#resourceName = create_string_buffer(b"COM1::115200")
#print(c_char_p(resourceName.raw).value)
tlPM.open(resourceName, c_bool(True), c_bool(True))

message = create_string_buffer(1024)
tlPM.getCalibrationMsg(message)
print(c_char_p(message.raw).value)
#get calibration points information
index = c_uint16(4)
serialNumber = create_string_buffer(1024)
calibrationDate = create_string_buffer(1024)
calibrationPointsCount = c_uint16(10)
author = create_string_buffer(1024)
sensorPosition = c_uint16(10)
wavelength = (c_double*8)()
power = (c_double*8)()
tlPM.getPowerCalibrationPointsInformation(index,serialNumber,calibrationDate,byref(calibrationPointsCount),author,byref(sensorPosition))
print(c_char_p(serialNumber.raw).value)
print(c_char_p(calibrationDate.raw).value)
print(calibrationPointsCount.value)
print(c_char_p(author.raw).value)
print(sensorPosition.value)
tlPM.getPowerCalibrationPoints(c_uint16(4),c_uint16(6),wavelength,power)
for i in range(8):
    print(wavelength[i].value)
time.sleep(0.5)

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
