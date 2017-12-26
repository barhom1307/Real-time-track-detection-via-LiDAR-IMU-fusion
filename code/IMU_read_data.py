
# The following program generates a txt file with the requested IMU info.
# The output file is built according to the following column division (8 in total):
# <x-axis> <y-axis> <z-axis> <yaw> <pitch> <roll> <velocity x-axis> <time stamp>
# Throughput rate is 20 Hz - meaning 20 rows per sec
# The txt file will be saved to <save_path> directory

from vnpy import *
import os
import time

save_path = '/home/avinoam/Documents/text_imu'   # please insert the desired path on your computer
file_name = 'data_file_2.txt'      # please insert the desired file name
save_name = os.path.join(save_path,file_name)
data_file = open(save_name, "w")


s = VnSensor()
s.connect('/dev/ttyUSB0', 115200)

timeout = 1  # seconds
start_time = time.time()
output = ''
counter = 0

while time.time() < (timeout + start_time):
    data = s.read_ins_solution_ecef()
    output = output + "%12s" % str(round(data.position.x, 1))  # x-axis in ECEF (m)
    output = output + "%12s" % str(round(data.position.y, 1))  # y-axis in ECEF (m)
    output = output + "%12s" % str(round(data.position.z, 1))  # z-axis in ECEF (m)
    output = output + "%10s" % str(round(data.yawPitchRoll.x, 2))  # yaw (deg)
    output = output + "%10s" % str(round(data.yawPitchRoll.y, 2))  # pitch (deg)
    output = output + "%10s" % str(round(data.yawPitchRoll.z, 2))  # roll (deg)
    output = output + "%10s" % str(round(data.velocity.x, 3))  # x-axis velocity in ECEF frame (m/s)
    output = output + "%20s" % str(round(time.time(),4))  #time_stamp
    output = output + '\n'
    data_file.write(output)
    output= ''
    counter = counter + 1

data_file.close()
s.disconnect()
print(counter)
