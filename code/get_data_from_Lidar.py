# The following function is the main function used by the user to create new connection with the VLP-16 sensor,
# in order to be able to decrypt the data packages sent by the sensor on UDP protocol.

# Important input (needs to be inserted by the user):
# 1. record_time - decimal number
# 2. save_path, file_number, file_name - 3 string variables which can be changed accordance to user choice.
# 3. laserID_flag - 3 valid options: ALL_LASERS, LASER_1_and_14, LASER_1
# 4. output_flag -  decimal number(!!!)- describes the bits the user wants to switch "on"
#                   from the following vector- <x-axis> <y-axis> <z-axis> <radius> <azimuth> <reflectivity> <time_stamp>

import socket
import os
import time
from decryptor_FSD import decryptor_FSD

# laserID_flag define
ALL_LASERS = 0
LASER_1_and_14 = 1
LASER_1 = 2

# initialize all parameters
IP_LiDAR = "0.0.0.0"
port_LiDAR = 2368

# printing the data into text file, make sure to change the "save_path":
save_path = '/home/avinoam/Documents/text_Lidar'    # please insert the desired path on your computer
file_name = 'data_file_8.txt'                       # please insert the desired file name
save_name = os.path.join(save_path,file_name)

# variables - user choice
record_time = 1             # please insert the desired time of the current record
laserID_flag = ALL_LASERS   # please insert the desired flag (3 valid options: ALL_LASERS, LASER_1_and_14, LASER_1)
output_flag = 127           # decimal number- describes the bits the user wants to switch "on"
                            # insert your request by the following order (from left to right):
                            # <x-axis> <y-axis> <z-axis> <radius> <azimuth> <reflectivity> <time_stamp>
time.sleep(1)
# Create a new socket using the given address family, socket type(AF_INET-default) and protocol number(SOCK_DGRAM- default).
# creating connection between the computer and the LiDAR:
sock = socket.socket(socket.AF_INET,  # Internet
                    socket.SOCK_DGRAM)  # UDP
sock.bind((IP_LiDAR, port_LiDAR))

# get the data from the lidar:
packet = ''
decrypted_data = ''
timeout = record_time*60  # seconds
start_time = time.time()
counter = 0
writing_flag = True

while time.time() < (timeout + start_time):
    # Receive data from the socket. The return value is a pair (string, address) where string is a string representing
    # the data received and address is the address of the socket sending the data.
    # argument for this func is buffer size
    packet, addr = sock.recvfrom(2000)
    decrypted_packet = decryptor_FSD(packet,output_flag,laserID_flag,time.time())
    decrypted_data = decrypted_data + decrypted_packet
    data_file = open(save_name, "w")
    data_file.write(decrypted_data)
    decrypted_data = ''
    counter = counter + 1

# closing the socket and the data_file:
sock.close()
data_file.close()
writing_flag = False
print(counter)




