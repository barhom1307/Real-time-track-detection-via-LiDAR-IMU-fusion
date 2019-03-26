#!/usr/bin/python

# General system related libraries
import time
import pytictoc
import matplotlib.pyplot as plt
import os
import sys
import warnings

# Calaculatin and data manipulation libraries:
import numpy as np
import pandas as pd
pd.set_option('float_format','{:f}'.format)
import pickle

# Project modules
from Track_calculations import cone_finder
import LIDAR
import IMU
import Server_Client
import utilities


# USER PARAMETERS:
WORK_MODE = 0           # offline(reading data from pickle_file) = 0 , online(recording to pcap_file) = 1
FOV = 160               # field of view
NUM_OF_LASERS = 16      # VLP-16 Lidar
NUM_OF_POINTS = 384     # number of points in each lidar packet
NUM_OF_IMU_DATA = 7     # Yaw/Pitch/Roll/Lon/Lat/Alt/Timestamp
PATH = '/home/avinoam/Desktop/'
TEST_NAME = time.strftime("%X_%d.%m.%y")


#files names
IMU_CSV =           PATH + 'IMU_' + TEST_NAME + '.csv'
IMU_INTERPOLATED =  PATH + 'IMU_after_interpolation_'+TEST_NAME+'.csv'
LIDAR_CSV =         PATH + 'LIDAR_' + TEST_NAME + '.csv'
CONES_CSV =         PATH + 'CONES_' + TEST_NAME + '.csv'
LIDAR_w_IMU_Cor =   PATH + 'Lidar_w_IMU_Correction_' + TEST_NAME + '.csv'
IMU_PKL =           PATH + 'imu_rec_' + TEST_NAME + '.pkl'
LIDAR_PKL =         PATH + 'lidar_rec_15:09:06_25.03.19.pkl'
#LIDAR_PKL =         PATH + 'lidar_rec_' + TEST_NAME + '.pkl'


class joint_feeder:
    def __init__(self):
        if WORK_MODE: # online_mode
            self.lidar_feeder = LIDAR.vlp16_online_feeder(LIDAR_PKL, FOV)
            #self.imu_feeder = IMU.vn200_online_feeder(IMU_PKL)
        else:   # offline mode
            self.lidar_feeder = LIDAR.vlp16_offline_feeder(LIDAR_PKL)
            # self.imu_feeder = IMU.vn200_offline_feeder(IMU_PKL)

    def close_joint_feeder(self, N_packets):
        if WORK_MODE: # online_mode
            self.lidar_feeder.close_socket()
            #self.imu_feeder.close_imu()
            print ('Done!!! - {} frames has been recorded'.format(N_packets))
        else:
            print ('Done!!! - {} frames has been decrypted'.format(N_packets))


def main():

    if not os.path.isdir(PATH):
        print('Error- the PATH you have entered dosen\'t exists')
        exit(0)
    #timeout = int(input('Please enter a desired recording time (in seconds): ')) # seconds

    feeder = joint_feeder()
    lidar_decoder = LIDAR.vlp16_decoder(feeder.lidar_feeder)
    # imu_feeder = feeder.imu_feeder

    client = Server_Client.Client()

    timeout = 1
    N_frames =  timeout * 10    #  10 frames per sec (velo working freq is 10Hz)

    t = pytictoc.TicToc()
    first_timeStamp = 0

    for frame in range(N_frames):
        t.tic()
        # utilities.print_progress(packet, N_frames-1)
        decoded_frame, timestamp_list = lidar_decoder.get_full_frame()

        #saving the first time_stamp and continuing with frame's reading - ignoring the first frame due to stop angle issues
        if frame == 0:
            first_timeStamp = timestamp_list[0]
            continue

        t_frame = timestamp_list[0] - first_timeStamp
        #print("#### - ", frame, "   " , decoded_frame[0][3], decoded_frame[-1][3])
        xyz_time_cones , fov_frame = cone_finder(decoded_frame, timestamp_list, FOV)

        #t.toc()

        # XY plot (Z=0) of lidar fov frame and cones
        #if not WORK_MODE:
        #   lidar_decoder.plot_2d(fov_frame, xyz_time_cones, t_frame)

        # sending xyz_time_cones to server "on the other side" :-)
        # client.send_packet(xyz_time_cones)

        if np.all(xyz_time_cones) == None:
            continue
        # else:
        #     cone_to_csv = pd.DataFrame(xyz_time_cones)
        #     cone_to_csv.to_csv(CONES_CSV, header= ['X', 'Y', 'Z', 'Time'], mode= 'a')

    feeder.close_joint_feeder(N_frames)


if __name__ == '__main__':
    main()