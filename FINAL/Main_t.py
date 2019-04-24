#!/usr/bin/python

# General system related libraries
import time
import pytictoc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
style.use('fivethirtyeight')
import os
import sys
import warnings

# Calaculatin and data manipulation libraries:
import numpy as np
import pandas as pd
pd.set_option('float_format','{:f}'.format)
import pickle
import struct

# Project modules
from Track_calculations import cone_finder
from Mapping import createMovie
import LIDAR
import IMU
import Server_Client
import utilities


# USER PARAMETERS:
WORK_MODE = 0           # offline(reading data from pickle_file) = 0 , online(recording to pickle file) = 1
PLOT = 0                # PLOT = 1 will show live plot of one of the options under "if-sentence", PLOT = 0 means no plotting
FOV = 170
NUM_OF_LASERS = 16      # VLP-16 Lidar
NUM_OF_POINTS = 384     # number of points in each lidar packet
NUM_OF_IMU_DATA = 7     # Yaw/Pitch/Roll/Lon/Lat/Alt/Timestamp
PATH = '/home/avinoam/Desktop/'
TEST_NAME = time.strftime("%X_%d.%m.%y")
# TEST_NAME = 'test'

#files names
IMU_CSV =           PATH + 'IMU_' + TEST_NAME + '.csv'
IMU_INTERPOLATED =  PATH + 'IMU_after_interpolation_'+TEST_NAME+'.csv'
LIDAR_CSV =         PATH + 'LIDAR_' + TEST_NAME + '.csv'
CONES_CSV =         PATH + 'CONES_' + TEST_NAME + '.csv'
LIDAR_w_IMU_Cor =   PATH + 'Lidar_w_IMU_Correction_' + TEST_NAME + '.csv'
# IMU_PKL =           PATH + 'imu_rec_' + TEST_NAME + '.pkl'
# LIDAR_PKL =         PATH + 'lidar_rec_' + TEST_NAME + '.pkl'

# IMU_PKL = PATH + 'imu_rec_test.pkl'
# LIDAR_PKL = PATH + 'lidar_rec_test.pkl'
LIDAR_PKL =         PATH + 'lidar_rec_16:09:36_13.04.19.pkl'
IMU_PKL =           PATH + 'imu_rec_16:09:36_13.04.19.pkl'

class joint_feeder:
    def __init__(self):
        if WORK_MODE: # online_mode
            self.imu_feeder = IMU.vn200_online_feeder(IMU_PKL)
            time.sleep(0.5)
            self.lidar_feeder = LIDAR.vlp16_online_feeder(LIDAR_PKL)
        else:   # offline mode
            self.imu_feeder = IMU.vn200_offline_feeder(IMU_PKL)
            time.sleep(0.5)
            self.lidar_feeder = LIDAR.vlp16_offline_feeder(LIDAR_PKL)


    def close_joint_feeder(self, N_packets):
        if WORK_MODE: # online_mode
            self.lidar_feeder.close_socket()
            self.imu_feeder.close_imu()
            time.sleep(2)
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
    imu_decoder = IMU.vn200_decoder(feeder.imu_feeder)

    try:
        client = Server_Client.Client()

        timeout = 1

        N_frames = timeout * 10    #  10 frames per sec (velo working freq is 10Hz)

        t = pytictoc.TicToc()
        time_val = 0

        for frame in range(N_frames):
            t.tic()
            # utilities.print_progress(packet, N_frames-1)
            decoded_frame, timestamp_list = lidar_decoder.get_full_frame()
            xyz_time_cones , fov_frame, cones_reflectivity = cone_finder(decoded_frame, timestamp_list, FOV)
            if np.all(xyz_time_cones) == None:
                continue
            xyzCones_world, y_car, x_car = imu_decoder.get_world_coords(xyz_time_cones)

            # XY plot (Z=0) of lidar fov frame and cones
            if PLOT:
                plt.ion()
                fig = plt.figure(1, figsize=[20,10])
                warnings.filterwarnings("ignore",".*GUI is implemented.*")
                utilities.plot_3d(fig, xyz_time_cones)
                # utilities.plot_2d(fig, xyz_time_cones)
                # lidar_decoder.plot_lidar_cones(fig, fov_frame, xyz_time_cones, t_frame)

            # sending xyz_time_cones, x_car, y_car (as bytes!) to server
            data = np.array(xyzCones_world.values)
            x_car = np.array(x_car)
            y_car = np.array(y_car)
            data = np.append(data, x_car, y_car)
            # xyzCones_world_val = xyzCones_world_val.copy(order = 'C')
            # x_car = np.array(x_car).copy(order = 'C')
            # y_car = np.array(y_car).copy(order = 'C')
            # data = xyzCones_world_val + x_car + y_car
            print(data, type(data))
            # client.send_packet(data)
            time_val += t.tocvalue()


    except KeyboardInterrupt or SystemError or OSError or TypeError or ValueError:
        feeder.close_joint_feeder(N_frames)

    # with open('/home/avinoam/Desktop/cones_13_04_19.pkl', 'wb') as cones_f:
    #         pickle.dump(cones, cones_f)
    # createMovie('/home/avinoam/Desktop/img_13_04_19')
    print('Average frame\'s decoding time: ' + str(round((time_val/N_frames),3)) + ' sec')
    feeder.close_joint_feeder(N_frames)


if __name__ == '__main__':
    main()