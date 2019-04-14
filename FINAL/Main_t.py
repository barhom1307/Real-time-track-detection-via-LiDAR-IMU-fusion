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

# Project modules
from Track_calculations import cone_finder
from Mapping import createMovie
import LID as LIDAR
import IMU
import Server_Client
import utilities


# USER PARAMETERS:
WORK_MODE = 1           # offline(reading data from pickle_file) = 0 , online(recording to pickle file) = 1
FOV = 160
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

IMU_PKL = PATH + 'imu_rec_test.pkl'
LIDAR_PKL = PATH + 'lidar_rec_test.pkl'
# LIDAR_PKL =         PATH + 'lidar_rec_16:09:36_13.04.19.pkl'
# IMU_PKL =           PATH + 'imu_rec_16:09:36_13.04.19.pkl'

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

    # TODO - maybe needs to be outside main func
    feeder = joint_feeder()
    lidar_decoder = LIDAR.vlp16_decoder(feeder.lidar_feeder)
    imu_decoder = IMU.vn200_decoder(feeder.imu_feeder)

    try:
        client = Server_Client.Client()

        timeout = 40
        N_frames = timeout * 10    #  10 frames per sec (velo working freq is 10Hz)

        t = pytictoc.TicToc()
        first_timeStamp = 0
        # cones = []
        time_val = 0

        for frame in range(N_frames):
            t.tic()
            # utilities.print_progress(packet, N_frames-1)
            decoded_frame, timestamp_list = lidar_decoder.get_full_frame()

            #saving the first time_stamp and continuing with frame's reading
            if frame == 0:
                first_timeStamp = timestamp_list[0]
            t_frame = timestamp_list[0] - first_timeStamp

            xyz_time_cones , fov_frame = cone_finder(decoded_frame, timestamp_list, FOV)
            if np.all(xyz_time_cones) == None:
                continue
            # cones.append(xyz_time_cones)
            # xyzCones_world, _ = imu_decoder.get_world_coords(xyz_time_cones)

            # XY plot (Z=0) of lidar fov frame and cones
            # lidar_decoder.plot_2d(fov_frame, xyz_time_cones, t_frame)
            # lidar_decoder.plot_2d(fov_frame, xyzCones_world.values, t_frame)
            # lidar_decoder.plot_2d(fov_frame, xyzCones_world, t_frame)

            time_val += t.tocvalue()


            # TODO - plot live stream IMU+lidar frame
            # fig = plt.figure()
            # ani = animation.FuncAnimation(fig, utilities.plot_3d_live, fargs=(fig, lidar_decoder, imu_decoder), interval=1)
            # plt.show()

            # sending xyz_time_cones to server "on the other side" :-)
            xyz_time_cones = xyz_time_cones.copy(order = 'C')
            client.send_packet(xyz_time_cones)


    except KeyboardInterrupt or SystemError or OSError or TypeError or ValueError:
        feeder.close_joint_feeder(N_frames)

    # with open('/home/avinoam/Desktop/cones_13_04_19.pkl', 'wb') as cones_f:
            # pickle.dump(cones, cones_f)
    # createMovie('/home/avinoam/Desktop/img_13_04_19')
    print(time_val/N_frames)
    feeder.close_joint_feeder(N_frames)


if __name__ == '__main__':
    main()