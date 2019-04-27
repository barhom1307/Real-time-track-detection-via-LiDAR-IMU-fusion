#!/usr/bin/python

# General system related libraries
import time
import pytictoc
import matplotlib.pyplot as plt
from matplotlib import style
style.use('fivethirtyeight')
import os
import warnings

# Calculation and data manipulation libraries:
import numpy as np
import pandas as pd
pd.set_option('float_format','{:f}'.format)

# Project modules
from Track_calculations import cone_finder
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


# files names - for online and offline (uncomment as you need)
# online_mode (new files will be created):
# IMU_PKL = PATH + 'imu_rec_' + TEST_NAME + '.pkl'
# LIDAR_PKL = PATH + 'lidar_rec_' + TEST_NAME + '.pkl'
# offline_mode (files on your computer):
LIDAR_PKL = PATH + 'lidar_rec_16:09:36_13.04.19.pkl'
IMU_PKL = PATH + 'imu_rec_16:09:36_13.04.19.pkl'


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
    timeout = int(input('Please enter a desired recording time (in seconds): '))  # seconds
    feeder = joint_feeder()
    lidar_decoder = LIDAR.vlp16_decoder(feeder.lidar_feeder)
    imu_decoder = IMU.vn200_decoder(feeder.imu_feeder)

    try:
        if WORK_MODE: # if online than we need to send data to server - creating client object on car
            client = Server_Client.Client()

        N_frames = timeout * 10    # 10 frames per sec (lidar working freq is 10Hz)
        t = pytictoc.TicToc()
        time_val = 0

        if PLOT:
            plt.ion()
            fig = plt.figure(1)
            warnings.filterwarnings("ignore",".*GUI is implemented.*")
            mng = plt.get_current_fig_manager()
            mng.full_screen_toggle()

        for frame in range(N_frames):
            t.tic()
            # utilities.print_progress(packet, N_frames-1)
            decoded_frame, timestamp_list = lidar_decoder.get_full_frame()
            xyz_time_cones , fov_frame, cones_reflectivity = cone_finder(decoded_frame, timestamp_list, FOV)
            if np.all(xyz_time_cones) == None:
                continue
            xyzCones_world, ypr, y_car, x_car = imu_decoder.get_world_coords(xyz_time_cones)

            # need to choose to uncomment 1 of 3 options of plotting below
            if PLOT:
                # utilities.plot_3d(fig, xyz_time_cones)
                utilities.plot_2d(xyz_time_cones)
                # lidar_decoder.plot_lidar_cones(fig, fov_frame, xyz_time_cones, frame_time)

            if WORK_MODE: # if online than we need to send data to server - creating client object on car
                data = xyzCones_world.iloc[:,0:2].append([[x_car,y_car],[ypr[0],ypr[1]],[ypr[2],0]])
                data = np.array(data.values)
                data = data.copy(order = 'C')
                client.send_packet(data)

            time_val += t.tocvalue()

    except KeyboardInterrupt or SystemError or OSError or TypeError or ValueError:
        feeder.close_joint_feeder(N_frames)


    print('Average frame\'s decoding time: ' + str(round((time_val/N_frames),3)) + ' sec')
    feeder.close_joint_feeder(N_frames)


if __name__ == '__main__':
    main()
