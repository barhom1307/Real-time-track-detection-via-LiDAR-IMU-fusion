#!/usr/bin/python

from vnpy import *
import pandas as pd
import numpy as np
import pickle
import time
from multiprocessing import Process, Queue
import scipy.interpolate as interp

NUM_OF_PACKETS_LIDAR = 754  # number of lidar packets per 1 sec
NUM_OF_PACKETS_IMU = 20     # number of IMU packets per 1 sec
COLUMNS_IMU = 7             # number of IMU data columns


def interpolate_imu_frame(imu_df):
    imu_full_frame = []
    for col in imu_df.columns:
        series = imu_df[col]
        start, end = series.index[0], series.index[-1]
        index = np.linspace(start, end, NUM_OF_PACKETS_LIDAR)
        spline = interp.InterpolatedUnivariateSpline(series.index, series.values)
        res = pd.Series(index=index, data=spline(index))
        imu_full_frame = np.append(imu_full_frame, res.values)
    imu_full_frame = np.reshape(imu_full_frame,(COLUMNS_IMU,NUM_OF_PACKETS_LIDAR)).T
    return imu_full_frame

def IMU_connect():
    sensor = VnSensor()
    print('Establishing Connection with IMU...')
    sensor.connect('/dev/ttyUSB0', 115200)  # Default = 115200
    #self.s.change_baudrate(921600) # Used to change baud rate (number of bytes/per sec in communication channel)
    print('Connection Established')
    return sensor

class vn200_online_feeder:
    def __init__(self, imu_pkl):
        self.sensor = IMU_connect()
        self.IMU_PKL = imu_pkl

    def get_packet(self):

        with open(self.IMU_PKL, 'wb') as imu_f:
            lla = self.sensor.read_ins_solution_lla()
            lla_data = [
                lla.yawPitchRoll.x,  # Yaw
                lla.yawPitchRoll.y,  # Pitch
                lla.yawPitchRoll.z,  # Roll
                lla.position.x,      # Lat
                lla.position.y,      # Long
                lla.position.z,      # Alt
                time.time()
            ]
            pickle.dump(lla_data, imu_f)
        return lla_data


    def close_imu(self):
        print('Terminating IMU Connection...')
        self.sensor.disconnect()


class vn200_offline_feeder:
    def __init__(self, imu_pkl):
        self.IMU_PKL = imu_pkl
        imu_array = []
        print("Connecting to offline IMU feeder...")
        with open(self.IMU_PKL,'rb') as imu_f:
            while True:
                try:
                    imu_packet = pickle.load(imu_f)
                    imu_array = np.append(imu_array, imu_packet)
                except EOFError:
                    break
        imu_array = np.reshape(imu_array,(int(len(imu_array)/COLUMNS_IMU),COLUMNS_IMU))
        self.imu_list = imu_array.tolist()

    def get_frame(self, lidar_timestamp):
        imu_frame = []
        imu_packet = self.imu_list.pop(0)
        if imu_packet[COLUMNS_IMU-1]<lidar_timestamp:
            imu_packet = self.imu_list.pop(0)
        else:
            for i in range(NUM_OF_PACKETS_IMU):
                imu_frame = np.append(imu_frame, imu_packet)
                imu_packet = self.imu_list.pop(0)
        return imu_frame


        # self.imu_data = np.reshape(self.imu_data,(NUM_OF_PACKETS_IMU,COLUMNS_IMU))
        # imu_df = pd.DataFrame(self.imu_data, columns=['Yaw', 'Pitch', 'Roll', 'Lat', 'Long', 'Alt', 'Time'])
        # imu_full_frame = pd.DataFrame(interpolate_imu_frame(imu_df), columns=['Yaw', 'Pitch', 'Roll', 'Lat', 'Long', 'Alt', 'Time'])
        # imu_full_frame.to_csv(IMU_CSV,index=False,mode='w',header=False)


def main():
    print('Running IMU script')
    # print(time.time())
    # if online:
    #     imu_online = vn200_online_feeder()
    #     for i in range(3):
    #         last_imu_packet, curr_imu_packet = imu_online.get_packet(1541619975.3525062)
    #         print(last_imu_packet[6], curr_imu_packet[6])
    # else:
    #     imu_offline = vn200_offline_feeder()
    #     imu_frame = imu_offline.get_frame(1541627089.6306312)
    #     print(imu_frame)


if __name__ == '__main__':
    main()