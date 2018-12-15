#!/usr/bin/python

from vnpy import *
import pandas as pd
import numpy as np
from scipy.stats import linregress
import pickle
import time
from multiprocessing import Process, Queue
import scipy.interpolate as interp

import RotMatrixes

NUM_OF_PACKETS_LIDAR = 754  # number of lidar packets per 1 sec
NUM_OF_PACKETS_IMU = 20     # number of IMU packets per 1 sec
COLUMNS_IMU = 7             # number of IMU data columns


def imu_interpolate(imu_data1, imu_data2, lidar_time):  # 1x7, 1x7, 1xlen(lidar_time)) || LIDAR_XYZT[3]
    interp_imu_data = pd.DataFrame(np.zeros((len(lidar_time), 6)))
    # if len(imu_data1)!=len(imu_data2):
    #     print("Error IMU data-sets length don't match!")
    #     return
    imu_data = pd.DataFrame([imu_data1[0:6], imu_data2[0:6]])  # 2x6, without time values
    time_vec = [imu_data1[6], imu_data2[6]]
    imu_data_slope = imu_data.apply(lambda y: linregress(time_vec, y).slope, axis=0)
    imu_data_intercept = imu_data.apply(lambda y: linregress(time_vec, y).intercept, axis=0)
    linear_df = pd.DataFrame([imu_data_slope, imu_data_intercept])
    print(linear_df)
    for i, t in enumerate(lidar_time):
        interp_imu_data.iloc[i, :] = linear_df.apply(lambda n: n[0]*t+n[1], axis=0)

    return interp_imu_data

def calc_corrected_xyz(interp_imu_data, lidar_xyz):  # both input are arrange so each row corresponds to the same time.
    corrected_xyz = pd.DataFrame(np.zeros((len(lidar_xyz), 3)))
    print(interp_imu_data)
    exit()

    for i,_ in enumerate(lidar_xyz):
        corrected_xyz.iloc[i, :] = interp_imu_data.apply(lambda a: a[0] , axis=1)

    print(corrected_xyz)
    exit()



def IMU_connect():
    sensor = VnSensor()
    print('Establishing Connection with IMU...')
    sensor.connect('/dev/ttyUSB0', 921600)  # Default = 115200
    #self.s.change_baudrate(921600) # Used to change baud rate (number of bytes/per sec in communication channel)
    print('Connection Established')
    return sensor

class vn200_online_feeder:
    def __init__(self, imu_pkl):
        self.sensor = IMU_connect()
        self.IMU_PKL = imu_pkl
        self.run()

    def receiver(self, queue):
        with open(self.IMU_PKL, 'wb') as imu_f:
            while True:
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
                queue.put(lla_data)
                pickle.dump(lla_data, imu_f)

    def run(self):
        self.queue = Queue()
        self.proc = Process(target=self.receiver, args=(self.queue,),name='IMU_proc')
        self.proc.start()

    def get_packet(self):
        return self.queue.get()

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

    def get_packet(self):
        return self.imu_list.pop(0)


    # def get_frame(self, lidar_timestamp):
    #     imu_frame = []
    #     imu_packet = self.imu_list.pop(0)
    #     if imu_packet[COLUMNS_IMU-1]<lidar_timestamp:
    #         imu_packet = self.imu_list.pop(0)
    #     else:
    #         for i in range(NUM_OF_PACKETS_IMU):
    #             imu_frame = np.append(imu_frame, imu_packet)
    #             imu_packet = self.imu_list.pop(0)
    #     return imu_frame


        # self.imu_data = np.reshape(self.imu_data,(NUM_OF_PACKETS_IMU,COLUMNS_IMU))
        # imu_df = pd.DataFrame(self.imu_data, columns=['Yaw', 'Pitch', 'Roll', 'Lat', 'Long', 'Alt', 'Time'])
        # imu_full_frame = pd.DataFrame(interpolate_imu_frame(imu_df), columns=['Yaw', 'Pitch', 'Roll', 'Lat', 'Long', 'Alt', 'Time'])
        # imu_full_frame.to_csv(IMU_CSV,index=False,mode='w',header=False)


def main():
    # imu_interpolate([1, 2, 3, 4, 5, 6, 0], [2, 3, 4, 5, 6, 7, 1], [-1, 1, 100])
    calc_corrected_xyz(imu_interpolate([1, 2, 3, 4, 5, 6, 0], [2, 3, 4, 5, 6, 7, 1], [-100, -1, 1, 100, 1001]), [[1, 1, 1], [2, 2, 2]])
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