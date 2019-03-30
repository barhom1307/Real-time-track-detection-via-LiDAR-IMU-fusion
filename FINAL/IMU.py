#!/usr/bin/python

from vnpy import *
import pandas as pd
import numpy as np
from scipy.stats import linregress
import pickle
import time
from multiprocessing import Process, Queue
from signal import signal, SIGTERM
import os
from matplotlib import style
from pytictoc import TicToc
from orderedset import OrderedSet
pd.set_option('float_format', '{:f}'.format)
style.use('fivethirtyeight')
timer = TicToc()

'''User Parameters'''
COLUMNS_IMU = 7             # number of IMU data columns
FLAG = True


def handler(signum, frame):
    global FLAG
    FLAG = False

'''   
Abstract: Reads data live from IMU sensor

Parameters: full path to pickle (for option to save data)

Returns:
'''
class vn200_online_feeder:
    def __init__(self, imu_pkl):
        self.IMU_PKL = imu_pkl
        self.run()

    def receiver(self, queue):
        sensor = VnSensor()
        print('Establishing Connection with IMU...')
        sensor.connect('/dev/ttyUSB0', 115200)  # Default = 115200, fastest 921600
        # sensor.change_baudrate(921600)
        print('Connection Established, ' + 'Baudrate is: ' + str(sensor.read_serial_baudrate()))
        signal(SIGTERM, handler)
        self.imu_packets = []
        while FLAG:
            lla = sensor.read_ins_solution_lla()
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
            self.imu_packets.append(lla_data)
        with open(self.IMU_PKL, 'wb') as imu_f:
            pickle.dump(self.imu_packets, imu_f)
        os._exit(0)

    def run(self):
        self.queue = Queue()
        self.proc = Process(target=self.receiver, args=(self.queue,),name='IMU_proc')
        self.proc.start()

    def get_packet(self):
        return self.queue.get()

    def close_imu(self):
        print('Terminating IMU Connection...')
        self.proc.terminate()


'''   
Abstract: Reads IMU data from csv file

Parameters: imu_pkl - full path to pickle file of the imu data. (pickle  = serialaized data)

Returns: 
'''


class vn200_offline_feeder:
    def __init__(self, imu_pkl):
        self.IMU_PKL = imu_pkl  # /...path.../../imu_rec_test.pkl
        imu_array = []
        print("Connecting to offline IMU feeder...")
        with open(self.IMU_PKL, 'rb') as imu_f:
            while True:
                try:
                    imu_packet = pickle.load(imu_f)
                    imu_array = np.append(imu_array, imu_packet)
                except EOFError:
                    break
        imu_array = np.reshape(imu_array, (int(len(imu_array)/COLUMNS_IMU), COLUMNS_IMU))
        self.imu_list = imu_array.tolist()

    def get_packet(self):
        return self.imu_list.pop(0)

'''
Abstract: Changes frame of refrence of lidar X-Y-Z
          to Earth LLA frame of refrence using IMU data, Y-P-R.
          
Parameters: X-Y-Z-t - data points filtered as cones by the cone_finder function

Returns: X-Y-Z in Earth frame of refrence
'''


class vn200_decoder:
    def __init__(self, packet_feeder):
        self.packet_feeder = packet_feeder
        imu_t1 = packet_feeder.get_packet()
        self.yaw0 = imu_t1[0]
        self.pitch0 = imu_t1[1]
        self.roll0 = imu_t1[2]


    '''     
    Abstract: Finds the IMU values within an error value (default is 0.1 sec) to the times in lidar frame

    Parameters: list of lidar of all packet times in lidar frame

    Returns: IMU values closest to packets time (with error, default 0.1 sec)
    '''
    def get_imu_frame(self, lidar_packet_time):
        imu_t1 = self.packet_feeder.get_packet()
        try:
            while abs(imu_t1[6] - lidar_packet_time[0]) > 0.1:  # TODO:Print and check times
                imu_t1 = self.packet_feeder.get_packet()
        except KeyError as e:
            pass
        imu_t2 = self.packet_feeder.get_packet()
        return [imu_t1, imu_t2]

    '''   
    Abstract: Uses linear regression to find the linear equeation of each imu data point. 
              Assumes change is linear around a small time delta. (For better precision different implementation maybe needed)

    Parameters: imu_data1 - closest data point before the lidar packet time point
                imu_data2 - closest data point after the lidar packet time point
                lidar_packet_time - time of all XYZ in this packet

    Returns: saves a 2x6 array with the slope (first row) and intercept (second row) for each IMU data, Y-P-R-L-L-A.
    '''
    def imu_interpolate(self, imu_data1, imu_data2, lidar_packet_time):  # 1x7, 1x7, 1xlen(lidar_time)) || LIDAR_XYZT[3]
        self.interp_imu_frame = pd.DataFrame(np.zeros((len(lidar_packet_time), 7)))
        imu_data = pd.DataFrame([imu_data1[0:6], imu_data2[0:6]])  # 2x6, without time values
        time_vec = [imu_data1[6], imu_data2[6]]

        imu_data_slope = imu_data.apply(lambda y: linregress(time_vec, y).slope, axis=0)
        # .apply passes each of the columns of imu_data (shape: [2x1]) to y.

        imu_data_intercept = imu_data.apply(lambda y: linregress(time_vec, y).intercept, axis=0)
        # .apply passes each of the columns of imu_data (shape: [2x1]) to y.
        linear_df = pd.DataFrame([imu_data_slope, imu_data_intercept])
        for i, t in enumerate(lidar_packet_time):
            self.interp_imu_frame.iloc[i, :] = linear_df.apply(lambda n: n[0] * t + n[1], axis=0)
        self.interp_imu_frame[6] = lidar_packet_time

    '''   
    Abstract: Creates 3 rotation matrixs 1 for each axis using the yaw, pitch and roll 
              from the IMU and combines them to 1 rotation matrix.

    Parameters: ypr - yaw, pitch, roll for the specific time we want.

    Returns: rotation matrix around all 3 axis
    '''
    def rotation_matrix(self, ypr):
        yaw = ypr[0]
        pitch = ypr[1]
        roll = ypr[2]

        R_Mat_Yaw = np.mat([[np.cos(yaw * (np.pi / 180)), np.sin(yaw * (np.pi / 180)), 0],
                            [-np.sin(yaw * (np.pi / 180)), np.cos(yaw * (np.pi / 180)), 0],
                            [0, 0, 1]])

        R_Mat_Pitch = np.mat([[np.cos(pitch * (np.pi / 180)), 0, -np.sin(pitch * (np.pi / 180))],
                              [0, 1, 0],
                              [np.sin(pitch * (np.pi / 180)), 0, np.cos(pitch * (np.pi / 180))]])

        R_Mat_Roll = np.mat([[1, 0, 0],
                             [0, np.cos(roll * (np.pi / 180)), np.sin(roll * (np.pi / 180))],
                             [0, -np.sin(roll * (np.pi / 180)), np.cos(roll * (np.pi / 180))]])

        return R_Mat_Yaw.dot(R_Mat_Pitch.dot(R_Mat_Roll))  # Returns matrix for LLA frame of reference

    '''   
    Abstract: The main function of the class, recieves lidar points in 
              lidar frame of refrence (FoR) and returns them in Earch LLA FoR

    Parameters: xyzt_cones - lidar points with time stamp after cone filtering

    Returns: xyz in earth LLA FoR
    '''
    def get_world_coords(self, xyzt_cones):  # both inputs are arranged so each row corresponds to the same time.
        xyzt_cones = pd.DataFrame(xyzt_cones)
        lidar_time_series = pd.Series(OrderedSet(xyzt_cones.iloc[:, 3]))
        xyz_cones_imu_coord_sys = xyzt_cones.iloc[:, 0:3].dot(np.mat([[0, 1, 0], [1, 0, 0],  [0, 0, -1]]))
        imu_t1, imu_t2 = vn200_decoder.get_imu_frame(self, lidar_packet_time=lidar_time_series)
        vn200_decoder.imu_interpolate(self, imu_t1, imu_t2, lidar_packet_time=lidar_time_series)
        ypr_db = self.interp_imu_frame.iloc[:, 0:3]
        ypr_rotation_mat = vn200_decoder.rotation_matrix(self, ypr_db.iloc[round(len(ypr_db)/2), :])
        xyz_cones_imu_coord_sys = xyz_cones_imu_coord_sys.dot(ypr_rotation_mat)
        # xyz_cones_lidar_coord_sys = xyz_cones_imu_coord_sys.dot(np.mat([[-1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        xyz_cones_lidar_coord_sys = xyz_cones_imu_coord_sys.dot(np.mat([[0, 1, 0], [1, 0, 0],  [0, 0, -1]]))
        ypr_middleOfFrame = ypr_db.iloc[round(len(ypr_db) / 2), :]
        return xyz_cones_lidar_coord_sys, ypr_middleOfFrame


def main():
    pass


if __name__ == '__main__':
    main()
