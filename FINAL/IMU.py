#!/usr/bin/python
import pickle as pkl
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
import matplotlib.pyplot as plt
import LIDAR
import Track_calculations

pd.set_option('float_format', '{:f}'.format)
style.use('fivethirtyeight')
timer = TicToc()

'''User Parameters'''
EARTH_CIRCUMFERENCE = 40007860
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
        self.METERS_PER_LAT = EARTH_CIRCUMFERENCE / 360
        self.METERS_PER_LON = (EARTH_CIRCUMFERENCE * np.cos(imu_t1[3] * (np.pi / 180))) / 360
        # imu_t1 = Lat and can be recaculate for each lat, but the difference is negligable. For speed we will use it as a constant.
        self.Offset(imu_t1[3], imu_t1[4])
        self.imu_ll_meters = pd.DataFrame()
        self.xyz_cones_lidar_coord_sys = pd.DataFrame()


    '''
    Abstract: Finds the IMU values within an error value (default is 0.1 sec) to the times in lidar frame

    Parameters: list of lidar of all packet times in lidar frame

    Returns:
    '''
    def get_imu_frame(self, lidar_packet_time):
        self.imu_t1 = self.packet_feeder.get_packet()
        try:
            while abs(self.imu_t1[6] - lidar_packet_time[round(len(lidar_packet_time)/2)]) > 0.1:  # TODO:Print and check times
                self.imu_t1 = self.packet_feeder.get_packet()
        except KeyError as e:
            pass
        self.imu_t2 = self.packet_feeder.get_packet()

    '''
    Abstract: Uses linear regression to find the linear equeation of each imu data point.
              Assumes change is linear around a small time delta. (For better precision different implementation maybe needed)

    Parameters: imu_data1 - closest data point before the lidar packet time point
                imu_data2 - closest data point after the lidar packet time point
                lidar_packet_time - time of all XYZ in this packet

    Returns: saves a 2x6 array with the slope (first row) and intercept (second row) for each IMU data, Y-P-R-L-L-A.
    '''
    def imu_interpolate(self, lidar_packet_time):  # 1x7, 1x7, 1xlen(lidar_time)) || LIDAR_XYZT[3]
        imu_data1=self.imu_t1
        imu_data2=self.imu_t2
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
        yaw = ypr[0] - self.yaw0
        pitch = ypr[1] - self.pitch0
        roll = ypr[2] - self.roll0

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

    Returns: xyz in earth LLA FoR, latitude car reference, longitude car reference
    '''
    def get_world_coords(self, xyzt_cones):  # both inputs are arranged so each row corresponds to the same time.
        xyzt_cones = pd.DataFrame(xyzt_cones)
        lidar_time_series = pd.Series(xyzt_cones.iloc[:, 3])

        # Change X-Y-Z to IMU frame of reference
        xyz_cones_imu_coord_sys = xyzt_cones.iloc[:, 0:3].dot(np.mat([[0, 1, 0], [1, 0, 0],  [0, 0, -1]]))

        # Get the imu points closest to the lidar time of mid frame
        vn200_decoder.get_imu_frame(self, lidar_packet_time=lidar_time_series)

        # Based on the 2 imu points we create a linear regression for each imu parameter and insert the lidar time vector.
        vn200_decoder.imu_interpolate(self, lidar_packet_time=lidar_time_series)

        # Create 1 rotation matrix based on imu values from the middle of the frame.
        # Saves alot of time instead of createing a different rotation matrixes for each time. Accuracy is good enough and speed is many times better.
        ypr_rotation_mat = vn200_decoder.rotation_matrix(self, ypr=self.interp_imu_frame.iloc[round(len(lidar_time_series)/2), 0:3])

        # Correct the LIDAR X-Y-Z using the IMU data Yaw-Pitch-Roll, euler angles (rotations along the major axis)
        xyz_cones_imu_coord_sys = xyz_cones_imu_coord_sys.dot(ypr_rotation_mat)

        # Return X-Y-Z to Lidar frame of reference for ploting-visualy more comfortable.
        self.xyz_cones_lidar_coord_sys = pd.DataFrame(xyz_cones_imu_coord_sys.dot(np.mat([[0, 1, 0], [1, 0, 0], [0, 0, -1]])))

        # We take into account that for small distances up 1 km^2 the surface is aprx. flat,
        # Thus longtitude and latitude degrees can be converted to meters.
        self.interp_imu_frame.iloc[:, 3] = (self.interp_imu_frame.iloc[:, 3] * self.METERS_PER_LAT) - self.LAT_OFFSET_METERS
        self.interp_imu_frame.iloc[:, 4] = (self.interp_imu_frame.iloc[:, 4] * self.METERS_PER_LON) - self.LON_OFFSET_METERS
        # We add to the X-Y columns of points that have been corrected using the Y-P-R the meter values from the long/lat deg.
        # (long = x, lat = y)
        # Thus we create a fixed world map, with all points of a given cone from different frames and packets being placed aprx. at the same spot.
        self.xyz_cones_lidar_coord_sys.iloc[:, 0] = self.xyz_cones_lidar_coord_sys.iloc[:, 0] + self.interp_imu_frame.iloc[:, 4]
        self.xyz_cones_lidar_coord_sys.iloc[:, 1] = self.xyz_cones_lidar_coord_sys.iloc[:, 1] + self.interp_imu_frame.iloc[:, 3]
        return self.xyz_cones_lidar_coord_sys, self.interp_imu_frame.iloc[:, 3][0], self.interp_imu_frame.iloc[:, 4][0]

    def Offset(self, LAT_DEG, LON_DEG):
        self.LAT_OFFSET_METERS = self.METERS_PER_LAT * LAT_DEG
        self.LON_OFFSET_METERS = self.METERS_PER_LON * LON_DEG


def main():
    imu = vn200_offline_feeder("imu_rec_16_09_36_13.04.19.pkl")
    lidar = LIDAR.vlp16_offline_feeder("lidar_rec_16_09_36_13.04.19.pkl")
    imu_decoder = vn200_decoder(imu)
    lidar_decoder = LIDAR.vlp16_decoder(lidar)
    xyzt, _ = Track_calculations.cone_finder(frame, time_stamp, 60)
    velo_frame, ypr = imu_decoder.get_world_coords(xyzt)

    # print(imu_decoder.get_imu_frame())
    with open("imu_rec_16_09_36_13.04.19.pkl", 'rb') as f:
        imu_raw = pd.DataFrame(pkl.load(f))
    # print(imu_raw.head())
    imu_ll= imu_raw.iloc[:, 3:5]
    # print(imu_ll.head())
    imu_decoder.Offset(imu_ll.iloc[0, 0], imu_ll.iloc[0, 1])
    imu_ll_meters = pd.DataFrame()
    # print(imu_ll.iloc[1,0], imu_ll.iloc[1,1])
    for i in range(len(imu_ll)):
        # print(imu_decoder.Deg2Meters(imu_ll.iloc[i, 0], imu_ll.iloc[i, 1]))
        imu_ll_meters = imu_ll_meters.append(([imu_decoder.Deg2Meters(imu_ll.iloc[i, 0], imu_ll.iloc[i, 1])]))
    # print(imu_ll_meters.tail())

    plt.scatter(imu_ll_meters.iloc[0:1200, 0], imu_ll_meters.iloc[0:1200, 1])
    plt.xlim(min(imu_ll_meters.iloc[0:1200,0])-1, max(imu_ll_meters.iloc[0:1200,0])+1)
    plt.ylim(min(imu_ll_meters.iloc[0:1200,1])-1, max(imu_ll_meters.iloc[0:1200,1])+1)
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()
    plt.show()


if __name__ == '__main__':
    main()