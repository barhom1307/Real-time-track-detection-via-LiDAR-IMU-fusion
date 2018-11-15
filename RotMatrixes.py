#!/usr/bin/python
import numpy as np
EARTH_CIRCUMFERENCE = 40007860


def Offset(LAT_DEG, LON_DEG):
    METERS_PER_LAT = EARTH_CIRCUMFERENCE/360
    METERS_PER_LON = (EARTH_CIRCUMFERENCE*np.cos(LAT_DEG*(np.pi/180)))/360
    return METERS_PER_LAT*LAT_DEG, METERS_PER_LON*LON_DEG, METERS_PER_LAT, METERS_PER_LON


def Deg2Meters(IMU_LL_DEG, LAT_OFFSET_METERS, LON_OFFSET_METERS, METERS_PER_LAT, METERS_PER_LON):
    return \
        (IMU_LL_DEG[0, 0]*METERS_PER_LAT) - LAT_OFFSET_METERS, \
        (IMU_LL_DEG[0, 1]*METERS_PER_LON) - LON_OFFSET_METERS, 0  # Returns distance in meters from starting point.


def Imu_Lidar_Offset():
    x = []
    y = []
    z = []
    return x, y, z


def Rot1(lidCoords):
    R_L2IMU = np.mat([[0, 1, 0], [1, 0, 0],  [0, 0, -1]])
    return lidCoords.dot(R_L2IMU)


def Rot2(lidarCoordsIMU, imu_coords):
    yaw = imu_coords[0, 0]
    pitch = imu_coords[0, 1]
    roll = imu_coords[0, 2]
    # lon = imu_coords[0, 6]
    # lat = imu_coords[0 , 7]
    R_Mat_Yaw = np.mat([[np.cos(yaw * (np.pi / 180)), np.sin(yaw * (np.pi / 180)), 0],
                        [-np.sin(yaw * (np.pi / 180)), np.cos(yaw * (np.pi / 180)), 0],
                        [0, 0, 1]])
    R_Mat_Pitch = np.mat([[np.cos(pitch * (np.pi / 180)), 0, -np.sin(pitch * (np.pi / 180))],
                          [0, 1, 0],
                          [np.sin(pitch * (np.pi / 180)), 0, np.cos(pitch * (np.pi / 180))]])
    R_Mat_Roll = np.mat([[1, 0, 0],
                         [0, np.cos(roll * (np.pi / 180)), np.sin(roll * (np.pi / 180))],
                         [0, -np.sin(roll * (np.pi / 180)), np.cos(roll * (np.pi / 180))]])
    R_L2YPR = R_Mat_Yaw.dot(R_Mat_Pitch.dot(R_Mat_Roll))

    # From LLA Frame of Reference to Earth XYZ Frame of Reference:
    # R_L2LL = np.mat([[-np.sin(lat) , -np.sin(lon) * np.cos(lat) , np.cos(lon) * np.cos(lat)]
    #                    , [np.cos(lat) , -np.sin(lon) * np.sin(lat) , np.cos(lon) * np.sin(lat)]
    #                    , [0 , np.cos(lon) ,np.sin(lon)]])
    # return R_L2LL.dot(R_L2YPR.dot(lidarCoordsIMU.T)).T

    return lidarCoordsIMU.dot(R_L2YPR.T)


def RotTot(IMU_YPR, IMU_LL_DEG, LIDAR_XYZ, OFFSET):
    LIDAR_XYZ1 = Rot1(LIDAR_XYZ)
    LIDAR_XYZ2 = Rot2(LIDAR_XYZ1, IMU_YPR)
    IMU_LL_METERS = Deg2Meters(IMU_LL_DEG, LAT_OFFSET_METERS=OFFSET[0], LON_OFFSET_METERS=OFFSET[1],
                               METERS_PER_LAT=OFFSET[2], METERS_PER_LON=OFFSET[3])
    return LIDAR_XYZ2 + IMU_LL_METERS

