#!/usr/bin/python

# coding: utf-8
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import pandas as pd
from matplotlib import style
import LIDAR
from pytictoc import TicToc
t = TicToc()

import Track_calculations
# Print iterations progress
def print_progress(iteration, total):
    """
    Call in a loop to create terminal progress bar
    Parameters
    ----------
    iteration :
                Current iteration (Int)
    total     :
                Total iterations (Int)
    """
    str_format = "{0:.0f}"
    percents = str_format.format(100 * (iteration / float(total)))
    filled_length = int(round(100 * iteration / float(total)))
    bar = '█' * filled_length + '-' * (100 - filled_length)

    sys.stdout.write('\r |%s| %s%%' % (bar, percents)),

    if iteration == total:
        sys.stdout.write('\n')
sys.stdout.flush()


def set_aspect_equal_3d(ax):
    """Fix equal aspect bug for 3D plots."""

    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()

    from numpy import mean
    xmean = mean(xlim)
    ymean = mean(ylim)
    zmean = mean(zlim)

    plot_radius = max([abs(lim - mean_)
                       for lims, mean_ in ((xlim, xmean),
                                           (ylim, ymean),
                                           (zlim, zmean))
                       for lim in lims])

    ax.set_xlim3d([xmean - plot_radius, xmean + plot_radius])
    ax.set_ylim3d([ymean - plot_radius, ymean + plot_radius])
    ax.set_zlim3d([zmean - plot_radius, zmean + plot_radius])



def plot_3d_live(i, fig, lidar_decoder, imu_decoder):

    t_total =0

    # for i in range(15):
    #     frame, time_stamp = lidar_decoder.get_full_frame()


    frame, time_stamp = lidar_decoder.get_full_frame()


    xyzt, _ = Track_calculations.cone_finder(frame, time_stamp, 60)
    velo_frame, ypr = imu_decoder.get_world_coords(xyzt)
    yaw = ypr[0] - imu_decoder.yaw0
    pitch = ypr[1] - imu_decoder.pitch0
    roll = ypr[2] - imu_decoder.roll0
    # t.toc()

    # TODO: Rotation Matrixes working, need to insert worrld_corrds() and use through it.
    # xyzt, _ = Track_calculations.cone_finder(frame, time_stamp, 60)  # Just filters distance for testing
    # xyzt = pd.DataFrame(xyzt)  # turn to pandas dataframe
    # xyz_imu_FoR = xyzt.iloc[:, 0:3].dot(np.mat([[0, 1, 0], [1, 0, 0],  [0, 0, -1]]))  # Change to IMU Field of Ref. for yaw-pitch-roll corrections
    #
    # imut1, _ = imu_decoder.get_imu_frame(time_stamp[35])  # Take 1 time stamp from middle of frame for speed
    # # first angle corrections
    # yaw = (imut1[0] - yaw0)
    # pitch = (imut1[1] - pitch0)
    # roll = (imut1[2] - roll0)
    #
    # frame_rot_mat = imu_decoder.rotation_matrix([yaw, pitch, roll])  # 1 matrix for the whoile frame
    # xyz_imu_FoR = xyz_imu_FoR.dot(frame_rot_mat)  # xyz after YPR corrections
    # velo_frame = xyz_imu_FoR.dot(np.mat([[1, 0, 0], [0, -1, 0],  [0, 0, -1]]))  # Return to LIDAR Frame of Ref. for plotting


    gs = GridSpec(3, 12)  # 2 rows, 6 columns
    gs.update(wspace=0.1, hspace=0.1, left=0.1, right=0.1, bottom=0.1, top=0.1)

    ax1 = plt.subplot2grid((3, 12), (2, 0), colspan=4, rowspan=1)
    ax1.scatter(t_total, yaw, c='r', s=50)
    ax1.set_xlim([-1, 1])
    ax1.set_ylim([-90, 90])
    ax1.set_title('Yaw', fontsize=20)
    ax1.set_xlabel('Time [sec]')
    ax1.set_ylabel('Angle [deg]')

    ax2 = plt.subplot2grid((3, 12), (2, 4), colspan=4, rowspan=1)
    ax2.scatter(t_total, pitch, c='r', s=50)
    ax2.set_xlim([-1, 1])
    ax2.set_ylim([-90, 90])
    ax2.set_title('Pitch', fontsize=20)
    ax2.set_xlabel('Time [sec]')
    ax2.set_ylabel('Angle [deg]')

    ax3 = plt.subplot2grid((3, 12), (2, 8), colspan=4, rowspan=1)
    ax3.scatter(t_total, roll, c='r', s=50)
    ax3.set_xlim([-1, 1])
    ax3.set_ylim([-90, 90])
    ax3.set_title('Roll', fontsize=20)
    ax3.set_xlabel('Time [sec]')
    ax3.set_ylabel('Angle [deg]')

    ax4 = plt.subplot2grid((3, 12), (0, 0), colspan=12, rowspan=2, projection='3d')
    plt.subplots_adjust(wspace=0, hspace=0)
    ax4.clear()
    try:
        ax4.scatter(np.array(velo_frame.iloc[:, 0]), np.array(velo_frame.iloc[:, 1]), np.array(velo_frame.iloc[:, 2]), s=25, cmap='gray')
    except AttributeError as e:
        pass

    ax4.set_xlim3d([-1, 1])
    ax4.set_ylim3d([-1, 1])
    ax4.set_zlim3d([-1, 1])
    ax4.set_title('LIDAR', fontsize=20)
    ax4.set_xlabel('{} axis'.format('X'))
    ax4.set_ylabel('{} axis'.format('Y'))
    ax4.set_zlabel('{} axis'.format('Z'))
    ax4.view_init(0, 180)
    set_aspect_equal_3d(ax4)


