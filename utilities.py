#!/usr/bin/python

# coding: utf-8
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
from matplotlib import style
import cv2
import os
import time

'''
Abstract:   print_progress func prints iterations progress bars '█' - according to the length of the "work/loop" given
Parameters: iteration-  Current iteration (Int)
            total-  Total iterations (Int) - must be bigger than 2
Returns:    None.
'''
def print_progress(iteration, total):
    str_format = "{0:.0f}"
    percents = str_format.format(100 * (iteration / float(total)))
    filled_length = int(round(100 * iteration / float(total)))
    bar = '█' * filled_length + '-' * (100 - filled_length)
    sys.stdout.write('\r |%s| %s%%' % (bar, percents)),
    if iteration == total:
        sys.stdout.write('\n')
sys.stdout.flush()


'''
Abstract:   set_aspect_equal_3d func taking care of the equal resolution of any 3D plot given.
Parameters: ax- input 3D fig
Returns:    None.
'''
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


'''
Abstract:   plot_2d func plots a 2d image of the given input "image" - only XY axis.
Parameters: image- input data- must be np.ndarray with 2 columns, which describes 2 axis (X,Y)
Returns:    None.
'''
def plot_2d(image):
    plt.scatter(image[:,0],image[:,1], s=50, c='r', marker='^')
    plt.title('Lidar view', loc='center')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim((-2.5, 2.5))
    plt.ylim((0, 10))
    plt.draw()
    plt.pause(0.0001)
    plt.clf()


'''
Abstract:   plot_3d func plots a 3d image of the given input "image".
Parameters: fig - figure
            image- input data- must be np.ndarray with 3 columns, which describes 3 axis (X,Y,Z)
Returns:    None.
'''
def plot_3d(fig, image):
    ax = fig.add_subplot(111, projection = '3d')
    ax.scatter(image[:,0],image[:,1],image[:,2], s=50, c='b')
    set_aspect_equal_3d(ax)
    ax.set_xlim3d((-2.5, 2.5))
    ax.set_ylim3d((0, 10))
    ax.set_zlim3d((-0.5,0.3))
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()
    ax.view_init(20, 80)
    plt.title('Lidar view', loc='center')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.draw()
    plt.pause(5)
    plt.clf()


'''
Abstract:   createMovie func creates mp4 movie from all images which locates at IMAGE_PATH directory.
Parameters: img_path - the full path of the image directory
            fps - choose your preferred frame per second ratio
Returns:    None.
'''
def createMovie(img_path, fps):
    video_name = img_path + '/mapping.mp4'
    from natsort import natsorted
    images = natsorted([img for img in os.listdir(img_path) if img.endswith(".png")])
    print(images)
    frame = cv2.imread(os.path.join(img_path, images[0]))
    height, width, layers = frame.shape

    # video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*"MJPG"), 1, (width,height))
    video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(img_path, image)))

    cv2.destroyAllWindows()
    video.release()


'''
Abstract:   plot_telemetry func creates mp4 movie from all images which locates at IMAGE_PATH directory.
Parameters: img_path - the full path of the image directory
            fps - choose your preferred frame per second ratio
Returns:    None.
'''
def plot_telemetry(xy_cones, xy_car, ypr, frame_time):
    yaw = ypr[0]
    pitch = ypr[1]
    roll = ypr[2]
    ypr_center_time_axis = 0 # ypr X axis isn't changing
    gs = GridSpec(6, 15)  # 6 rows, 15 columns
    gs.update(wspace=0.1, hspace=0.1, left=0.1, right=0.1, bottom=0.1, top=0.1)

    cone_img = np.concatenate(xy_cones, axis=0)
    car_route_img = np.reshape(xy_car, (len(xy_car),2))

    curr_time = time.time()-frame_time

    # main cones positions & car's position on map
    plt.clf()
    ax0 = plt.subplot2grid((6, 15), (0, 7), colspan=9, rowspan=9)
    ax0.scatter(car_route_img[:,0], car_route_img[:,1], s=50, c='b', marker='o', label='Car\'s Route')
    ax0.scatter(cone_img[:,0],cone_img[:,1], s=100, c='r', marker='^', label='Cones Positions')
    ax0.set_xlabel('x-axis (right / left) [m]')
    ax0.set_ylabel('y-axis (forward / backward) [m]')
    ax0.set_title('Cones Clustering Map - K-means Algorithm \n Frame timeStamp-{} sec'.format(round(curr_time, 3)),
              {'fontsize': 20})
    ax0.legend(loc='best')

    # yaw plot
    ax1 = plt.subplot2grid((6, 15), (1, 0), rowspan=3, colspan=1)
    ax1.scatter(ypr_center_time_axis, yaw, c='g', s=200)
    ax1.set_xlim([-1, 1])
    ax1.set_ylim([-180, 180])
    ax1.set_title('Yaw \n {}'.format(round(yaw,2)))
    ax1.set_ylabel('Angle [deg]')

    # pitch plot
    ax2 = plt.subplot2grid((6, 15), (1, 2), rowspan=3, colspan=1)
    ax2.scatter(ypr_center_time_axis, pitch, c='g', s=200)
    ax2.set_xlim([-1, 1])
    ax2.set_ylim([-90, 90])
    ax2.set_title('Pitch \n {}'.format(round(pitch,2)))
    ax2.set_xlabel('Time [sec]')

    # roll plot
    ax3 = plt.subplot2grid((6, 15), (1, 4), rowspan=3, colspan=1)
    ax3.scatter(ypr_center_time_axis, roll, c='g', s=200)
    ax3.set_xlim([-1, 1])
    ax3.set_ylim([-90, 90])
    ax3.set_title('Roll \n {}'.format(round(roll,2)))


    # ax4 = plt.subplot2grid((6, 15), (3, 1), rowspan=2, colspan=2)
    # ax4.scatter(long, lat, c='r', s=50)
    # # ax4.set_xlim([-1, 1])
    # # ax4.set_ylim([-90, 90])
    # ax4.set_title('Lat')
    # ax4.set_xlabel('Time [sec]')
    # ax4.set_ylabel('Angle [deg]')

    # ax5 = plt.subplot2grid((6, 15), (3, 4), rowspan=2, colspan=2)
    # ax5.scatter(t_0, long, c='r', s=500)
    # ax5.set_xlim([-1, 1])
    # ax5.set_ylim([-90, 90])
    # ax5.set_title('Long')
    # ax5.set_xlabel('Time [sec]')
    # ax5.set_ylabel('Angle [deg]')

    plt.show()
    plt.pause(0.01)
