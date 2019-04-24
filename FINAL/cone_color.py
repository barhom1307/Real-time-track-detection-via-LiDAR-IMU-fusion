import numpy as np
import pickle
import pandas as pd
import matplotlib.pyplot as plt
import warnings
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib.gridspec import GridSpec
from utilities import set_aspect_equal_3d



def get_frame_cones():
    with open('/home/avinoam/Desktop/cones_13_04_19.pkl', 'rb') as cone_pkl:
        cone_data = pickle.load(cone_pkl)
    cones = [pd.DataFrame(f) for f in cone_data]
    return cones

if __name__ == '__main__':
    cones = get_frame_cones()
    print(cones[0])

    warnings.filterwarnings("ignore",".*GUI is implemented.*")
    x = np.linspace(1,10,10)
    y = np.linspace(1,10,10)

    fig = plt.figure(1)
    fig.add_subplot(2,1,1)
    plt.scatter(cones[0].iloc[:,0] , cones[0].iloc[:,1], c='r', s=50)
    fig.add_subplot(2,1,2)
    plt.scatter(cones[0].iloc[:,0] , cones[0].iloc[:,1], c='r', s=50)
    # ax = Axes3D(fig)
    # ax2 = fig2.add_subplot(111)
    # ax2.scatter(cones[0].iloc[:,0],cones[0].iloc[:,1], s=50, c='r')
    # ax.scatter(cones[0].iloc[:,0],cones[0].iloc[:,1],cones[0].iloc[:,6], s=50, c='b')

    plt.tight_layout()
    plt.show()
    # gs = GridSpec(2, 1)
    # gs.update(wspace=0.1, hspace=0.1, left=0.1, right=0.1, bottom=0.1, top=0.1)
    #
    # ax1 = plt.subplot2grid((2, 1), (1, 0), colspan=4, rowspan=1)
    # ax1.scatter(cones[0].iloc[:,0] , cones[0].iloc[:,1], c='r', s=50)
    # ax1.set_xlim([-3, 3])
    # ax1.set_ylim([0, 10])
    # ax1.set_title('Cones- 2D', fontsize=20)
    # ax1.set_xlabel('X [m]')
    # ax1.set_ylabel('Y [m]')
    #
    # ax4 = plt.subplot2grid((2, 1), (0, 0), colspan=12, rowspan=2, projection='3d')
    # plt.subplots_adjust(wspace=0, hspace=0)
    # ax4.clear()
    # try:
    #     ax4.scatter(cones[0].iloc[:,0], cones[0].iloc[:,1], cones[0].iloc[:,6], s=50, c='b')
    # except AttributeError as e:
    #     pass
    #
    # plt.gca().invert_xaxis()
    # plt.gca().invert_yaxis()
    # ax4.set_xlim3d([-3, 3])
    # ax4.set_ylim3d([0, 10])
    # # ax4.set_zlim3d([-1, 1])
    # ax4.set_title('Cones reflectivity', fontsize=20)
    # ax4.set_xlabel('{} axis'.format('X'))
    # ax4.set_ylabel('{} axis'.format('Y'))
    # ax4.set_zlabel('{} axis'.format('Ref'))
    # ax4.view_init(20, 80)
    # set_aspect_equal_3d(ax4)
    #
    # plt.show()
    # plt.pause(5)
    # plt.clf()
    #
    # # plt.ion()
    # # fig = plt.figure(1, figsize=[20,10])
    # # fig2 = plt.figure(2, figsize=[20,10])
    # # warnings.filterwarnings("ignore",".*GUI is implemented.*")
    # # # ax = fig.add_subplot(111, projection = '3d')
    # # ax = Axes3D(fig)
    # # ax2 = fig2.add_subplot(111)
    # # ax2.scatter(cones[0].iloc[:,0],cones[0].iloc[:,1], s=50, c='r')
    # # ax.scatter(cones[0].iloc[:,0],cones[0].iloc[:,1],cones[0].iloc[:,6], s=50, c='b')
    # # # set_aspect_equal_3d(ax)
    # # ax.set_xlim3d((-2.5, 2.5))
    # # ax.set_ylim3d((0, 10))
    # # # ax.set_zlim3d((-0.5,0.3))
    # # plt.gca().invert_xaxis()
    # # plt.gca().invert_yaxis()
    # # ax.view_init(20, 80)
    # # plt.title('Lidar view', loc='center')
    # # plt.xlabel('X [m]')
    # # plt.ylabel('Y [m]')
    # # plt.draw()
    # # plt.pause(15)
    # # plt.clf()
