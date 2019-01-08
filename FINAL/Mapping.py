from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import warnings
from scipy.stats import multivariate_normal
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.cm as cm
import pandas as pd
import cv2
import os

# Globals
IDX = 1
IMAGE_PATH = "/home/avinoam/Desktop/images"
SAVE_IMAGES_FLAG = True


def plot_map_state(fig, cone_vec, old_Z):
    x = np.linspace(-30,30,300)    # each block will be 20cm wide
    y = np.linspace(-20,20,200)    # each block will be 20cm wide
    X, Y = np.meshgrid(x,y)
    sigma = np.array([1, 1])
    covariance = np.diag(sigma**2)
    # covariance = np.array([[.5, .25],[.25, .5]])
    amp = 4/(sigma[0]*sigma[1]*np.pi)

    xy = np.column_stack([X.flat, Y.flat])
    Z = old_Z + np.sum([amp*multivariate_normal.pdf(xy, mean=np.array(cone_vec[i]), cov=covariance).reshape(X.shape) for i in range(len(cone_vec))], axis=0)
    # Z = old_Z + np.sum([multivariate_normal.pdf(xy, mean=np.array(cone_vec[i]), cov=covariance).reshape(X.shape) for i in range(len(cone_vec))], axis=0)

    ax = fig.gca(projection='3d')
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()

    # Plot the surface.
    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm, linewidth=0.1)
    #  (elev,azim) ‘elev’ stores the elevation angle in the z plane. ‘azim’ stores the azimuth angle in the x,y plane
    ax.view_init(55, 60)
    plt.title('Cones Probability Map \n Frame number-{}'.format(IDX), {'fontsize': 20}, loc='center')
    plt.xlabel('x-axis (right / left)')
    plt.ylabel('y-axis (forward / backward)')

    # Customize the z-axis.
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.9, aspect=10)
    ax.set_aspect('equal')
    plt.draw()

    # save image to specific directory
    if SAVE_IMAGES_FLAG:
        imageFile = IMAGE_PATH + "/image_{}.png".format(IDX)
        plt.savefig(imageFile)

    # clear plot
    plt.pause(0.5)
    plt.clf()

    return Z


def createMovie():
    video_name = IMAGE_PATH + '/mapping.mp4'

    images = sorted([img for img in os.listdir(IMAGE_PATH) if img.endswith(".png")])
    frame = cv2.imread(os.path.join(IMAGE_PATH, images[0]))
    height, width, layers = frame.shape

    # video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*"MJPG"), 1, (width,height))
    video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*"mp4v"), 0.5, (width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(IMAGE_PATH, image)))

    cv2.destroyAllWindows()
    video.release()


if __name__ == '__main__':
    plt.ion()
    temp_delta = 0
    fig = plt.figure(figsize=(20,20))
    warnings.filterwarnings("ignore",".*GUI is implemented.*")
    cone_vec = pd.read_csv('/home/avinoam/Desktop/map_test.csv').values
    cone_vec = cone_vec.tolist()
    cone_vec = np.array_split(cone_vec,6)
    Z = 0
    for cones in cone_vec:
        Z = plot_map_state(fig, cones, Z)
        IDX += 1

    createMovie()
