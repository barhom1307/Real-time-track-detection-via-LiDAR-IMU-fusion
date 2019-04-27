#!/usr/bin/python

import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.cm as cm
from scipy.stats import multivariate_normal
import warnings


class nf(float):
    def __repr__(self):
        str = '%.1f' % (self.__float__(),)
        if str[-1] == '0':
            return '%.0f' % self.__float__()
        else:
            return '%.1f' % self.__float__()


def route_calc(temp_delta, cone_vec, plot_type, fig):

    x = np.linspace(-4,4,300)
    y = np.linspace(0,10,300)
    X , Y = np.meshgrid(x,y)
    cone_vec = cone_vec[:,[0,1]]
    sigma = np.array([0.38, 1])
    covariance = np.diag(sigma**2)
    amp = 1000/(sigma[0]*sigma[1]*3.14)
    xy = np.column_stack([X.flat, Y.flat])
    z_tot = np.sum([amp*multivariate_normal.pdf(xy, mean=cone_vec[i], cov=covariance).reshape(X.shape) for i in range(len(cone_vec))], axis=0)
    z_tot += np.sqrt(pow(3*X,2) + pow(3*Y,2))

    if (plot_type == 1):
        ax = fig.gca(projection='3d')
        # Plot the surface.
        surf = ax.plot_surface(X, Y, z_tot, cmap=cm.coolwarm, linewidth=0.005, antialiased=False)
        # Customize the z axis.
        ax.set_zlim(0, 100)
        ax.zaxis.set_major_locator(LinearLocator(10))
        ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.8, aspect=10)
        plt.draw()
        plt.pause(0.001)
        plt.clf()
    
    if (plot_type == 2):
        ax = fig.gca()
        CS = ax.contour(X, Y,z_tot)
        # Recast levels to new class
        CS.levels = [nf(val) for val in CS.levels]
        # Label levels with specially formatted floats
        if plt.rcParams["text.usetex"]:
            fmt = r'%r \%%'
        else:
            fmt = '%r %%'
        ax.clabel(CS, CS.levels, inline=True, fmt=fmt, fontsize=10)


    level = 15
    indy, indx = np.where(z_tot < level)

    if (len(indy) != 0):
        max_alloc = np.argwhere(indy == max(indy))
        # find the steering value
        alpha = [math.atan2(x[indx[max_alloc[i]]],y[indy[max_alloc[i]]]) for i in range(len(max_alloc))]
        Lp = [np.sqrt(pow(x[indx[max_alloc[i]]],2) + pow(y[indy[max_alloc[i]]],2)) for i in range(len(max_alloc))]
        R = [Lp[i]/(2*math.sin(alpha[i])) for i in range(len(alpha))]

        #delta = [1.525/R[i]*180/3.14 for i in range(len(R))]
        delta_st = [1.525/R[int(len(R)/2)]*(180/3.14)]
        delta_st = np.int(delta_st[0])

        if(delta_st < -25):
            delta_st = -25
        elif(delta_st > 25):
            delta_st = 25

        if(np.abs(delta_st-temp_delta) > 30):
            delta_st = temp_delta
        else:
            temp_delta = delta_st
    else:
        delta_st = temp_delta


    if(plot_type == 2):
        deg = np.linspace(0,360,721)
        sind = np.sin(list(np.deg2rad(i) for i in deg))
        cosd = np.cos(list(np.deg2rad(i) for i in deg))
        x_r = R[int(len(R)/2)]*cosd + np.sign(alpha[int(len(R)/2)])*np.abs(R[int(len(R)/2)])
        y_r = R[int(len(R)/2)]*sind
        plt.axis([-10, 10 ,0 ,20])
        plt.plot(x_r,y_r,'g-',linewidth=3.0)
        plt.plot(x[indx],y[indy],'b*')
        plt.draw()
        plt.pause(5)
        plt.clf()
    
    return delta_st, temp_delta


if __name__ == '__main__':
    # cone_vec for example:
    cone_vec = np.array([[-2.5,2],[2.5,2],[2.7,11],[-2.6,10],[-4,4],[-4,5]])
    # plot type- 0 for NO plot, 1 for surf plot, 2 for contour plot
    PLOT = 2
    temp_delta = 0
    figure = 0
    if PLOT:
        plt.ion()
        figure = plt.figure()
        warnings.filterwarnings("ignore",".*GUI is implemented.*")
    delta_st, new_temp_delta = route_calc(temp_delta, cone_vec, plot_type = PLOT, fig=figure)
    print('Delta = {:d}'.format(delta_st))
    print('Temp = {:d}'.format(new_temp_delta))
