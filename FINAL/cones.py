import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.cm as cm
from pytictoc import TicToc
from scipy.stats import multivariate_normal
import warnings


def compute_Z(X,Y,cone_x,cone_y,len_x):
    sa = 0.6
    sb = 1
    A = 1000
    Z = np.zeros((len_x,len_x))
    for i in range(len(X)):
        for j in range(len(Y)):
            exp_val = -(pow((X[i,j]-cone_x),2)/pow(sa,2) + (pow((Y[i,j]-cone_y),2)/pow(sb,2)))
            Z[i,j] = (A/(2*np.pi*sa*sb))*np.exp(exp_val)/2
    return Z

class nf(float):
    def __repr__(self):
        str = '%.1f' % (self.__float__(),)
        if str[-1] == '0':
            return '%.0f' % self.__float__()
        else:
            return '%.1f' % self.__float__()

def route_calc(temp_delta, cone_vec, plot_type, calc_type, fig):

    x = np.linspace(-4,4,300)
    y = np.linspace(0,10,300)
    cone_x = cone_vec[:,0]
    cone_y = cone_vec[:,1]
    z_tot = np.zeros((len(x),len(x)))
    X , Y = np.meshgrid(x,y)

    if (calc_type == 0): # longer compute - using the function "compute_Z"
        for i in range(len(cone_x)):
            z_tot += compute_Z(X,Y,cone_x[i],cone_y[i],len(x))
        z_tot += np.sqrt(pow(3*X,2) + pow(3*Y,2))
    
    elif (calc_type == 1): # shorter compute -unsure if it's right - need to check with ron
        cone_vec = cone_vec[:,[0,1]]
        sigma = np.array([0.38, 1])
        covariance = np.diag(sigma**2)
        amp = 1000/(sigma[0]*sigma[1]*3.14)

        xy = np.column_stack([X.flat, Y.flat])
        z_tot = np.sum([amp*multivariate_normal.pdf(xy, mean=cone_vec[i], cov=covariance).reshape(X.shape) for i in range(len(cone_vec))], axis=0)
        
        z_tot += np.sqrt(pow(3*X,2) + pow(3*Y,2))

    PLOT = plot_type    # 0 for surf plot , 1 for contour plot

    if (PLOT == 1):
        ax = fig.gca(projection='3d')

        # Plot the surface.
        surf = ax.plot_surface(X, Y, z_tot, cmap=cm.coolwarm,
                               linewidth=0.005, antialiased=False)
        
        # Customize the z axis.
        ax.set_zlim(0, 100)
        ax.zaxis.set_major_locator(LinearLocator(10))
        ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
        
        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.8, aspect=10)

        #ax.set_aspect('equal')
        plt.draw()
        plt.pause(0.001)
        plt.clf()
    
    elif (PLOT == 2):
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
    
    #indy,indx = np.where(z_tot < level)   Change the order iny indx
    indy,indx = np.where(z_tot < level)

    if (len(indy) != 0):
        max_alloc = np.argwhere(indy == max(indy))

        # find the steering value
        alpha = [math.atan2(x[indx[max_alloc[i]]],y[indy[max_alloc[i]]]) for i in range(len(max_alloc))]

        #Lp = np.ravel(np.sqrt(pow(x[indx[max_alloc]],2) + pow(y[indy[max_alloc]],2)))
        Lp = [np.sqrt(pow(x[indx[max_alloc[i]]],2) + pow(y[indy[max_alloc[i]]],2)) for i in range(len(max_alloc))]

        R = [Lp[i]/(2*math.sin(alpha[i])) for i in range(len(alpha))]

        #delta = [1.525/R[i]*180/3.14 for i in range(len(R))]
        delta_st = [1.525/R[int(len(R)/2)]*(180/3.14)]
        delta_st = np.int(delta_st[0])

        if(delta_st < -25):
            delta_st = -25
        elif(delta_st > 25):
            delta_st = 25

        # print('Temp = {:d}'.format(temp_delta))
        if(np.abs(delta_st-temp_delta) > 30):
            delta_st = temp_delta

        else:
            temp_delta = delta_st

    else:
        delta_st = temp_delta

    if(PLOT == 2):
        deg = np.linspace(0,360,721)
        sind = np.sin(list(np.deg2rad(i) for i in deg))
        cosd = np.cos(list(np.deg2rad(i) for i in deg))
        x_r = R[int(len(R)/2)]*cosd + np.sign(alpha[int(len(R)/2)])*np.abs(R[int(len(R)/2)])
        y_r = R[int(len(R)/2)]*sind
        plt.axis([-10, 10 ,0 ,20])
        plt.plot(x_r,y_r,'g-',linewidth=3.0)
        plt.plot(x[indx],y[indy],'b*')

        #ax.set_aspect('equal')
        plt.draw()
        plt.pause(5)
        plt.clf()
    
    return delta_st, temp_delta

if __name__ == '__main__':
    cone_vec = np.array([[-2.5,2],[2.5,2],[2.7,11],[-2.6,10],[-4,4],[-4,5]])
    #plot type - 0 for NO plot, 1 for surf plot , 2 for contour plot
    #calc_type - 0 for original calc(long) , 1 for shorter calc
    plt.ion()
    temp_delta = 0
    figure = plt.figure()
    warnings.filterwarnings("ignore",".*GUI is implemented.*")
    delta_st, new_temp_delta = route_calc(temp_delta, cone_vec, plot_type = 2, calc_type = 1, fig=figure)
    print('Delta = {:d}'.format(delta_st))
    print('Temp = {:d}'.format(new_temp_delta))
