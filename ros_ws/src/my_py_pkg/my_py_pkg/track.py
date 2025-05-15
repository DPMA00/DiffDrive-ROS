
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev
import pandas as pd
from scipy.interpolate import CubicSpline
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from scipy.spatial import KDTree

class Track:
    def __init__(self, data='/home/dpma/projects/DiffDrive-ROS/ros_ws/src/my_py_pkg/my_py_pkg/track_points.csv'):
        self.df = pd.read_csv(data, delimiter=',')
        xy = self.df[['x', 'y']].values
        x = xy[:, 0]
        y = xy[:, 1]
        tck, u = splprep(xy.T, u=None, s=0.0, per=1)


        u_new = np.linspace(u.min(), u.max(), 1000)
        self.x_new, self.y_new = splev(u_new, tck, der=0)

        dx = np.diff(self.x_new)
        dy = np.diff(self.y_new)

        s = np.cumsum(np.sqrt(dx**2+dy**2))
        s = np.insert(s,0,0)
        self.L = s[-1]


        self.cs_x = CubicSpline(s, self.x_new, bc_type='periodic')
        self.cs_y = CubicSpline(s, self.y_new, bc_type='periodic')



        self.x_vec, self.y_vec,self.theta_vec = self.create_grid(density=5000)
        pts = np.column_stack((self.x_vec, self.y_vec))
        self.kd = KDTree(pts)
        self.ds = self.theta_vec[1]-self.theta_vec[0]
        self.Length = self.theta_vec[-1] + self.ds


    def create_grid(self, density=5000):

        theta_p = np.linspace(0,self.L, density)

        x_p = self.cs_x(theta_p)
        y_p = self.cs_y(theta_p)
        
        return x_p, y_p, theta_p

    
    def project(self, x, y):

        dist, idx = self.kd.query([x,y])
        return self.theta_vec[idx]



    def plot_track(self):
        plt.figure(figsize=(8, 6))
        plt.plot(self.x_new, self.y_new, label='Racetrack Path')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.show()

    def get_track_plot_params(self):
        return self.x_new, self.y_new
    

    def plot_simulation(self, Nsim, x, y, v):
        x = np.asarray(x)
        y = np.asarray(y)
        v = np.asarray(v)

        
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)


        fig, axs = plt.subplots()
        norm = plt.Normalize(v.min(), v.max()/4)
        lc = LineCollection(segments, cmap='viridis', norm=norm)
        lc.set_array(v)  
        lc.set_linewidth(2)
        line = axs.add_collection(lc)
        fig.colorbar(line, ax=axs)       
        axs.set_xlim(x.min(), x.max())
        axs.set_ylim(y.min(), y.max())  

        #axs.axis('equal'); 
        axs.grid(True)
        axs.legend()
        axs.set_title(f"MPC Closed‑Loop Simulation ({Nsim} steps)")
        axs.set_xlabel("x [m]"); plt.ylabel("y [m]")
        plt.show()

"""
track = Track(data='/home/dpma/projects/DiffDrive-ROS/ros_ws/src/my_py_pkg/my_py_pkg/track_points2.csv')
track.plot_track()

track.create_grid(density= 2000)
"""


