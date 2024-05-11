import numpy as np
from sklearn.neighbors import NearestNeighbors
import rospy
from sensor_msgs.msg import LaserScan
import pandas as pd
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")

def wraptopi(phi):
    return np.mod(phi + np.pi, 2*np.pi) - np.pi

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        Vt[m-1,:] *= -1
        R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=25, tolerance=0.0001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.max(distances)
        #if mean_error < tolerance:#np.abs(prev_error - mean_error) < tolerance:
        #    break
        prev_error = mean_error

    # calculate final transformation
    _,R,t = best_fit_transform(A, src[:m,:].T)

    return R, t, distances, i


class ScanICP(object):

    phi = np.linspace(-2*np.pi/3, 2*np.pi/3, 682)

    def __init__(self, r):
        mask = (r < 1.5) & (r > 0.1)
        self.r = r[mask]
        self.phi = ScanICP.phi[mask]
        self.m = len(self.r)
        self.x = self.r*np.cos(self.phi)
        self.y = self.r*np.sin(self.phi)
        self.P = np.array([self.x, self.y]).T

    def icp_match(self, prev_scan):

        if prev_scan.m > self.m:
            P_prev = prev_scan.P[np.random.randint(prev_scan.m, size=self.m), :]
            P_new = self.P
        elif prev_scan.m < self.m:
            P_new = self.P[np.random.randint(self.m, size=prev_scan.m), :]
            P_prev = prev_scan.P
        else:
            P_prev = prev_scan.P
            P_new = self.P

        Ricp, Ticp, d, i = icp(P_prev, P_new)

        while np.any(d >= 0.025):
            P_prev = P_prev[d < 0.025,:]
            P_new = P_new[d < 0.025,:]

            Ricp, Ticp, d, i = icp(P_prev, P_new)

        return Ricp, Ticp

class GlobalFrame(object):

    def __init__(self):
        self.lidar_data = None
        self.pose = np.zeros((2,))
        self.traj = self.pose

        # Initialize translation and rotation arrays
        self.R = np.zeros((2, 2))
        self.R[0, :] = np.eye(2)
        self.T = np.zeros((2,))

        self.scans = []

        rospy.init_node('scan_icp_node', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def laser_callback(self, data):
        self.lidar_data = np.array(data.ranges)

    def run_icp(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.lidar_data is not None:
                if len(self.scans) == 0:
                    self.scans.append(ScanICP(self.lidar_data))
                else:
                    new_scan = ScanICP(self.lidar_data)
                    Ricp, Ticp = new_scan.icp_match(self.scans[-1])

                    self.R = np.dot(self.R, Ricp.T)
                    self.T = np.dot(self.R, Ticp.reshape(2, 1)).flatten()

                    self.pose -= self.T
                    self.traj = np.vstack((self.traj, self.pose))

                    P_trans = np.dot(self.R, new_scan.P.T) - np.sum(self.T, 0).reshape(2, 1)

                    self.scans.append(new_scan)

                    print("Final Odometry:", self.pose)

            rate.sleep()


if __name__ == "__main__":
    global_frame = GlobalFrame()
    global_frame.run_icp()
