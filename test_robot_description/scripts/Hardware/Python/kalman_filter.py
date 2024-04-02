#/usr/env/bin python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/test_robot/odom_noisy", Odometry, self.odom_callback)
        self.imu_sum = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher("/test_robot/odom_kalman_filtered", Odometry, queue_size=10)

        self.mean = 0.0
        self.variance = 1000 # setting a higher variance to indicate high uncertainity
        self.imu_angular_z = 0.0 # stores the last value of angular velocity in z axis coming from imu
        self.is_first_odom = True
        self.last_angular_z = 0.0 # stores the last value of angular velocity in z axis coming from wheel encoders
        self.motion = 0.0
        self.kalman_odom = Odometry()
        self.motion_variance = 4.0
        self.measurement_variance = 0.5 # measurement is taken from imu

    def imu_callback(self, imu):
        self.imu_angular_z = imu.angular_velocity.z

    def odom_callback(self, odom):
        self.kalman_odom = odom

        if self.is_first_odom:
            self.mean = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z
            self.is_first_odom = False
            
            return 
        
        self.motion = odom.twist.twist.angular.z - self.imu_angular_z
        
        self.state_prediction()
        self.measurement_update()

        self.kalman_odom.twist.twist.angular.z = self.mean

        self.odom_pub(self.kalman_odom)

        self.last_angular_z = odom.twist.twist.angular.z

    def measurement_update(self):
        self.mean = ((self.measurement_variance * self.mean) + (self.variance * self.imu_angular_z)) / (self.measurement_variance + self.variance)
        self.variance = (self. variance * self.measurement_variance) / (self.variance + self.measurement_variance)


    def state_prediction(self):
        self.mean = self.mean + self.motion
        self.variance = self.variance + self.motion_variance


if __name__ == "_main__":
    rospy.init_node("kalman_filter_node")
    kf = KalmanFilter()
    rospy.spin()



