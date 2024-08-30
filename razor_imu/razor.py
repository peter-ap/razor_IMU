import serial
import time
import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time
import rclpy.time_source
from rclpy.clock import Clock
from sensor_msgs.msg import Imu
import numpy as np # Scientific computing library for Python
 
 
#VARIABLES
degrees2rad = np.pi/180.0
imu_yaw_calibration = 0.0

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
orientation_covariance = [
0.0025 , 0. , 0.,
0., 0.0025, 0.,
0., 0., 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
angular_velocity_covariance = [
0.02, 0. , 0.,
0. , 0.02, 0.,
0. , 0. , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
linear_acceleration_covariance = [
0.04 , 0. , 0.,
0. , 0.04, 0.,
0. , 0. , 0.04
]



#Needs to be set at launch file 
frame_id ="imu"

#STAND ALONE FUNCTIONS
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

# ------------------------


class imuMsg(Node):
    def __init__(self):
        super().__init__('razor_imu')
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
           # open port
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)         # 1/timeout is the frequency at which the port is read
        # set output mode
        self.ser.write(('#ox').encode("utf-8"))
        #wait for 5 seconds
        time.sleep(5)
    
    def timer_callback(self):
        #get data from imu
        line = self.ser.readline().decode().strip()
        line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
        line = line.replace("\r\n","")   # Delete "\r\n"
        imuData = line.split(",")    # Fields split
       #-------------------------------------
       #Convert Euler to Quaternion
       #-------------------------------------
       #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw_deg = -float(imuData[0])
        yaw_deg = yaw_deg + imu_yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        yaw = yaw_deg*degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = -float(imuData[1])*degrees2rad
        roll = float(imuData[2])*degrees2rad
        q = get_quaternion_from_euler(roll, pitch, yaw)
        
        
       #-------------------------------------
        #Fill message
       #-------------------------------------
        imuMsg = Imu()
        imuMsg.orientation.x = q[0]
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        
        imuMsg.linear_acceleration.x = -float(imuData[3]) * accel_factor
        imuMsg.linear_acceleration.y = float(imuData[4]) * accel_factor
        imuMsg.linear_acceleration.z = float(imuData[5]) * accel_factor
        
        imuMsg.angular_velocity.x = float(imuData[6])
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -float(imuData[7])
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        imuMsg.angular_velocity.z = -float(imuData[8])

        imuMsg.orientation_covariance = orientation_covariance
        imuMsg.angular_velocity_covariance = angular_velocity_covariance
        imuMsg.linear_acceleration_covariance = linear_acceleration_covariance


        # time_stamp = time.time()
        imuMsg.header.stamp= self.get_clock().now().to_msg()
        imuMsg.header.frame_id = frame_id
        # imuMsg.header.seq = seq
        # seq = seq + 1
        self.publisher.publish(imuMsg)
        
def main(args=None):
    rclpy.init(args=args)

    razor_imu=imuMsg()
    
    rclpy.spin(razor_imu)

    razor_imu.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
