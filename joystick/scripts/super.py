import rospy
import numpy as np
from std_msgs.msg import Int16, Int8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import  Pose2D, Twist
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from kalman.cfg import kparametersConfig


class localization_ekf:
    def __init__ (self):
        self.dt=0.00001 #make accurate time
        # self.Q = np.diag([0.00003, 0.00003,0.01,0.0003])
        # self.R = np.diag([0])####
        # self.H = np.array([[0,0,1,0]])
        
        # self.x0 = np.array([[0,0,0,0]]).T
        # self.P0 = np.diag([1,1,1,1])

        self.Kp = 5
        self.Ki = 0.0
        self.Kd = 0.001
        self.detection = None
        self.error = 0.0
        self.error_sum = 0.0
        self.error_diff = 0.0
        self.error_prev = 0.0
        self.target_angle = 0.0
        self.yaw = 0.0
        self.yaw_enc = 0.0
        self.ax = 0.0
        self.vx = 0.0
        self.pid_value = 0.0
        self.prev_x = 0
        self.prev_y = 0
        





    def node_init(self):
        rospy.init_node("ekf_node",anonymous=True)
        
        # rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        # rospy.Subscriber("/imu_target",Int8,self.pid)
        rospy.Subscriber("/odom",Odometry,   self.ekf_callback)
        rospy.Subscriber("detection", Int8, self.detection_callback)
        rospy.Subscriber("mine_theta", Int8, self.mine_pose)

        
        
        # srv = Server(kparametersConfig, self.callback)
        # self.filtered_pose_pub=rospy.Publisher('/filtered_pose', Pose2D,queue_size=10)
        # self.angles_pub= rospy.Publisher('/angles', Pose2D,queue_size=10)
        # self.yaw_PID_pub= rospy.Publisher('/yaw_PID', Int16,queue_size=10)
        self.mine_pose_pub= rospy.Publisher('/mine_pose', Pose2D,queue_size=10)
        rospy.spin()
    
    # def callback(self, config, level):
    #     self.Q = np.diag([config.a, config.b,config.c,config.d])
    #     self.R = np.diag([config.R])
    #     self.Kp = config.Kp
    #     self.Ki = config.Ki
    #     self.Kd = config.Kd
    #     return config

    def imu_callback(self, imu_msg):
        # self.yaw = self.imu_msg.yaw
        # self.yaw = self.yaw * np.pi/180
        # self.ax=0

        self.orientation_q = imu_msg.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y,
                                  self.orientation_q.z, self.orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (self.orientation_list)
        # self.yaw = self.yaw*180/np.pi
        # self.ax = imu_msg.linear_acceleration.x

    #     self.error =self.target_angle - self.yaw
    #     self.error_sum += self.error
    #     self.error_diff = self.error - self.error_prev
    #     self.error_prev = self.error
    #     self.pid_value = self.Kp*self.error + self.Ki*self.error_sum + self.Kd*self.error_diff
    #     if self.error > -0.03 and self.error < 0.03:
    #         self.pid_value = 0
    #         self.error_sum = 0

    #     if self.pid_value > 20:
    #         self.pid_value = 20
    #     elif self.pid_value < -20:   
    #         self.pid_value = -20

    #     self.speed = int(self.pid_value)

    #     self.pid_angle = Int16()
    #     self.pid_angle.data = self.speed
    #     self.yaw_PID_pub.publish(self.pid_angle)

    # def pid(self,flag):
    #     if flag.data == 1:
    #         self.target_angle = self.yaw
    #         print("target_angle: ",self.target_angle)
    #     else:
    #         pass


        

    def ekf_callback(self, odom_msg):
        # self.x_hat = np.array([[0,0,0,0]]).T

        # orientation_q_enc = odom_msg.pose.pose.orientation
        # orientation_list_enc = [orientation_q_enc.x, orientation_q_enc.y,
        #                          orientation_q_enc.z, orientation_q_enc.w]
        # (roll_enc, pitch_enc, self.yaw_enc) = euler_from_quaternion (orientation_list_enc)

        self.x_pose= odom_msg.pose.pose.position.x
        self.y_pose= odom_msg.pose.pose.position.y

        # self.x_hat[0][0] = odom_msg.pose.pose.position.x
        # self.x_hat[1][0] = odom_msg.pose.pose.position.y
        # self.x_hat[2][0] = self.yaw_enc
        # self.x_hat[3][0] = odom_msg.twist.twist.linear.x

        # self.h = np.array([[self.yaw_enc]])

        # # self.vx = self.ax*self.dt + self.x_hat[0][3]
        # self.imu_measurement = np.array([[self.yaw]])

        # self.A = np.array([
        #     [1,0,-self.dt*self.x_hat[3][0]*np.sin(self.x_hat[2][0]),
        #      self.dt*np.cos(self.x_hat[2][0])],

        #     [0,1,self.dt*self.x_hat[3][0]*np.cos(self.x_hat[2][0]),
        #      self.dt*np.sin(self.x_hat[2][0])],
        #      [0,0,1,0],
        #      [0,0,0,1]])

        # self.p = (self.A.dot(self.P0).dot(self.A.T)) + self.Q
        # self.k = self.p.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.p).dot(self.H.T)+self.R))
        
        # self.x_hat = self.x_hat + self.k.dot((self.imu_measurement-self.h))
        # self.P0 = (np.eye(4)-self.k.dot(self.H)).dot(self.p)

        # self.filtered_pose = Pose2D()
        # self.filtered_pose.x = self.x_hat[0][0]
        # self.filtered_pose.y = self.x_hat[1][0]
        # self.filtered_pose.theta = self.x_hat[2][0]
        # self.filtered_pose_pub.publish(self.filtered_pose)
        # self.x0 = self.x_hat
        # print("x0: ",self.x0)
        # print("x_hat: ",self.x_hat)

        # print("p0: ",self.P0)
        # self.angles = Pose2D()
        # self.angles.x = self.yaw*180/np.pi
        # self.angles.y = self.yaw_enc*180/np.pi
        # self.angles.theta = self.x_hat[2][0]*180/np.pi
        # self.angles_pub.publish(self.angles)




    
    def detection_callback(self,msg):
        self.detection = msg.data

    def mine_pose(self,msg):
        self.mine_position = Pose2D()
        self.location = msg.data
        
        if self.detection == 1:
            print("detection: ", self.detection)
            if self.location == 1:
                print("location:", self.location)
                self.mine_position.x = self.x_pose + 1.08 * np.cos(self.yaw)
                self.mine_position.y = self.y_pose + 1.08 * np.sin(self.yaw)
                
                print("abs x = ",abs((self.mine_position.x - self.prev_x)), "")
                if ((abs(self.mine_position.x - self.prev_x)) > 0.8) or ((abs(self.mine_position.y - self.prev_y))>0.8):
                    print("location 1 and pose >0.6")
                    
                    self.prev_x = self.mine_position.x
                    self.prev_y = self.mine_position.y
                    self.mine_position.theta = 1
                    self.mine_pose_pub.publish(self.mine_position)
            elif self.location == 2:
                print("location:", self.location)
                self.mine_position.x = self.x_pose + 1.08 * np.cos(self.yaw)
                self.mine_position.y = self.y_pose + 1.08 * np.sin(self.yaw)
                if ((abs(self.mine_position.x - self.prev_x)) > 0.8) or ((abs(self.mine_position.y - self.prev_y))>0.8):
                    print("location 2 and pose >0.6")
                    
                    self.mine_position.theta = 2
                    self.mine_pose_pub.publish(self.mine_position)



       




if __name__=='__main__':
    localization_ekf = localization_ekf()
    localization_ekf.node_init()

