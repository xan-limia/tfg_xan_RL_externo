import rospy
import rosbag
import cv2, numpy
import os, sys, signal
import datetime
import random
from collections import deque
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String, Float32, Int16

# PARAMETROS ROBOT
MODEL = 'turtlebot3_burger'
TOPIC_VEL = '/cmd_vel'
TOPIC_CAMERA = '/camera/image'
# TOPIC_MODEL_STATE = '/gazebo/model_states'
# TOPIC_SET_MODEL_STATE = '/gazebo/set_model_state'
TOPIC_REINFORCEMENT = '/reinforcement'
TOPIC_IMG_MASK = '/img_mask'
TOPIC_JOY = '/joy'
TOPIC_STOP_ROBOT = '/stop_robot'
TOPIC_START_ROBOT = '/start_robot'

class JoyNode:
    def __init__(self):

        rospy.init_node('joy_controller')
        self.velocity_publisher = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.reinforcement_publisher = rospy.Publisher(TOPIC_REINFORCEMENT, String, queue_size=10)
        self.manual_control_stop_publisher = rospy.Publisher(TOPIC_STOP_ROBOT, Int16, queue_size=10)
        self.manual_control_start_publisher = rospy.Publisher(TOPIC_START_ROBOT, Int16, queue_size=10)

        self.joy_suscriber = rospy.Subscriber(TOPIC_JOY, Joy, self.joy_callback)

        self.image_subscriber = rospy.Subscriber(TOPIC_CAMERA, Image, self.image_callback)
        

        self.joy = None
        self.stop = 0
        self.change = False

        self.inn = 0

        self.linear = 0.15
        
        self.img_msg = None

        
    def image_callback(self, data):
        self.img_msg = data

 

    def joy_callback(self, data):

        
        self.buttons = data.buttons
        self.axes = data.axes

        if numpy.shape(self.buttons)[0]>0:
            inn=1
            self.cross=self.buttons[0]
            self.circle=self.buttons[1]
            self.square=self.buttons[2]
            self.triangle=self.buttons[3]

        if numpy.shape(self.axes)[0]>0:
          self.inn=1
          # self.linear=self.axes[1]
          self.angular=self.axes[0]
        
        if self.inn==1:
          if self.buttons[0]==0 and self.buttons[1]==0 and self.buttons[2]==0 and self.buttons[3]==0 and self.axes[0]==0 and self.axes[1]==0:
              selfinn=0
          else:
              pass


    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)


    def controller(self):
        self.bag = rosbag.Bag('manual_teleop_robot_real_1.bag', 'w')
        angular = -1
        while True:
            if self.inn == 1:
                if self.circle == 1:
                    self.stop = 1
                    self.manual_control_stop_publisher.publish(self.stop)
                    self.stop_robot()
                elif self.cross == 1 :
                    self.stop = 0
                    self.manual_control_start_publisher.publish(self.stop)

                elif self.triangle == 1:
                    
                    twist = Twist()
                    twist.linear.x = self.linear; twist.linear.y = 0.0; twist.linear.z = 0.0
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular
                    self.velocity_publisher.publish(twist)

                    if self.angular != angular:
                        current_time = rospy.Time.now()
                        topic = "/images"
                        self.bag.write(topic, self.img_msg, current_time)

                        angular_vel = Float32()
                        angular_vel.data = self.angular
                        self.bag.write("/angular", angular_vel, current_time)

                    angular = self.angular
                
                    
if __name__ == '__main__':

    node = JoyNode()

    def signal_handler(sig, frame):
        node.bag.close() # cerrar ficheiro bag
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.controller() # executar entrenamento
