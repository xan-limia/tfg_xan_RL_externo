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
from std_msgs.msg import String, Bool

# PARAMETROS ROBOT
MODEL = 'turtlebot3_burger'
TOPIC_VEL = '/cmd_vel'
TOPIC_CAMERA = '/camera/image'
TOPIC_MODEL_STATE = '/gazebo/model_states'
TOPIC_SET_MODEL_STATE = '/gazebo/set_model_state'
TOPIC_REINFORCEMENT = '/reinforcement'
TOPIC_IMG_MASK = '/img_mask'
TOPIC_JOY = '/joy'
TOPIC_STOP_ROBOT = '/stop_robot'

class JoyNode:
    def __init__(self):

        rospy.init_node('joy_controller')
        self.velocity_publisher = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.reinforcement_publisher = rospy.Publisher(TOPIC_REINFORCEMENT, String, queue_size=10)
        self.joy_suscriber = rospy.Subscriber(TOPIC_JOY, Joy, self.joy_callback)
        self.manual_control_publisher = rospy.Publisher(TOPIC_STOP_ROBOT, Bool, queue_size=10)


        self.joy = None
        self.stop = False
        self.change = False

        self.inn = 0

        
        

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
          self.linear=self.axes[1]
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
            
        while True:
            if self.inn == 1:
                if self.circle == 1:
                    self.stop = True
                    self.manual_control_publisher.publish(self.stop)
                    self.stop_robot()
                elif self.cross ==1 :
                    self.stop = False
                    self.manual_control_publisher.publish(self.stop)

                elif self.triangle == 1:
                    twist = Twist()
                    twist.linear.x = self.linear; twist.linear.y = 0.0; twist.linear.z = 0.0
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular
                    self.velocity_publisher.publish(twist)
                
                    

            
                
                 
        



if __name__ == '__main__':

    node = JoyNode()

    def signal_handler(sig, frame):
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.controller() # executar entrenamento
