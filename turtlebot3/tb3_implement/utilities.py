import rospy
import rosbag
import cv2, numpy, pyexiv2
import os, sys, signal
import datetime
import random
from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String, Int16

import parametres as par
from parametres import msg


# ENMASCARAR IMAXE

def mask_images(img, mask = par.MASK):
        h, w = img.shape[:2]
        
        if mask[0] == 0:
            up = 0
        else:
            up = int(h/mask[0])

        if mask[1] == 0:
            down = 0
        else:
            down = int(h/mask[1])

        if mask[2] == 0:
            left = 0
        else:
            left = int(w/mask[2])

        if mask[3] == 0:
            right = 0
        else:
            right = int(w/mask[3])
       
        maskImg = numpy.zeros((h - up - down, w - left - right))
        maskImg = img[up:h - down, left:w - right]

        # maskImg = numpy.zeros_like(img)
        # maskImg[up:h - down, left:w - right] = img[up:h - down, left:w - right]

        return maskImg



# ENCONTRAR ESTADO MAIS CERCANO

def find_closest_state(image, stored_images, threshold = par.TH_DIST_IMAGE):
        print("estados", len(stored_images))

        list_dist = []
        if len(stored_images) == 0:
            return None
        min_dist = float('inf')
        min_idx = -1
        maskImg = mask_images(img=image)
        for i, img in enumerate(stored_images):
            mimg = mask_images(img=img[0])
            distance = numpy.sum((cv2.absdiff(maskImg.flatten(), mimg.flatten()) ** 2))
            list_dist.append([distance, i])
            if distance < min_dist:
                min_dist = distance
                min_idx = i

        # print(list_dist)
        list_dist = []

        if min_dist > threshold: # estado novo  # Probar a cambiar a TH_DIST_IMAGE / N_PX 
            return None
        else:
            return min_idx # detectamos estado actual       

# DETECTAR REFORZO NA IMAXE    
def check_ref_in_images(image, threshold = par.TH_R_IMAGE, color = par.COLOR, w = par.W, h = par.H, x = par.X, y = par.Y):
        img_gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        umbral, img_binaria = cv2.threshold(img_gris, 127, 255, cv2.THRESH_BINARY)
        region = img_binaria[y:y+h, x:x+w]

        # black_region = numpy.zeros((h, w), dtype=numpy.uint8)
        # coincidences = cv2.compare(region, black_region, cv2.CMP_EQ)

        ref_region = numpy.ones((h, w), dtype=numpy.uint8) * color
        coincidences = cv2.compare(region, ref_region, cv2.CMP_EQ)

        percentage = numpy.count_nonzero(coincidences) / coincidences.size
        # print(percentage)

        if percentage >= threshold:
            return 0.01
        else:
            return -1

# NODO COS DATOS COMUNS
class TrainingNode:
    def __init__(self, foldername, actions = par.ACTIONS):

        self.velocity_publisher = rospy.Publisher(par.TOPIC_VEL, Twist, queue_size=10)
        self.maskImg_publisher = rospy.Publisher(par.TOPIC_IMG_MASK, Image, queue_size=10)
        self.reinforcement_publisher = rospy.Publisher(par.TOPIC_REINFORCEMENT, String, queue_size=10)
        
        self.image_subscriber = rospy.Subscriber(par.TOPIC_CAMERA, Image, self.image_callback)        
        self.manual_stop_subscriber = rospy.Subscriber(par.TOPIC_STOP_ROBOT, Int16, self.manual_stop_callback)
        self.manual_start_subscriber = rospy.Subscriber(par.TOPIC_START_ROBOT, Int16, self.manual_start_callback)
        self.model_position_subscriber = rospy.Subscriber(par.TOPIC_MODEL_STATE, ModelStates, self.position_callback)

        self.set_position = rospy.ServiceProxy(par.TOPIC_SET_MODEL_STATE, SetModelState)

        self.bridge = CvBridge()

        self.image = None
        self.img_msg = None
        self.first_position = None
        self.robot_position = None
        self.first_pos_call = True

        self.stop_manual = False

        self.stored_images = []
        self.valid_pos = deque(maxlen=10)
        
        self.finish_position = None
        self.isfinish = 0
        self.rigth_lap = False


        self.actions = actions
        self.n_actions = par.N_ACTIONS

        self.number_states = 0
        self.current_state = None
        self.last_action = None

        self.folder = foldername

        now = datetime.datetime.now()
        bag_name = f"{self.folder}/{self.folder}_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.bag"
        self.bag_file = os.path.join(os.getcwd(), bag_name)
        self.bag = None

        self.linear_vel   = 0.0
        self.angular_vel  = 0.0

    ## CALLBACKS DE TOPICS
    def position_callback(self, msg):
        robot_index = msg.name.index(par.MODEL)
        self.robot_position = msg.pose[robot_index]
        
        if self.first_pos_call:

            self.first_position = self.robot_position
            self.finish_position = self.first_position
            self.first_pos_call = False

    def manual_stop_callback(self, data):
        self.stop_manual = 1

    def manual_start_callback(self, data):
        self.stop_manual = 0

    def image_callback(self, data):
        self.img_msg = data
        image_bridge = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = image_bridge

        maskImg = mask_images(img=image_bridge)
        maskImgMsg = self.bridge.cv2_to_imgmsg(maskImg, "bgr8")
        self.maskImg_publisher.publish(maskImgMsg)

    ## EXECUTAR ACCION
    def execute_action(self, action):
    
        self.linear_vel = self.actions[action][0] * par.VELOCITY_FACTOR
        self.angular_vel = self.actions[action][1] * par.VELOCITY_FACTOR

        twist = Twist()

        twist.linear.x = self.linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular_vel

        self.velocity_publisher.publish(twist)

    ## REINICIAR POSICION ROBOT
    def reset_position(self):
        # print(len(self.valid_pos))

        model_state = ModelState()
        model_state.model_name = par.MODEL 

        if self.valid_pos: # ultima posicion gardada se a lista non esta vacia
            last_pos = self.valid_pos[-1]
            model_state.pose = last_pos
            self.finish_position = last_pos
            del self.valid_pos[-1]
        else: # Posicion inicial        
            model_state.pose = self.first_position
            self.finish_position = self.first_position

        self.set_position(model_state)

        self.time_last_pose_saved = datetime.datetime.now() # reiniciamos o tempo de gardado de posicion para evitar que se garde a posicion onde se detecta reforzo

    ## ENGADIR POSICIONS A COLA
    def append_pos(self):

        if self.valid_pos: # Xa existe algunha posicion miramos tempo transcurrido
            time = datetime.datetime.now()
            time_diff = time - self.time_last_pose_saved
            if time_diff.seconds >= 5:
                self.valid_pos.append(self.robot_position)
                self.time_last_pose_saved = time
        else: # se non existe posicion engadimos unha e gardamos o tempo actual
            self.valid_pos.append(self.robot_position)
            self.time_last_pose_saved = datetime.datetime.now()


    ## DETER ROBOT
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

    def check_finish_pos(self):
        if self.robot_position == None or self.finish_position == None:
            pass
        elif self.robot_position.position.x >= self.finish_position.position.x - 0.1 and self.robot_position.position.x <= self.finish_position.position.x + 0.1 and self.robot_position.position.y >= self.finish_position.position.y - 0.1 and self.robot_position.position.y <= self.finish_position.position.y + 0.1:
            self.rigth_lap = True
        elif self.rigth_lap:
            self.isfinish = self.isfinish+1
            self.rigth_lap = False