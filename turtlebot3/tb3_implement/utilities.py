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

ACTIONS = 7

# msg = """
# Posibles Accions 
# ---------------------------
# 1 Xiro Esquerda Brusco
# 2 Xiro Esquerda Suave
# 3 Avanzar Recto
# 4 Xiro Dereita Suave
# 5 Xiro Dereita Brusco
# """

# ACTIONS = 5

VELOCITY_FACTOR = 1

# PIXELES

N_PX = 60*80

# THRESHOLDS
# TH_DIST_IMAGE = 200000
TH_DIST_IMAGE = 180000
# TH_R_IMAGE = 0.8
TH_R_IMAGE = 0.05
# TH_R_IMAGE = 0.04

Q_VALUE_DEFAULT = 0.01

# # AREA REFORZO
# W = 8
# H = 6
# X = 36
# Y = 52

# AREA REFORZO
# W = 48
# H = 12
# X = 16
# Y = 47

# AREA REFORZO
W = 80
H = 12
X = 0
Y = 47

# AREA REFORZOs
# W = 40
# H = 24
# X = 0
# Y = 36

# POLITICAE-GREEDY
EPSILON = 0.00

# PARAMETROS Q-LEARNING
LEARNING_RATE = 0.1 
DISCOUNT_FACTOR = 0.9

# PARAMETROS ROBOT
MODEL = 'turtlebot3_burger'
TOPIC_VEL = '/cmd_vel'
TOPIC_CAMERA = '/camera/image'
TOPIC_MODEL_STATE = '/gazebo/model_states'
TOPIC_SET_MODEL_STATE = '/gazebo/set_model_state'
TOPIC_REINFORCEMENT = '/reinforcement'
TOPIC_IMG_MASK = '/img_mask'
TOPIC_STOP_ROBOT = '/stop_robot'
TOPIC_START_ROBOT = '/start_robot'


# def vels(linear_vel, angular_vel):
#         return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel,angular_vel)

def check_default_action(angular_vel):
        if angular_vel == 1.20:
            action = 0
        elif angular_vel == 0.90:
            action = 1
        elif angular_vel == 0.54:
            action = 2
        elif angular_vel == 0.0:
            action = 3
        elif angular_vel == -0.54:
            action = 4
        elif angular_vel == -0.90:
            action = 5
        elif angular_vel == -1.20:
            action = 6
        return action

def load_images_ql(folder, stored_images = [], q_values = []):
    if not os.path.exists(folder):
        os.makedirs(folder) # crear directorio se non existe
        return
    for filename in os.listdir(folder):
        if(filename.endswith('.png')):
            img = cv2.imread(os.path.join(folder, filename))
            stored_images.append(img) # Gardar imaxe sen mascara

            img = pyexiv2.Image(os.path.join(folder, filename)) # ler a imaxe con pyexiv para acceder os metadatos
            metadata = img.read_exif() # ler metadato

            # Transformar string nun vector q values
            text = metadata['Exif.Photo.UserComment']  
            if text.startswith("q_values"):
                q_values = eval(text.split("=")[1])
                q_values.append(q_values)
            elif text.startswith("linear"):
                angular_vel = float(text.split("=")[2])
                action = check_default_action(angular_vel=angular_vel)
                init_q_values = [random.uniform(0, 0.01) for _ in range(ACTIONS)] # inicializar de forma aleatoria 0 e 0.1
                init_q_values[action] = Q_VALUE_DEFAULT
                q_values.append(init_q_values) # q values novo estado
            else:
                init_q_values = [random.uniform(0, 0.01) for _ in range(ACTIONS)] # inicializar de forma aleatoria 0 e 0.1
                q_values.append(init_q_values) # q values novo estado
    number_states = len(stored_images)
    return stored_images, q_values, number_states

MASK = (3,0,5,5)

def mask_images(img, mask):
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

def find_closest_state(image, stored_images):
        print("estados", len(stored_images))

        list_dist = []
        if len(stored_images) == 0:
            return None
        min_dist = float('inf')
        min_idx = -1
        maskImg = mask_images(image, MASK)
        for i, img in enumerate(stored_images):
            mimg = mask_images(img=img, mask=MASK)
            distance = numpy.sum((cv2.absdiff(maskImg.flatten(), mimg.flatten()) ** 2))
            list_dist.append([distance, i])
            if distance < min_dist:
                min_dist = distance
                min_idx = i

        # print(list_dist)
        list_dist = []

        if min_dist > TH_DIST_IMAGE: # estado novo  # Probar a cambiar a TH_DIST_IMAGE / N_PX 
            return None
        else:
            return min_idx # detectamos estado actual