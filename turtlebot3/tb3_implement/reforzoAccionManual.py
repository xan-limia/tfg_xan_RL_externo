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
from std_msgs.msg import String

# Ctrl+C to quit

msg = """
Posibles Accions 
---------------------------
1 Xiro Esquerda Mais Brusco
2 Xiro Esquerda Brusco
3 Xiro Esquerda Suave
4 Avanzar Recto
5 Xiro Dereita Suave
6 Xiro Dereita Brusco
7 Xiro Dereita Mais Brusco
"""

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

# THRESHOLDS
TH_DIST_IMAGE = 200000
# TH_R_IMAGE = 0.8
# TH_R_IMAGE = 0.05
TH_R_IMAGE = 0.04

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

# AREA REFORZO
# W = 40
# H = 24
# X = 0
# Y = 36

# PARAMETROS ROBOT
MODEL = 'turtlebot3_burger'
TOPIC_VEL = '/cmd_vel'
TOPIC_CAMERA = '/camera/image'
TOPIC_MODEL_STATE = '/gazebo/model_states'
TOPIC_SET_MODEL_STATE = '/gazebo/set_model_state'
TOPIC_REINFORCEMENT = '/reinforcement'
TOPIC_IMG_MASK = '/img_mask'

class ManualNode:
    def __init__(self, foldername):

        rospy.init_node('randomLearning')
        self.velocity_publisher = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber(TOPIC_CAMERA, Image, self.image_callback)
        self.model_position_subscriber = rospy.Subscriber(TOPIC_MODEL_STATE, ModelStates, self.position_callback)
        self.set_position = rospy.ServiceProxy(TOPIC_SET_MODEL_STATE, SetModelState)
        self.reinforcement_publisher = rospy.Publisher(TOPIC_REINFORCEMENT, String, queue_size=10)
        self.maskImg_publisher = rospy.Publisher(TOPIC_IMG_MASK, Image, queue_size=10)


        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 

        self.image = None
        self.img_msg = None
        self.robot_position = None

        self.stored_images = []
        self.state_action = []
        self.last_index_action = None
        self.valid_pos = deque(maxlen=10)

        # self.actions = [(0.15, 0.90), (0.15, 0.54), (0.15, 0.0), (0.15, -0.54), (0.15, -0.90)]
        self.actions = [(0.15, 1.20), (0.15, 0.90), (0.15, 0.54), (0.15, 0.0), (0.15, -0.54), (0.15, -0.90), (0.15, -1.20)]

        self.folder = foldername

        now = datetime.datetime.now()
        bag_name = f"{self.folder}/{self.folder}_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.bag"
        self.bag_file = os.path.join(os.getcwd(), bag_name)
        self.bag = None

        self.number_states = 0

        self.linear_vel   = 0.0
        self.angular_vel  = 0.0

        self.write = False
        

    def vels(self, linear_vel, angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel,angular_vel)
    
    def position_callback(self, msg):
        robot_index = msg.name.index(MODEL)
        self.robot_position = msg.pose[robot_index]

    def image_callback(self, data):
        self.img_msg = data
        image_bridge = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # self.image = self.mask_images(image_bridge)
        self.image = image_bridge ## Descomentar para executar sen mascara

    def mask_images(self, img):
        h, w = img.shape[:2]
        # mascara 1, tercio superior
        maskImg = numpy.zeros((h - int(h/3), w))
        maskImg= img[int(h / 3):]

        # mascara 2, tercio superior, tercio dereito
        # maskImg = numpy.zeros((h - int(h/3), w - int(w/3)))
        # maskImg= img[int(h / 3):, :w - int(w/3)]

        # mascara 3, tercio superior, quinto esquerda, quinto dereita
        # maskImg = numpy.zeros((h - int(h/3), w - int(w/5)*2))
        # maskImg = img[int(h / 3):, int(w/5):w - int(w/5)]

        # mascara 4, tercio superior, quinto dereita
        # maskImg = numpy.zeros((h - int(h/3), w - int(w/5)))
        # maskImg = img[int(h / 3):, :w - int(w/5)]
        return maskImg
    
    def load_images(self):
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
            return
        for filename in os.listdir(self.folder):
            if(filename.endswith('.png')):
                img = cv2.imread(os.path.join(self.folder, filename))
                # maskImg = self.mask_images(img)
                # self.stored_images.append(maskImg)
                self.stored_images.append(img) # Gardar imaxe sen mascara

                img = pyexiv2.Image(os.path.join(self.folder, filename))
                metadata = img.read_exif()
                text = metadata['Exif.Photo.UserComment']               
                linear_vel = float(text.split("=")[1].split("\n")[0])
                angular_vel = float(text.split("=")[2])
                self.state_action.append((linear_vel, angular_vel))
        
        self.number_states = len(self.stored_images)


    def check_ref_in_images(self, x, y, w, h, threshold):

        img_gris = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        umbral, img_binaria = cv2.threshold(img_gris, 127, 255, cv2.THRESH_BINARY)
        region = img_binaria[y:y+h, x:x+w]

        # black_region = numpy.zeros((h, w), dtype=numpy.uint8)
        # coincidences = cv2.compare(region, black_region, cv2.CMP_EQ)

        white_region = numpy.ones((h, w), dtype=numpy.uint8) * 255
        coincidences = cv2.compare(region, white_region, cv2.CMP_EQ)

        percentage = numpy.count_nonzero(coincidences) / coincidences.size
        #print(percentage)
        
        return percentage >= threshold

    def find_closest_velocity(self):
        print("estados", len(self.stored_images))
       
        list_dist = []
        if len(self.stored_images) == 0:
            return None
        min_dist = float('inf')
        min_idx = -1
        # maskImg = self.image
        maskImg = self.mask_images(self.image)
        # print(maskImg.shape)
        for i, img in enumerate(self.stored_images):
            img = self.mask_images(img=img)
            distance = numpy.sum((cv2.absdiff(maskImg.flatten(), img.flatten()) ** 2))
            list_dist.append([distance, i])
            if distance < min_dist:
                min_dist = distance
                min_idx = i

        print(list_dist)
        list_dist = []

        if min_dist > TH_DIST_IMAGE:
            return None
        else:
            self.last_index_action = min_idx
            self.current_state = min_idx
            return self.state_action[min_idx]
        
    def execute_action(self, action):
    
        self.linear_vel = self.actions[action][0] * VELOCITY_FACTOR
        self.angular_vel = self.actions[action][1] * VELOCITY_FACTOR

        twist = Twist()

        twist.linear.x = self.linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular_vel

        self.velocity_publisher.publish(twist)
        
    def reset_position(self):
        model_state = ModelState()
        model_state.model_name = MODEL 

        if self.valid_pos: # ultiuma posicion gardada se a lista non esta vacia
            last_pos = self.valid_pos[-1]
            model_state.pose = last_pos
            self.valid_pos.pop()
        else: # Posicion inicial
            # model_state.pose.position.x = 0.244979
            # model_state.pose.position.y = -1.786919
            # model_state.pose.position.z = -0.001002

            model_state.pose.position.x = 0.244508
            model_state.pose.position.y = -1.631067
            model_state.pose.position.z = -0.001002

            # model_state.pose.position.x = 0.244508
            # model_state.pose.position.y = -1.704041
            # model_state.pose.position.z = -0.001002
            
        self.set_position(model_state)

        self.time_last_pose_saved = datetime.datetime.now() # reiniciamos o tempo de gardado de posicion para evitar que se garde a posicion onde se detecta reforzo

    def append_pos(self):

        if self.valid_pos: # Xa existe algunha posicion miramos tempo transcurrido
            time = datetime.datetime.now()
            time_diff = time - self.time_last_pose_saved
            if time_diff.seconds >= 10:
                self.valid_pos.append(self.robot_position)
                self.time_last_pose_saved = time
        else: # se non existe posicion engadimos unha e gardamos o tempo actual
            self.valid_pos.append(self.robot_position)
            self.time_last_pose_saved = datetime.datetime.now()


    def stop_robot(self):
        # print("No hay coincidencias, denetiendo robot\n")
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        node.velocity_publisher.publish(twist)

    def append_states(self):
        if self.image is not None:
            self.stored_images.append(self.image)
            self.state_action.append((self.linear_vel, self.angular_vel))
            self.last_index_action = len(self.stored_images) - 1
            print("salvados", len(self.stored_images))
        self.number_states = len(self.stored_images)

    def write_images(self):

        for archivo in os.listdir(self.folder):
            ruta_archivo = os.path.join(self.folder, archivo)
            if ruta_archivo.endswith(".bag"):
                pass
            else:
                os.remove(ruta_archivo)

        
        for i, img in enumerate(self.stored_images):
            now = datetime.datetime.now()
            filename = f"{self.folder}/image_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.png"
            filepath = os.path.join(os.getcwd(), filename)
            self.lastSavedFilename = filepath
            cv2.imwrite(filepath, img)
            # Leer imagen
            img_data = pyexiv2.Image(filepath)
            metadata = img_data.read_exif()
            # Agregar metadato
            metadata['Exif.Photo.UserComment'] = f"linear={self.state_action[i][0]}\nangular={self.state_action[i][1]}"
            img_data.modify_exif(metadata)

    def sav_image(self, prefix):
        now = datetime.datetime.now()
        filename = f"{self.folder}/{prefix}_image_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.png"
        filepath = os.path.join(os.getcwd(), filename)
        self.lastSavedFilename = filepath
        cv2.imwrite(filepath, self.image)
        # Leer imagen
        # img_data = pyexiv2.Image(filepath)
        # metadata = img_data.read_exif()
        # # Agregar metadato
        # metadata['Exif.Photo.UserComment'] = f"linear={self.state_action[i][0]}\nangular={self.state_action[i][1]}"
        # img_data.modify_exif(metadata)
    
    def train(self):
        self.load_images()
        self.bag = rosbag.Bag(self.bag_file, 'w')
        first = True
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            if self.image is not None:
                # maskImg = self.mask_images(self.image)
                # maskImgMsg = self.bridge.cv2_to_imgmsg(maskImg, "bgr8")
                # self.maskImg_publisher.publish(maskImgMsg)

                if first:
                    # first_frame = self.image
                    first_frame = self.mask_images(self.image)
                    first = False
                
                # Comentar esta parte par realizar probas sin reiniciar a posicion do robot
                result = self.check_ref_in_images(x=X, y=Y, w=W, h=H, threshold=TH_R_IMAGE)
                if result == False :
                    message = String()
                    message.data = f"negative reinforcement detected in state {self.current_state}"
                    self.reinforcement_publisher.publish(message)
                    self.bag.write(TOPIC_REINFORCEMENT, message, current_time)
                    print(message.data)
                    self.sav_image("reinf")
                    
                    if self.last_index_action is not None: 
                        del self.stored_images[self.last_index_action]
                        del self.state_action[self.last_index_action]

                    # last_frame = self.image
                    last_frame = self.mask_images(self.image)

                    distance = numpy.sum((cv2.absdiff(first_frame.flatten(), last_frame.flatten()) ** 2))

                    self.reset_position()
                    first = True
                    with open("distancia_reforzo_siguelinea.txt", "a") as archivo:
                        archivo.write(str(distance) + "\n")

                else:
                    self.append_pos() # engadir nova posicion a cola


                #######################################################################
               
                closest_velocity = self.find_closest_velocity()
            
                if closest_velocity is not None:
                    message = String()
                    message.data = f"{self.current_state}, x: {self.robot_position.position.x}, y: {self.robot_position.position.y}, z: {self.robot_position.position.z}"
                    topic = f"/state_{str(self.current_state).zfill(2)}"
                    self.bag.write(topic, message, current_time)

                    twist = Twist()

                    twist.linear.x = closest_velocity[0]; twist.linear.y = 0.0; twist.linear.z = 0.0

                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = closest_velocity[1]

                    self.velocity_publisher.publish(twist)

                    self.image = None
                else:
                    self.stop_robot()

                    key = input(msg)
                    if key.lower() == '1':
                        action = 0
                        self.execute_action(action=action)
                        self.write = True
                    elif key.lower() == '2':
                        action = 1
                        self.execute_action(action=action)
                        self.write = True
                    elif key.lower() == '3':
                        action = 2
                        self.execute_action(action=action)
                        self.write = True
                    elif key.lower() == '4':
                        action = 3
                        self.execute_action(action=action)
                        self.write = True
                    elif key.lower() == '5':
                        action = 4
                        self.execute_action(action=action)
                        self.write = True
                    elif key.lower() == '6':
                        action = 5
                        self.execute_action(action=action)
                        self.write = True
                    elif key.lower() == '7':
                        action = 6
                        self.execute_action(action=action)
                        self.write = True
               
                    if self.write:
                        message = String()
                        message.data = f"action: {action}"
                        topic = f"/action_{action}"
                        self.bag.write(topic, message, current_time)

                        topic = "/images"
                        self.bag.write(topic, self.img_msg, current_time)
                        
                        self.append_states()

                        self.image = None
                        self.write = False

                    

                    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    node = ManualNode(foldername)

    def signal_handler(sig, frame):
        #print("Programa detenido")
        node.write_images()

        node.stop_robot()
    
        node.bag.close()

        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.train()

    node.stop_robot()
    
    