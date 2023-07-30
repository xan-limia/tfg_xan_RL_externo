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

# PIXELES

N_PX = 60*80

# THRESHOLDS
TH_DIST_IMAGE = 200000  # Probar a cambiar a TH_DIST_IMAGE / N_PX 
# TH_DIST_IMAGE = 150
TH_R_IMAGE = 0.0000000000001

# AREA REFORZO
# W = 8
# H = 6
# X = 36
# Y = 52

# AREA REFORZO
W = 48
H = 12
X = 16
Y = 47

# AREA REFORZO
# W = 56
# H = 12
# X = 0
# Y = 47

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

class QLNode:
    def __init__(self, foldername):

        rospy.init_node('qlearning')
        self.velocity_publisher = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber(TOPIC_CAMERA, Image, self.image_callback)
        self.model_position_subscriber = rospy.Subscriber(TOPIC_MODEL_STATE, ModelStates, self.position_callback)
        self.set_position = rospy.ServiceProxy(TOPIC_SET_MODEL_STATE, SetModelState)
        self.reinforcement_publisher = rospy.Publisher(TOPIC_REINFORCEMENT, String, queue_size=10)

        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 

        self.image = None
        self.img_msg = None
        self.robot_position = None

        self.stored_images = []
        self.q_values = []
        self.valid_pos = deque(maxlen=10)

        # self.actions = [(0.15, 0.90), (0.15, 0.54), (0.15, 0.0), (0.15, -0.54), (0.15, -0.90)]
        self.actions = [(0.15, 1.20), (0.15, 0.90), (0.15, 0.54), (0.15, 0.0), (0.15, -0.54), (0.15, -0.90), (0.15, -1.20)]

        self.stored_velocities = []
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

    def vels(self, linear_vel, angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel,angular_vel)
    
    def position_callback(self, msg):
        robot_index = msg.name.index(MODEL)
        self.robot_position = msg.pose[robot_index]

    # IMAXES

    def image_callback(self, data):
        self.img_msg = data
        image_bridge = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # self.image = self.mask_images(image_bridge)
        self.image = image_bridge ## Descomentar para executar sen mascara

    def mask_images(self, img):
        h, w = img.shape[:2]
        # mascara 1, tercio superior
        # maskImg = numpy.zeros((h - int(h/3), w))
        # maskImg= img[int(h / 3):]

        # mascara 2, tercio superior, tercio dereito
        # maskImg = numpy.zeros((h - int(h/3), w - int(w/3)))
        # maskImg= img[int(h / 3):, :w - int(w/3)]

        # mascara 3, tercio superior, quinto esquerda, quinto dereita
        maskImg = numpy.zeros((h - int(h/3), w - int(w/5)*2))
        maskImg = img[int(h / 3):, int(w/5):w - int(w/5)]

        # mascara 4, tercio superior, quinto dereita
        # maskImg = numpy.zeros((h - int(h/3), w - int(w/5)))
        # maskImg = img[int(h / 3):, :w - int(w/5)]

        return maskImg
    
    def load_images(self):
        if not os.path.exists(self.folder):
            os.makedirs(self.folder) # crear directorio se non existe
            return
        for filename in os.listdir(self.folder):
            if(filename.endswith('.png')):
                img = cv2.imread(os.path.join(self.folder, filename))
                # maskImg = self.mask_images(img)
                # self.stored_images.append(maskImg)
                self.stored_images.append(img) # Gardar imaxe sen mascara

                img = pyexiv2.Image(os.path.join(self.folder, filename)) # ler a imaxe con pyexiv para acceder os metadatos
                metadata = img.read_exif() # ler metadato

                # Transformar string nun vector q values
                text = metadata['Exif.Photo.UserComment']  
                q_values = eval(text.split("=")[1])
                self.q_values.append(q_values)
        
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
        # print(percentage)

        if percentage >= threshold:
            return 0.01
        else:
            return -1

    def find_closest_state(self):
        print("estados", len(self.stored_images))

        list_dist = []
        if len(self.stored_images) == 0:
            return None
        min_dist = float('inf')
        min_idx = -1
        # maskImg = self.image
        maskImg = self.mask_images(self.image)
        for i, img in enumerate(self.stored_images):
            mimg = self.mask_images(img=img)
            distance = numpy.sum((cv2.absdiff(maskImg.flatten(), mimg.flatten()) ** 2))
            list_dist.append([distance, i])
            if distance < min_dist:
                min_dist = distance
                min_idx = i

        print(list_dist)
        list_dist = []

        if min_dist > TH_DIST_IMAGE: # estado novo  # Probar a cambiar a TH_DIST_IMAGE / N_PX 
            return None
        else:
            return min_idx # detectamos estado actual
        
    def append_states(self):
        if self.image is not None:
            self.stored_images.append(self.image) # novo estado
            init_q_values = [random.uniform(0, 0.1) for _ in range(ACTIONS)] # inicializar de forma aleatoria 0 e 0.1
            self.q_values.append(init_q_values) # q values novo estado
        self.number_states = len(self.stored_images)

    ## ACCIONS

    def random_action(self):
        action = numpy.random.randint(ACTIONS)
        return action
    
    def best_action(self):
        q_values = self.q_values[self.current_state]
        action = numpy.argmax(q_values)
        return action

    def select_action(self):
        if numpy.random.random() < EPSILON:
            action = self.random_action()  # Explorar mais ao principio
        else:
            action = self.best_action()
        return action
    
    def execute_action(self, action):
    
        self.linear_vel = self.actions[action][0] * VELOCITY_FACTOR
        self.angular_vel = self.actions[action][1] * VELOCITY_FACTOR

        twist = Twist()

        twist.linear.x = self.linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular_vel

        self.velocity_publisher.publish(twist)

    ## ACTUALIZAR Q VALUES

    def update_q_values(self, reward, new_state):
        current_q_value = self.q_values[self.current_state][self.last_action]

        new_q_values = self.q_values[new_state]
        print(self.q_values[self.current_state])
        max_q_value = numpy.amax(new_q_values, axis=None)

        new_q_value = current_q_value + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_q_value - current_q_value)
        self.q_values[self.current_state][self.last_action] = new_q_value

        

    ## REINICIAR POSICION ROBOT
        
    def reset_position(self):
        # print(len(self.valid_pos))

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

    ## ENGADIR POSICIONS A COLA

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


    ## DETER ROBOT

    def stop_robot(self):
        # print("No hay coincidencias, denetiendo robot\n")
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        node.velocity_publisher.publish(twist)

    
    ## GARDAR IMAXES EN DISCO
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
            metadata['Exif.Photo.UserComment'] = f"q_values={self.q_values[i]}"
            img_data.modify_exif(metadata)
            
    ## ENTRENAR
    def train(self):
        self.load_images()
        self.bag = rosbag.Bag(self.bag_file, 'w')
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.image is not None:
                
                self.current_state = self.find_closest_state() # encontrar estado actual
                print("estado actual: ", self.current_state)

                # first_frame = self.image
            
                if self.current_state is not None:
                    message = String()
                    message.data = f"{self.current_state}, x: {self.robot_position.position.x}, y: {self.robot_position.position.y}, z: {self.robot_position.position.z}"
                    topic = f"/state_{str(self.current_state).zfill(2)}"
                    self.bag.write(topic, message, current_time)
                    
                    action = self.select_action() # seleccionar accion

                    self.execute_action(action=action) # executar accion selecionada
                    # print("accion: ", action)

                    message.data = f"action: {action},  state: {self.current_state}"
                    topic = f"/action_{action}"
                    self.bag.write(topic, message, current_time)

                    self.last_action = action 
                    
                    self.image = None

                    while self.image is None:
                        pass

                    reward = self.check_ref_in_images(x=X, y=Y, w=W, h=H, threshold=TH_R_IMAGE) # obter recompensa

                    # last_frame = self.image

                    # distance = numpy.sum((cv2.absdiff(first_frame.flatten(), last_frame.flatten()) ** 2))

                    new_state = self.find_closest_state() # obter o estado despois de executar a accion
                    
                    if new_state is None: # estado novo (crear)
                        print(new_state)
                        self.stop_robot() # eliminar no real
                        self.append_states()
                        new_state = self.find_closest_state()

                    print("estado siguiente: ", new_state)
                    
                    
                    self.update_q_values(reward, new_state) # actualizar q_values

                    if reward == -1: # recompensa negativa
                        message = String()
                        now = datetime.datetime.now()
                        message.data = f"{now.strftime('%m-%d_%H-%M-%S')}: negative reinforcement detected in state {self.current_state} when applying action {action}"
                        self.reinforcement_publisher.publish(message)
                        self.bag.write(TOPIC_REINFORCEMENT, message, current_time)

                        self.reset_position() # reiniciar a ultima posicion gardada
                        
                    else:
                        self.append_pos() # engadir nova posicion a cola

                    self.image = None # reiniciar a imaxe capturada para obligar a ter unha nova

                else:
                    self.stop_robot()

                    self.append_states() # crear novo estado

                    topic = "/images"
                    self.bag.write(topic, self.img_msg, current_time)

                    self.image = None

                    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    node = QLNode(foldername)

    def signal_handler(sig, frame):
        #print("Programa detenido")
        node.write_images() # escribir imaxes en disco

        node.stop_robot() # parar robot

        node.bag.close() # cerrar ficheiro bag

        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.train() # executar entrenamento

    node.stop_robot()
    
    