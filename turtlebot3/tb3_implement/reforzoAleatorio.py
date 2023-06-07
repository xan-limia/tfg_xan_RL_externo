import rospy
import rosbag
import cv2, numpy, pyexiv2
import os, sys, signal
import datetime
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String

# Ctrl+C to quit

# Posibles Accions
# ---------------------------
# 0 Avanzar Recto
# 1 Xiro Esquerda
# 2 Xiro Esquerda Brusco
# 3 Xiro Dereita
# 4 Xiro Dereita Brusco

ACTIONS = 5

# THRESHOLDS
TH_DIST_IMAGE = 600000
TH_R_IMAGE = 0.8

# AREA REFORZO
W = 8
H = 6
X = 36
Y = 52

# PARAMETROS ROBOT
MODEL = 'turtlebot3_burger'
TOPIC_VEL = '/cmd_vel'
TOPIC_CAMERA = '/camera/image'
TOPIC_MODEL_STATE = '/gazebo/model_states'
TOPIC_SET_MODEL_STATE = '/gazebo/set_model_state'
TOPIC_REINFORCEMENT = '/reinforcement'

class RandomLNode:
    def __init__(self, foldername):

        rospy.init_node('randomLearning')
        self.velocity_publisher = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber(TOPIC_CAMERA, Image, self.image_callback)
        self.model_position_subscriber = rospy.Subscriber(TOPIC_MODEL_STATE, ModelStates, self.position_callback)
        self.set_position = rospy.ServiceProxy(TOPIC_SET_MODEL_STATE, SetModelState)
        self.reinforcement_publisher = rospy.Publisher(TOPIC_REINFORCEMENT, String, queue_size=10)


        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 

        self.image = None
        self.robot_position = None

        self.stored_images = []
        self.state_action = []
        self.last_index_action = None

        self.actions = [(0.15, 0.0), (0.15, 0.54), (0.15, 0.90), (0.15, -0.54), (0.15, -0.90)]


        self.folder = foldername

        now = datetime.datetime.now()
        bag_name = f"{self.folder}/{self.folder}_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.bag"
        self.bag_file = os.path.join(os.getcwd(), bag_name)
        self.bag = None

        self.number_states = 0

        self.linear_vel   = 0.0
        self.angular_vel  = 0.0
        

    def vels(self, linear_vel, angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel,angular_vel)
    
    def position_callback(self, msg):
        robot_index = msg.name.index(MODEL)
        self.robot_position = msg.pose[robot_index]

    def image_callback(self, data):
        image_bridge = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = self.mask_images(image_bridge)
        # self.image = image_bridge ## Descomentar para executar sen mascara

    def mask_images(self, img):
        h, w = img.shape[:2]
        maskImg = numpy.zeros_like(img)
        maskImg[int(h / 3):] = img[int(h / 3):]
        return maskImg
    
    def load_images(self):
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
            return
        for filename in os.listdir(self.folder):
            if(filename.endswith('.png')):
                img = cv2.imread(os.path.join(self.folder, filename))
                maskImg = self.mask_images(img)
                self.stored_images.append(maskImg)
                # self.stored_images.append(img) # Gardar imaxe sen mascara

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

        black_region = numpy.zeros((h, w), dtype=numpy.uint8)

        coincidences = cv2.compare(region, black_region, cv2.CMP_EQ)

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
        for i, img in enumerate(self.stored_images):
            distance = numpy.sum((cv2.absdiff(self.image.flatten(), img.flatten()) ** 2))
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
            return self.state_action[min_idx]
        
    def random_action(self):
        action = numpy.random.randint(ACTIONS)  
        return action
    
    def execute_action(self, action):
    
        self.linear_vel = self.actions[action][0]
        self.angular_vel = self.actions[action][1]

        twist = Twist()

        twist.linear.x = self.linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular_vel

        self.velocity_publisher.publish(twist)
        
    def reset_position(self):
        model_state = ModelState()
        model_state.model_name = MODEL 
        model_state.pose.position.x = 0.244979
        model_state.pose.position.y = -1.786919
        model_state.pose.position.z = -0.001002
        self.set_position(model_state)

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
        self.image = None
        self.write = False
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
            
    
    def train(self):
        self.load_images()
        self.bag = rosbag.Bag(self.bag_file, 'w')
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.image is not None:

                result = self.check_ref_in_images(x=X, y=Y, w=W, h=H, threshold=TH_R_IMAGE)
                if(result == False):
                    message = String()
                    message.data = f"negative reinforcement detected in state {self.current_state}"
                    self.reinforcement_publisher.publish(message)
                    self.bag.write(TOPIC_REINFORCEMENT, message, current_time)

                    if self.last_index_action is not None: 
                        del self.stored_images[self.last_index_action]
                        del self.state_action[self.last_index_action]
                       
                    self.reset_position()

                closest_velocity = self.find_closest_velocity()
            
                if closest_velocity is not None:
                    message = String()
                    message.data = f"{self.current_state}, x: {self.robot_position.position.x}, y: {self.robot_position.position.y}, z: {self.robot_position.position.z}"
                    topic = f"/state_{self.current_state}"
                    self.bag.write(topic, message, current_time)

                    twist = Twist()

                    twist.linear.x = closest_velocity[0]; twist.linear.y = 0.0; twist.linear.z = 0.0

                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = closest_velocity[1]

                    self.velocity_publisher.publish(twist)
                else:
                    self.stop_robot()

                    random_action = self.random_action()

                    self.execute_action(random_action)

                    self.append_states()

                    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    node = RandomLNode(foldername)

    def signal_handler(sig, frame):
        #print("Programa detenido")
        node.write_images()

        # twist = Twist()
        # twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        # twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        # node.velocity_publisher.publish(twist)
        node.stop_robot()

        node.bag.close()

        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.train()

    node.stop_robot()
    
    