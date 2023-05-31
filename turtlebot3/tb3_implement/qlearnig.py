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

msg = """
Introduce la direccion
---------------------------
1 Avanzar Recto
2 Xiro Esquerda
3 Xiro Esquerda Brusco
4 Xiro Dereita
5 Xiro Dereita Brusco
Ctrl+C to quit
"""

TH_DIST_IMAGE = 650000
TH_R_IMAGE = 0.8
W = 8
H = 6
X = 36
Y = 52

EPSILON = 0.1
ACTIONS = 5

LEARNING_RATE = 0.1
DISCOUNT_FACTOR = 0.9

MODEL = 'turtlebot3_burger'

class TeleoperationNode:
    def __init__(self, foldername):
        rospy.init_node('teleoperation')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber('/camera/image', Image, self.image_callback)
        #self.turtlebot3_model = rospy.get_param("model", "burger")
        self.model_position = rospy.Subscriber("/gazebo/model_states", ModelStates, self.position_callback)
        self.set_position = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 

        self.image = None
        self.img_msg = None
        self.robot_position = None

        self.stored_images = []
        self.q_values = []
        self.valid_pos = deque(maxlen=10)

        self.stored_velocities = []
        self.number_states = 0
        self.current_state = None
        self.last_action = None
        #self.penultimate_index_action = None

        self.folder = foldername

        now = datetime.datetime.now()
        bag_name = f"{self.folder}/{self.folder}_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.bag"
        self.bag_file = os.path.join(os.getcwd(), bag_name)
        self.bag = None

       
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
    
    def position_callback(self, msg):
        robot_index = msg.name.index('turtlebot3_burger')
        self.robot_position = msg.pose[robot_index]

    def image_callback(self, data):
        self.img_msg = data
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
                q_values = eval(text.split("=")[1])
                self.q_values.append(q_values)
        
        self.number_states = len(self.stored_images)


    def check_ref_in_images(self, x, y, w, h, threshold):

        img_gris = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        umbral, img_binaria = cv2.threshold(img_gris, 127, 255, cv2.THRESH_BINARY)
        region = img_binaria[y:y+h, x:x+w]

        black_region = numpy.zeros((h, w), dtype=numpy.uint8)

        coincidences = cv2.compare(region, black_region, cv2.CMP_EQ)

        percentage = numpy.count_nonzero(coincidences) / coincidences.size
        # print(percentage)

        # if percentage == 1.0:
        #     return 1
        if percentage >= threshold:
            return 1
        else:
            return -1

    def find_closest_state(self):
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
            return min_idx # detectamos estado actual
        
    def append_states(self):
        if self.image is not None:
            self.stored_images.append(self.image)
            init_q_values = [random.uniform(0, 0.1) for _ in range(5)] # inicializar de forma aleatoria 0 e 0.1
            self.q_values.append(init_q_values) 
            self.last_index_action = len(self.stored_images) - 1
            # print("salvados", len(self.stored_images))
        # self.img_msg = None
        # self.image = None
        self.number_states = len(self.stored_images)

    ## ACCIONS

    def select_action(self):

        q_values = self.q_values[self.current_state]
        if numpy.random.random() < EPSILON:
            action = numpy.random.randint(5)  # Explorar mais ao principio
        else:
            action = numpy.argmax(q_values)
        return action
    
    def execute_action(self, action):
        if action == 0:
            self.target_linear_vel = 0.15
            self.target_angular_vel = 0.0
        elif action == 1:
            self.target_angular_vel = 0.54
            self.target_linear_vel = 0.15
        elif action == 2:
            self.target_angular_vel = 0.90
            self.target_linear_vel = 0.15
        elif action == 3:
            self.target_angular_vel = -0.54
            self.target_linear_vel = 0.15
        elif action == 4:
            self.target_angular_vel = -0.90
            self.target_linear_vel = 0.15

        twist = Twist()

        twist.linear.x = self.target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.target_angular_vel

        self.velocity_publisher.publish(twist)

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
        if self.valid_pos:
            last_pos = self.valid_pos[-1]
        else: # Posicion inicial
            model_state = ModelState()
            model_state.model_name = MODEL 
            model_state.pose.position.x = 0.244979
            model_state.pose.position.y = -1.786919
            model_state.pose.position.z = -0.001002
            self.set_position(model_state)
            return
        
        model_state = ModelState()
        model_state.model_name = MODEL 
        model_state.pose = last_pos
        self.set_position(model_state)

            


    ## DETER ROBOT

    def stop_robot(self):
        print("No hay coincidencias, denetiendo robot\n")
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        node.velocity_publisher.publish(twist)

    

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
            
    
    def train(self):
        self.load_images()
        # self.bag = rosbag.Bag(self.bag_file, 'w')
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.image is not None:
                
                self.current_state = self.find_closest_state()
            
                if self.current_state is not None:
                    # message = String()
                    # message.data = f"{self.current_state}"
                    # topic = f"/state_{self.current_state}"
                    # self.bag.write(topic, message, current_time)
                    # self.bag.write('/position', self.robot_position, current_time)

                    #print("Se ha encontrado coincidiencia, el estado existe\n")
                    
                    action = self.select_action()

                    self.execute_action(action=action)

                    self.last_action = action

                    reward = self.check_ref_in_images(x=X, y=Y, w=W, h=H, threshold=TH_R_IMAGE)

                    new_state = self.find_closest_state()
                    
                    if new_state is None: # estado novo
                        self.stop_robot() # eliminar no real
                        self.append_states()
                        # n_state = True
                        new_state = self.find_closest_state()
                    # else:
                    #     n_state = False
                    
                    # if self.current_state != new_state:
                    #     self.update_q_values(reward, new_state)
                    self.update_q_values(reward, new_state)

                    if(reward == -1):
                        # message = String()
                        # message.data = "reinforcement detected"
                        # self.bag.write('/reinforcement', message, current_time)
                        # if(n_state == True):
                        #     self.stored_images.pop()
                        #     self.q_values.pop()

                        self.reset_position()
                        self.time_last_pose_saved = datetime.datetime.now()
                    else:
                        if self.valid_pos:
                            time = datetime.datetime.now()
                            time_diff = time - self.time_last_pose_saved
                            if time_diff.seconds >= 10:
                                self.valid_pos.append(self.robot_position)
                                self.time_last_pose_saved = time
                        else:
                            time = datetime.datetime.now()
                            self.valid_pos.append(self.robot_position)
                            self.time_last_pose_saved = time

                else:
                    self.stop_robot()

                    self.append_states()

                    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    node = TeleoperationNode(foldername)

    def signal_handler(sig, frame):
        #print("Programa detenido")
        node.write_images()

        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        node.velocity_publisher.publish(twist)

        # node.bag.close()

        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.train()

    node.stop_robot()
    
    