import rospy
import rosbag
import cv2, numpy, pyexiv2
import os, sys, signal, select
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios
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
# 0 Xiro Esquerda Brusco
# 1 Xiro Esquerda
# 2 Avanzar Recto
# 3 Xiro Dereita
# 4 Xiro Dereita Brusco

msg = """
Posibles Accions 
---------------------------
1 Xiro Esquerda Brusco
2 Xiro Esquerda
3 Avanzar Recto
4 Xiro Dereita
5 Xiro Dereita Brusco
"""

ACTIONS = 5

VELOCITY_FACTOR = 1/3

# THRESHOLDS
TH_DIST_IMAGE = 350000
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



class ManualNode:
    def __init__(self, foldername):

        rospy.init_node('manualTeleop')
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

        self.actions = [(0.15, 0.90), (0.15, 0.54), (0.15, 0.0), (0.15, -0.54), (0.15, -0.90)]


        self.folder = foldername

        now = datetime.datetime.now()
        bag_name = f"{self.folder}/{self.folder}_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.bag"
        self.bag_file = os.path.join(os.getcwd(), bag_name)
        self.bag = None

        self.number_states = 0

        self.linear_vel   = 0.0
        self.angular_vel  = 0.0

        self.write = False

    def getKey(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
    
    def position_callback(self, msg):
        robot_index = msg.name.index(MODEL)
        self.robot_position = msg.pose[robot_index]
    
    def image_callback(self, data):
        image_bridge = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # self.image = self.mask_images(image_bridge)
        self.image = image_bridge ## Descomentar para executar sen mascara

    def stop_robot(self):
        # print("No hay coincidencias, denetiendo robot\n")
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        node.velocity_publisher.publish(twist)


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

    def append_states(self):
        if self.image is not None:
            self.stored_images.append(self.image)
            self.state_action.append((self.linear_vel, self.angular_vel))
            self.last_index_action = len(self.stored_images) - 1
            print("salvados", len(self.stored_images))
        self.number_states = len(self.stored_images)
        self.write = False


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

    def teleop(self):
        self.load_images()
        self.bag = rosbag.Bag(self.bag_file, 'w')
        print(msg)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            # key = cv2.waitKey(1) & 0xff
            # if key == ord('1'):
            key = self.getKey()
            if key == '1':
                self.angular_vel = 0.90
                self.linear_vel = 0.15
                self.write = True
                action = 1
                
            # elif key == ord('2'):
            elif key == '2':
                self.angular_vel = 0.54
                self.linear_vel = 0.15
                self.write = True
                action = 2
                
            # elif key == ord('3'):
            elif key == '3':
                self.angular_vel = 0.00
                self.linear_vel = 0.15
                self.write = True
                action = 3
                
            # elif key == ord('4'):
            elif key == '4':
                self.angular_vel = -0.54
                self.linear_vel = 0.15
                self.write = True
                action = 4
               
            # elif key == ord('5'):
            elif key == '5':
                self.angular_vel = -0.90
                self.linear_vel = 0.15
                self.write = True
                action = 5
            else:
                if (key == '\x03'):
                    node.write_images()

                    node.stop_robot()
    
                    node.bag.close()
                    break
                
        
            # if key == ord('q'):
            #     break

            # if key == ord('r'):
            #     cv2.destroyWindow('window')
            #     cv2.namedWindow("window", 1) 

            twist = Twist()

            twist.linear.x = self.linear_vel * VELOCITY_FACTOR; twist.linear.y = 0.0; twist.linear.z = 0.0

            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular_vel * VELOCITY_FACTOR

            self.velocity_publisher.publish(twist)
        
            
            if self.write:
                message = String()
                message.data = f"action: {action}"
                topic = f"/action_{action}"
                self.bag.write(topic, message, current_time)
                self.append_states()
                self.image = None

            





if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    node = ManualNode(foldername)

    # def signal_handler(sig, frame):
    #     #print("Programa detenido")
    #     node.write_images()

    #     node.stop_robot()
    
    #     node.bag.close()

    #     exit(0)

    # signal.signal(signal.SIGINT, signal_handler)
    # signal.signal(signal.SIGTSTP, signal_handler)

    node.teleop()

    node.stop_robot()

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)






