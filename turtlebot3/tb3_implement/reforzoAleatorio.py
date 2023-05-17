import rospy
import rosbag
import cv2, numpy, pyexiv2
import os, sys, signal
import datetime
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

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
W = 10
H = 7
X = 35
Y = 45

MODEL = 'turtlebot3_burger'

class TeleoperationNode:
    def __init__(self, foldername):
        rospy.init_node('teleoperation')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber('/camera/image', Image, self.image_callback)
        #self.turtlebot3_model = rospy.get_param("model", "burger")
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 

        self.velocities = []
        self.image = None
        self.img_msg = None
        self.write = False

        self.stored_images = []
        self.stored_velocities = []
        self.last_index_action = None
        self.penultimate_index_action = None

        self.folder = foldername

        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
    

    def image_callback(self, data):
        self.img_msg = data
        image_bridge = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = self.mask_images(image_bridge)
        # self.image = image_bridge

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
            self.stored_velocities.append((linear_vel, angular_vel))


    def check_ref_in_images(self, x, y, w, h, threshold):

        img_gris = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        umbral, img_binaria = cv2.threshold(img_gris, 127, 255, cv2.THRESH_BINARY)
        region = img_binaria[y:y+h, x:x+w]

        black_region = numpy.zeros((h, w), dtype=numpy.uint8)

        coincidences = cv2.compare(region, black_region, cv2.CMP_EQ)

        percentage = numpy.count_nonzero(coincidences) / coincidences.size
        print(percentage)
        
        return percentage >= threshold

    def find_closest_velocity(self):
        print("estados", len(self.stored_images))
       
        if len(self.stored_images) == 0:
            return None
        min_dist = float('inf')
        min_idx = -1
        for i, img in enumerate(self.stored_images):
            distance = numpy.sum((cv2.absdiff(self.image.flatten(), img.flatten()) ** 2))
            print(distance, i)
            if distance < min_dist:
                min_dist = distance
                min_idx = i

        #print(len(self.stored_images), min_idx, min_dist, self.stored_velocities[min_idx].linear.x, self.stored_velocities[min_idx].angular.z)
        if min_dist > TH_DIST_IMAGE:
            return None
        else:
            self.penultimate_index_action = self.last_index_action
            self.last_index_action = min_idx
            return self.stored_velocities[min_idx]
        
    def reset_position(self):
        model_state = ModelState()
        model_state.model_name = MODEL 
        model_state.pose.position.x = 0.244979
        model_state.pose.position.y = -1.786919
        model_state.pose.position.z = -0.001002
        self.set_state(model_state)

    def stop_robot(self):
        print("No hay coincidencias, denetiendo robot\n")
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        node.velocity_publisher.publish(twist)

    def append_states(self):
        if len(self.velocities) > 0 and self.image is not None:
            self.stored_images.append(self.image)
            self.stored_velocities.append((self.target_linear_vel, self.target_angular_vel))
            self.penultimate_index_action = self.last_index_action
            self.last_index_action = len(self.stored_images) - 1
            print("salvados", len(self.stored_images))
        self.img_msg = None
        self.image = None
        self.velocities = []
        self.write = False

    def write_images(self):

        for archivo in os.listdir(self.folder):
            ruta_archivo = os.path.join(self.folder, archivo)
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
            metadata['Exif.Photo.UserComment'] = f"linear={self.stored_velocities[i][0]}\nangular={self.stored_velocities[i][1]}"
            img_data.modify_exif(metadata)
            
    
    def teleop(self):
        self.load_images()
        while not rospy.is_shutdown():

            if self.image is not None:

                result = self.check_ref_in_images(x=X, y=Y, w=W, h=H, threshold=TH_R_IMAGE)
                if(result == False):
                    print("refuerzo negativo, reseteando posicion\n")
                    if self.last_index_action is not None: # and len(self.stored_images) > 1 and self.last_index_action != self.penultimate_index_action:
                        #del self.stored_images[self.penultimate_index_action]
                        del self.stored_images[self.last_index_action]
                        #del self.stored_velocities[self.penultimate_index_action]
                        del self.stored_velocities[self.last_index_action]
                       
                    self.reset_position()

                closest_velocity = self.find_closest_velocity()
            
                if closest_velocity is not None:
                    print("Se ha encontrado coincidiencia, adoptando velocidad correspondiente\n")
                    twist = Twist()

                    twist.linear.x = closest_velocity[0]; twist.linear.y = 0.0; twist.linear.z = 0.0

                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = closest_velocity[1]

                    self.velocity_publisher.publish(twist)
                else:
                    self.stop_robot()

                    random_action = random.randint(1, 5)

                    if random_action == 1:
                        self.target_linear_vel = 0.05
                        self.target_angular_vel = 0.0
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    elif random_action == 2:
                        self.target_angular_vel = 0.18
                        self.target_linear_vel = 0.05
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    elif random_action == 3:
                        self.target_angular_vel = 0.30
                        self.target_linear_vel = 0.05
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    elif random_action == 4:
                        self.target_angular_vel = -0.18
                        self.target_linear_vel = 0.05
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    elif random_action == 5:
                        self.target_angular_vel = -0.30
                        self.target_linear_vel = 0.05
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))

                    twist = Twist()

                    twist.linear.x = self.target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.target_angular_vel

                    self.velocities.append(twist)
                    self.velocity_publisher.publish(twist)


                    if self.write:
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
       
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.teleop()

    node.stop_robot()
    
    