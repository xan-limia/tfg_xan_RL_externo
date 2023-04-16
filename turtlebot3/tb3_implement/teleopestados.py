import rospy
import rosbag
import cv2, numpy, pyexiv2
import os, sys
import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

msg = """
Introduce la direccion
---------------------------
        w
   a         d
        s

q to quit
"""

TH_DIST_IMAGE = 650000


class TeleoperationNode:
    def __init__(self, foldername):
        rospy.init_node('teleoperation')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.turtlebot3_model = rospy.get_param("model", "burger")

        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 

        self.velocities = []
        self.image = None
        self.img_msg = None
        self.write = False

        self.stored_images = []
        self.stored_velocities = []

        self.folder = foldername

        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
    

    def image_callback(self, data):
        self.img_msg = data
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # img_resized = cv2.resize(self.image, (0, 0), fx=4, fy=4)
        # cv2.imshow("window", img_resized)
    
    def load_images(self):
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
            return
        for filename in os.listdir(self.folder):
            if(filename.endswith('.png')):
                img = cv2.imread(os.path.join(self.folder, filename))
                self.stored_images.append(img)

            img = pyexiv2.Image(os.path.join(self.folder, filename))
            metadata = img.read_exif()
            text = metadata['Exif.Photo.UserComment']               
            linear_vel = float(text.split("=")[1].split("\n")[0])
            angular_vel = float(text.split("=")[2])
            self.stored_velocities.append((linear_vel, angular_vel))




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
            return self.stored_velocities[min_idx]
        

    def stop_robot(self):
        print("No hay coincidencias, denetiendo robot\n")
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        node.velocity_publisher.publish(twist)

    def write_images(self):
        if self.write:
            if len(self.velocities) > 0 and self.image is not None:
                now = datetime.datetime.now()
                print(now)
                filename = f"{self.folder}/image_{now.strftime('%Y-%m-%d_%H-%M-%S')}.png"
                filepath = os.path.join(os.getcwd(), filename)
                cv2.imwrite(filepath, self.image)
                # Leer imagen
                img_data = pyexiv2.Image(filepath)
                metadata = img_data.read_exif()
                # Agregar metadato
                metadata['Exif.Photo.UserComment'] = f"linear={self.target_linear_vel}\nangular={self.target_angular_vel}"
                img_data.modify_exif(metadata)
                self.stored_images.append(self.image)
                self.stored_velocities.append((self.target_linear_vel, self.target_angular_vel))
                print("salvados", len(self.stored_images))
            self.img_msg = None
            self.image = None
            self.velocities = []
            self.write = False
        
    def teleop(self):
        self.load_images()
        while not rospy.is_shutdown():
            #cv2.waitKey(0)
            if self.image is not None:
                closest_velocity = self.find_closest_velocity()
            
                if closest_velocity is not None:
                    print("Se ha encontrado coincidiencia, adoptando velocidad correspondiente\n")
                    twist = Twist()

                    twist.linear.x = closest_velocity[0]; twist.linear.y = 0.0; twist.linear.z = 0.0

                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = closest_velocity[1]

                    self.velocity_publisher.publish(twist)
                else:
                    self.stop_robot()

                    key = input(msg)
                    if key.lower() == 'w':
                        self.target_linear_vel = 0.05
                        self.target_angular_vel = 0.0
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    elif key.lower() == 's':
                        self.target_linear_vel = -0.025
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    elif key.lower() == 'a':
                        self.target_angular_vel = 0.18
                        self.target_linear_vel = 0.05
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    elif key.lower() == 'd':
                        self.target_angular_vel = -0.18
                        self.target_linear_vel = 0.05
                        self.write = True
                        print(self.vels(self.target_linear_vel,self.target_angular_vel))
                    if key.lower() == 'q':
                        break

                    twist = Twist()

                    twist.linear.x = self.target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.target_angular_vel

                    self.velocities.append(twist)
                    self.velocity_publisher.publish(twist)

                    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    node = TeleoperationNode(foldername)
    node.teleop()

    node.stop_robot()
    
    