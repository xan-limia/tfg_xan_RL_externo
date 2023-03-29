import rospy
import rosbag
import cv2, numpy
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

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


class TeleoperationNode:
    def __init__(self, filebagname):
        rospy.init_node('teleoperation')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.turtlebot3_model = rospy.get_param("model", "burger")

        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 

        self.velocities = []
        self.image = None
        self.img_msg = None
        self.bag_filename = os.path.join(os.getcwd(), filebagname)
        self.bag = None
        self.write = False

        self.stored_images = []
        self.stored_velocities = []

        self.folder = filebagname.replace(".bag", "")

        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
    
    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        if self.turtlebot3_model == "burger":
            vel = self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
        else:
            vel = self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

        return vel
    
    def checkAngularLimitVelocity(self, vel):
        if self.turtlebot3_model == "burger":
            vel = self.constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
        else:
            vel = self.constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

        return vel

    def image_callback(self, data):
        self.img_msg = data
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # img_resized = cv2.resize(self.image, (0, 0), fx=4, fy=4)
        # cv2.imshow("window", img_resized)
        
    def load_rosbag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/image', '/velocities']):
            if topic == '/image':
                imagecv2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.stored_images.append(imagecv2)
            else:
                self.stored_velocities.append(msg)

    def find_closest_velocity(self):
        if len(self.stored_images) == 0:
            return None
        min_dist = float('inf')
        min_idx = -1
        for i, img in enumerate(self.stored_images):
            if self.image is None:
                pass
            else:
                distance = numpy.sum((cv2.absdiff(self.image.flatten(), img.flatten()) ** 2))
                if distance < min_dist:
                    min_dist = distance
                    min_idx = i
        if min_dist > 100000:
            return None
        else:
            return self.stored_velocities[min_idx]
        
    def teleop(self):
        self.bag = rosbag.Bag(self.bag_filename, 'w')
        self.load_rosbag()
        while not rospy.is_shutdown():
            #cv2.waitKey(0)
            twist = self.find_closest_velocity()
            if twist is not None:
                print("Se ha encontrado coincidiencia, adoptando velocidad correspondiente\n")
                self.velocity_publisher.publish(twist)
            else:
                print("No hay coincidencias, denetiendo robot\n")
                twist = Twist()
                twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
                node.velocity_publisher.publish(twist)
                key = input(msg)
                if key.lower() == 'w':
                    self.target_linear_vel = 0.02
                    self.target_angular_vel = 0.0
                    self.write = True
                    print(self.vels(self.target_linear_vel,self.target_angular_vel))
                elif key.lower() == 's':
                    self.target_linear_vel = -0.02
                    self.write = True
                    print(self.vels(self.target_linear_vel,self.target_angular_vel))
                elif key.lower() == 'a':
                    self.target_angular_vel = 0.02
                    self.target_linear_vel = 0.02
                    self.write = True
                    print(self.vels(self.target_linear_vel,self.target_angular_vel))
                elif key.lower() == 'd':
                    self.target_angular_vel = 0.02
                    self.target_angular_vel = -0.02
                    self.write = True
                    print(self.vels(self.target_linear_vel,self.target_angular_vel))
                if key == ord('q'):
                    break

                twist = Twist()

                self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = self.control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.control_angular_vel

                self.velocities.append(twist)
                self.velocity_publisher.publish(twist)

                if self.write:
                    current_time = rospy.Time.now()
                    if len(self.velocities) > 0 and self.image is not None:
                        now = datetime.datetime.now()
                        print(now)
                        if not os.path.exists(self.folder):
                            os.makedirs(self.folder)
                        filename = f"{self.folder}/image_{now.strftime('%Y-%m-%d_%H-%M-%S')}.jpg"
                        filepath = os.path.join(os.getcwd(), filename)
                        cv2.imwrite(filepath, self.image)
                        self.stored_images.append(self.image)
                        self.stored_velocities.append(self.velocities[-1])
                        self.bag.write('/image', self.img_msg, current_time)
                        self.bag.write('/velocities', self.velocities[-1], current_time)
                    self.img_msg = None
                    self.image = None
                    self.velocities = []
                    self.write = False

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre del archivo bag")
        sys.exit()
    filebagname = sys.argv[1]

    node = TeleoperationNode(filebagname)
    node.teleop()

    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    node.velocity_publisher.publish(twist)
    node.bag.close()
    