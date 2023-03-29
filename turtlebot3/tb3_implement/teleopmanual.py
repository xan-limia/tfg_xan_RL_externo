import rospy
import rosbag
import cv2
import os, sys
import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a         d
        s

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key : force stop

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
        #self.rate = rospy.Rate(10)
        self.velocities = []
        self.image = None
        self.img_msg = None
        self.bag_filename = os.path.join(os.getcwd(), filebagname)
        self.bag = None
        self.write = False

        self.folder = filebagname.replace(".bag", "")

        self.status = 0
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
        img_resized = cv2.resize(self.image, (0, 0), fx=4, fy=4)
        cv2.imshow("window", img_resized)
        # cv2.imshow("window", self.image)


    def teleop(self):
        self.bag = rosbag.Bag(self.bag_filename, 'w')
        print(msg)
        while not rospy.is_shutdown():
            
            key = cv2.waitKey(1) & 0xff
            if key == ord('w'):
                self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
                self.status = self.status + 1
                self.write = True
                print(self.vels(self.target_linear_vel,self.target_angular_vel))
            elif key == ord('s'):
                self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
                self.fstatus = self.status + 1
                self.write = True
                print(self.vels(self.target_linear_vel,self.target_angular_vel))
            elif key == ord('a'):
                self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
                self.status = self.status + 1
                self.write = True
                print(self.vels(self.target_linear_vel,self.target_angular_vel))
            elif key == ord('d'):
                self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
                self.status = self.status + 1
                self.write = True
                print(self.vels(self.target_linear_vel,self.target_angular_vel))
            elif key == ord(' '):
                self.target_linear_vel   = 0.0
                self.control_linear_vel  = 0.0
                self.target_angular_vel  = 0.0
                self.control_angular_vel = 0.0
                print(self.vels(self.target_linear_vel, self.target_angular_vel))
            
            if key == ord('q'):
                break

            if key == ord('r'):
                cv2.destroyWindow('window')
                cv2.namedWindow("window", 1) 


            if self.status == 20 :
                print(msg)
                self.status = 0

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