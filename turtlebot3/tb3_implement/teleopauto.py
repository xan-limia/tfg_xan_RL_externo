import rospy
import cv2, numpy
import os, sys
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from scipy.spatial import distance


class AutonomousNode:
    def __init__(self, bag_filename):
        rospy.init_node('autonomous')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_subscriber = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.bridge = CvBridge()
        cv2.namedWindow("window", 1) 
        self.velocities = []
        self.image = None
        self.bag_filename = bag_filename
        self.bag = None
        self.stored_images = []
        self.stored_velocities = []

    def load_rosbag(self):
        self.bag = rosbag.Bag(self.bag_filename)
        for topic, msg, t in self.bag.read_messages(topics=['/image', '/velocities']):
            if topic == '/image':
                imagecv2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.stored_images.append(imagecv2)
            else:
                self.stored_velocities.append(msg)
        self.bag.close()

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img_resized = cv2.resize(self.image, (0, 0), fx=4, fy=4)
        cv2.imshow("window", img_resized)
        # cv2.imshow("window", self.image)

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
        return self.stored_velocities[min_idx]

    def teleop(self):
        self.load_rosbag()
        while not rospy.is_shutdown():
            key = cv2.waitKey(1) & 0xff
            twist = self.find_closest_velocity()
            if twist is not None:
                self.velocity_publisher.publish(twist)
            if key == ord('q'):
                break
            if key == ord('r'):
                cv2.destroyWindow('window')
                cv2.namedWindow("window", 1)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre del archivo bag")
        sys.exit()
    filebagname = sys.argv[1]
    node = AutonomousNode(filebagname)
    node.teleop()