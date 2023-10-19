import rosbag
import numpy as np
import sys, os
import matplotlib.pyplot as plot
from cv_bridge import CvBridge
import cv2

def vel_hist(bag):
    velocities = []

    topic_angular = '/angular'

    for topic, msg, time in bag.read_messages(topics=[topic_angular]):
        velocities.append(msg.data)

    bag.close()

    plot.hist(velocities, bins=10)
    plot.xlabel('Velocity Value')
    plot.ylabel('Frequency')
    plot.title('Angular velocities')
    plot.grid(True)

    plot.show()

def save_images_from_bag(bag, bag_name):
    bridge = CvBridge()


    topic_images = '/images'
    os.makedirs(bag_name, exist_ok=True)
    for topic, msg, t in bag.read_messages(topics=[topic_images]):
        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        image_name = os.path.join(bag_name, f'img_{t}.png')

        cv2.imwrite(image_name, image)
    
def average_px(bag_name):
    images = []

    for filename in os.listdir(bag_name):
        if filename.endswith('.png'):
            image = cv2.imread(os.path.join(bag_name, filename))
            images.append(image)

    image_sum = np.zeros_like(images[0], dtype=np.float64)

    for image in images:
        image_sum += image

    average = (image_sum / len(images)).astype(np.uint8)

    # resize = cv2.resize(average, (320, 240))
    cv2.imwrite(os.path.join('result_analize_images', 'average_px_robot_real.png'), average)
    # cv2.imshow('result', resize)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

def color_filter(bag_name):
    images = []

    for filename in os.listdir(bag_name):
        if filename.endswith('.png'):
            image = cv2.imread(os.path.join(bag_name, filename))
            images.append(image)

    low_y = np.array([20, 150, 150], dtype=np.uint8)
    up_y = np.array([70, 255, 255], dtype=np.uint8)

    # low_y = np.array([200, 200, 200], dtype=np.uint8)
    # up_y = np.array([255, 255, 255], dtype=np.uint8)

    heatmap = np.zeros(images[0].shape[:2], dtype=np.uint8)

    for image in images:
        y_mask = cv2.inRange(image, low_y, up_y)
        # heatmap += y_mask
        heatmap = cv2.add(heatmap, y_mask)

    heatmap_resize = cv2.resize(heatmap, (320, 240))
    cv2.imwrite(os.path.join('result_analize_images', 'color_filter_robot_real.png'), heatmap)
    # cv2.imshow('result', heatmap_resize)
    # # cv2.imshow('original', images[0])
    # cv2.waitKey()
    # cv2.destroyAllWindows()

MASK = (0,0,0,0)

def mask_images(img, mask = MASK):
        h, w = img.shape[:2]
        
        if mask[0] == 0:
            up = 0
        else:
            up = int(h/mask[0])

        if mask[1] == 0:
            down = 0
        else:
            down = int(h/mask[1])

        if mask[2] == 0:
            left = 0
        else:
            left = int(w/mask[2])

        if mask[3] == 0:
            right = 0
        else:
            right = int(w/mask[3])
       
        maskImg = np.zeros((h - up - down, w - left - right))
        maskImg = img[up:h - down, left:w - right]

        # maskImg = numpy.zeros_like(img)
        # maskImg[up:h - down, left:w - right] = img[up:h - down, left:w - right]

        return maskImg

# N_PX = 160*120
N_PX = 80*60

def dist_hist(bag_name):
    images = []

    for filename in os.listdir(bag_name):
        if filename.endswith('.png'):
            image = cv2.imread(os.path.join(bag_name, filename))
            images.append(image)

    distances = np.zeros((len(images), len(images)))
    average_distances = np.zeros(len(images))
    for i in range(len(images)):
        maskImg1 = mask_images(img=images[i])
        total_distance = 0.0
        for j in range(len(images)):
            if i != j:
                maskImg2 = mask_images(img=images[j])
                distance = np.sum((cv2.absdiff(maskImg1.flatten(), maskImg2.flatten()) ** 2))
                # distances[i][j] = distance
                # total_distance += distance

                distances[i][j] = distance/N_PX
                total_distance += distance/N_PX

        average_distance = total_distance / (len(images) - 1)
        average_distances[i] = average_distance

    plot.figure(1)
    plot.hist(distances, bins=5)
    plot.xlabel('Distance')
    plot.ylabel('Frequency')
    plot.title('Euclidean distances')
    plot.grid(True)

    plot.figure(2)
    plot.hist(average_distances, bins=5)
    plot.xlabel('Average Distance')
    plot.ylabel('Frequency')
    plot.title('Euclidean distances')
    plot.grid(True)

    plot.show()

if __name__ == '__main__':
    if len(sys.argv) >= 1:
        bagfile = sys.argv[1]

    bag = rosbag.Bag(bagfile, 'r')
    bag_name = bagfile.split('.bag')[0]

    # vel_hist(bag=bag)

    # save_images_from_bag(bag=bag, bag_name=bag_name)

    # average_px(bag_name=bag_name)
    
    # color_filter(bag_name=bag_name)

    dist_hist(bag_name)
