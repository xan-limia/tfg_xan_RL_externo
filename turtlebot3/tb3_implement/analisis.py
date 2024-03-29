import rosbag
import numpy as np
import sys, os
import matplotlib.pyplot as plot
from cv_bridge import CvBridge
import cv2
import rospy
import re

def vel_hist(bag):
    velocities = []

    topic_angular = '/angular'

    for topic, msg, time in bag.read_messages(topics=[topic_angular]):
        velocities.append(msg.data)

    bag.close()

    plot.hist(velocities, bins=11)
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

    cv2.imwrite(os.path.join('result_analize_images', 'average_px_robot_real_1.png'), average)

    # resize = cv2.resize(average, (320, 240))
    # cv2.imshow('result', resize)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

def color_filter(bag_name):
    images = []

    for filename in os.listdir(bag_name):
        if filename.endswith('.png'):
            image = cv2.imread(os.path.join(bag_name, filename))
            images.append(image)

    # low_y = np.array([20, 150, 150], dtype=np.uint8)
    # up_y = np.array([70, 255, 255], dtype=np.uint8)

    low_y = np.array([200, 200, 200], dtype=np.uint8)
    up_y = np.array([255, 255, 255], dtype=np.uint8)

    heatmap = np.ones(images[0].shape[:2], dtype=np.uint8) * 255

    for image in images:
        y_mask = cv2.inRange(image, low_y, up_y)
        # heatmap += y_mask
        heatmap = cv2.subtract(heatmap, y_mask)

    cv2.imwrite(os.path.join('result_analize_images', 'color_filter_robot_real_1.png'), heatmap)

    # heatmap_resize = cv2.resize(heatmap, (320, 240))
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

def dist_hist(bag_name):
    images = []

    for filename in os.listdir(bag_name):
        if filename.endswith('.png'):
            image = cv2.imread(os.path.join(bag_name, filename))
            images.append(image)
    n_images = len(images)
    distances = np.zeros((n_images, n_images))
    average_distances = np.zeros(n_images)
    for i in range(n_images):
        maskImg1 = mask_images(img=images[i])
        h, w = maskImg1.shape[:2]
        n_px = h * w
        total_distance = 0.0
        for j in range(n_images):
            if i != j:
                maskImg2 = mask_images(img=images[j])
                distance = np.sum((cv2.absdiff(maskImg1.flatten(), maskImg2.flatten()) ** 2))
                distances[i][j] = distance / n_px
                total_distance += distance / n_px

        average_distance = total_distance / (n_images - 1)
        average_distances[i] = average_distance

    plot.figure(1)
    plot.hist(distances, bins=5)
    plot.xlabel('Distance')
    plot.ylabel('Frequency')
    plot.title(f'Euclidean distances, Nº Img= {n_images}, MASK = {MASK}')
    plot.grid(True)

    plot.figure(2)
    plot.hist(average_distances, bins=5)
    plot.xlabel('Average Distance')
    plot.ylabel('Frequency')
    plot.title(f'Euclidean distances, Nº Img = {n_images},  MASK = {MASK}')
    plot.grid(True)

    plot.show()

def make_mask_ref(img, mask, ref_area, imgresult):
    image = cv2.imread(img)
    h, w = image.shape[:2]
        
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
    
    maskImg = np.zeros_like(image)
    maskImg[up:h - down, left:w - right] = image[up:h - down, left:w - right]

   
    cv2.rectangle(maskImg, (ref_area[2], ref_area[3]), (ref_area[2] + ref_area[0], ref_area[3] + ref_area[1]), (0, 0, 255), 1)
    cv2.imshow("Imagen con rectángulo rojo", maskImg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cv2.imwrite(imgresult, maskImg)

def train_times(bag):

    topic_ref = '/reinforcement'
    topic_finish = '/finish_train'

    info = bag.get_type_and_topic_info()
    train_time = rospy.Time(0)
    last_time = rospy.Time(0)

    if topic_finish in info.topics:
        for _, _, time in bag.read_messages(topics=[topic_ref]):
            train_time = time

    else:
        for _, _, time in bag.read_messages(topics=[topic_ref]):
            train_time = last_time
            last_time = time

    start = bag.get_start_time()
    end = bag.get_end_time()
    total_time = end- start

    final_time = end - train_time.to_sec()

    print("tempo total = ", total_time)     
    print("tempo entrenamento = ", train_time.to_sec())
    print("tempo final = ", final_time)
    
    bag.close()

def draw_trajectory(bag):
    coords_x = []
    coords_y = []

    state_topics = [topic for topic in bag.get_type_and_topic_info().topics.keys() if topic.startswith('/state_')]
    coords = []

    for topic in state_topics:
        for _, msg, time in bag.read_messages(topics=[topic]):
            search_digits = re.search(r"x:\s*([-+]?\d*\.\d+|\d+),\s*y:\s*([-+]?\d*\.\d+|\d+)", msg.data)
            if search_digits:
                x = float(search_digits.group(1))
                y = float(search_digits.group(2))
                coords.append((x, y, time))

    sorted_coords = sorted(coords, key=lambda x: x[2])

    coords_x, coords_y, _ = zip(*sorted_coords)

    plot.plot(coords_x, coords_y, marker='o', linestyle='-')
    plot.xlabel('X')
    plot.ylabel('Y')
    plot.grid(True)
    plot.show()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        bagfile = sys.argv[1]
        bag = rosbag.Bag(bagfile, 'r')
        bag_name = bagfile.split('.bag')[0]

    

    # vel_hist(bag=bag)

    # save_images_from_bag(bag=bag, bag_name=bag_name)

    # average_px(bag_name=bag_name)
    
    # color_filter(bag_name=bag_name)

    # dist_hist(bag_name)

    # make_mask_ref('result_analize_images/and_or_y5y6.png', (3,0,5,5), (48,40,16,20), 'exemplo_area_ref_w48_h40_x16_y20_sl.png')
    
    train_times(bag)
    
    # draw_trajectory(bag)
