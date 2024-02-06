from utilities import *

# PARAMETROS QLEARNING
Q_VALUE_DEFAULT = 0.01

# POLITICA E-GREEDY
EPSILON = 0.00

# PARAMETROS Q-LEARNING
LEARNING_RATE = 0.1 
DISCOUNT_FACTOR = 0.9

def save_images_from_bag(bag):
    bridge = CvBridge()

    images = []
    velocities  =[]

    topic_images = '/images'
    topic_vel = '/angular'

    for topic, msg, t in bag.read_messages(topics=[topic_images]):
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        images.append(image)

    for topic, msg, t in bag.read_messages(topics=[topic_vel]):
        velocities.append(msg.data)

    return images, velocities

def make_states(images):
    states = []
    current_state = None
    for image in images:
        current_state = find_closest_state(image=image, stored_images=states)
        reward = check_ref_in_images(image=image)
        if current_state is None and reward == POSITIVE_REWARD:
            init_q_values = [random.uniform(0, 0.01) for _ in range(N_ACTIONS)] # inicializar de forma aleatoria 0 e 0.1
            states.append((image, init_q_values))

    return states

def action_check(velocity, actions = ACTIONS):
    vel_dist = []
    for action in actions:
        vel_dist.append(abs(velocity - action[1]))
    return vel_dist.index(min(vel_dist))


def train_offpolicy(states, images, velocities):

    current_state = None

    for i, image in enumerate(images):
        reward = check_ref_in_images(image=image)
        current_state = find_closest_state(image=image, stored_images=states)

        if current_state is not None:
            action = action_check(velocity=velocities[i])
            current_q_value = states[current_state][1][action]
            if i+1 < len(images):
                next_state = find_closest_state(image=images[i+1], stored_images=states)
            else:
                next_state = find_closest_state(image=images[0], stored_images=states)
               
            new_q_values = states[next_state][1]   

            max_q_value = numpy.amax(new_q_values)

            new_q_value = current_q_value + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_q_value - current_q_value)
            states[current_state][1][action] = new_q_value

    return states

def write_images(states, foldername):
    os.makedirs(foldername, exist_ok=True)
    for state in states:
            now = datetime.datetime.now()
            filename = f"{foldername}/image_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.png"
            filepath = os.path.join(os.getcwd(), filename)
            cv2.imwrite(filepath, state[0])
            # Leer imagen
            img_data = pyexiv2.Image(filepath)
            metadata = img_data.read_exif()
            # Agregar metadato
            metadata['Exif.Photo.UserComment'] = f"q_values={state[1]}"
            img_data.modify_exif(metadata)





            



if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Introduce el archivo bag y el nombre de un directorio')
        
    bagfile = sys.argv[1]
    bag = rosbag.Bag(bagfile, 'r')
    bag_name = bagfile.split('.bag')[0]  

    foldername = sys.argv[2]  

    images = []
    velocities = []
    states = []

    images, velocities = save_images_from_bag(bag=bag)

    states = make_states(images=images)

    states = train_offpolicy(states=states, images=images, velocities=velocities)

    write_images(states=states, foldername=foldername)






