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
from std_msgs.msg import String, Int16

import utilities as u


# PIXELES

N_PX = 60*80


Q_VALUE_DEFAULT = 0.01

# POLITICAE-GREEDY
EPSILON = 0.00

# PARAMETROS Q-LEARNING
LEARNING_RATE = 0.1 
DISCOUNT_FACTOR = 0.9

class QLNode(u.TrainingNode):
    def __init__(self, foldername):
        rospy.init_node('qlearning')
        u.TrainingNode.__init__(self, foldername=foldername)
        
        self.q_values = []

    def check_default_action(self, angular_vel):
        if angular_vel == 1.20:
            action = 0
        elif angular_vel == 0.90:
            action = 1
        elif angular_vel == 0.54:
            action = 2
        elif angular_vel == 0.0:
            action = 3
        elif angular_vel == -0.54:
            action = 4
        elif angular_vel == -0.90:
            action = 5
        elif angular_vel == -1.20:
            action = 6
        return action
    
    
    def load_images(self):
        if not os.path.exists(self.folder):
            os.makedirs(self.folder) # crear directorio se non existe
            return
        for filename in os.listdir(self.folder):
            if(filename.endswith('.png')):
                img = cv2.imread(os.path.join(self.folder, filename))
                self.stored_images.append((img, datetime.datetime.now())) # Gardar imaxe sen mascara

                img = pyexiv2.Image(os.path.join(self.folder, filename)) # ler a imaxe con pyexiv para acceder os metadatos
                metadata = img.read_exif() # ler metadato

                # Transformar string nun vector q values
                text = metadata['Exif.Photo.UserComment']  
                if text.startswith("q_values"):
                    q_values = eval(text.split("=")[1])
                    self.q_values.append(q_values)
                elif text.startswith("linear"):
                    angular_vel = float(text.split("=")[2])
                    action = self.check_default_action(angular_vel=angular_vel)
                    init_q_values = [random.uniform(0, 0.01) for _ in range(self.n_actions)] # inicializar de forma aleatoria 0 e 0.1
                    init_q_values[action] = Q_VALUE_DEFAULT
                    self.q_values.append(init_q_values) # q values novo estado
                else:
                    init_q_values = [random.uniform(0, 0.01) for _ in range(self.n_actions)] # inicializar de forma aleatoria 0 e 0.1
                    self.q_values.append(init_q_values) # q values novo estado
        self.number_states = len(self.stored_images)

    ## ACCIONS

    def random_action(self):
        action = numpy.random.randint(self.n_actions)
        return action
    
    def best_action(self):
        q_values = self.q_values[self.current_state]
        action = numpy.argmax(q_values)
        return action

    def select_action(self):
        if numpy.random.random() < EPSILON:
            action = self.random_action()  # Explorar mais ao principio
        else:
            action = self.best_action()
        return action
    
    def append_states(self):
        now = datetime.datetime.now()
        if self.image is not None:
            self.stored_images.append((self.image, now)) # novo estado
            init_q_values = [random.uniform(0, 0.01) for _ in range(self.n_actions)] # inicializar de forma aleatoria 0 e 0.1
            self.q_values.append(init_q_values) # q values novo estado
        self.number_states = len(self.stored_images)

    ## ACTUALIZAR Q VALUES

    def update_q_values(self, reward, new_state):
        current_q_value = self.q_values[self.current_state][self.last_action]

        new_q_values = self.q_values[new_state]
        # print(self.q_values[self.current_state])
        print([ "{:0.2f}".format(x) for x in self.q_values[self.current_state] ])
        max_q_value = numpy.amax(new_q_values)

        new_q_value = current_q_value + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_q_value - current_q_value)
        self.q_values[self.current_state][self.last_action] = new_q_value
    
    ## GARDAR IMAXES EN DISCO
    def write_images(self):

        for archivo in os.listdir(self.folder):
            ruta_archivo = os.path.join(self.folder, archivo)
            if ruta_archivo.endswith(".bag"):
                pass
            else:
                os.remove(ruta_archivo)

        for i, img in enumerate(self.stored_images):
            filename = f"{self.folder}/image_{img[1].strftime('%Y-%m-%d_%H-%M-%S-%f')}.png"
            filepath = os.path.join(os.getcwd(), filename)
            self.lastSavedFilename = filepath
            cv2.imwrite(filepath, img[0])
            # Leer imagen
            img_data = pyexiv2.Image(filepath)
            metadata = img_data.read_exif()
            # Agregar metadato
            metadata['Exif.Photo.UserComment'] = f"q_values={self.q_values[i]}"
            img_data.modify_exif(metadata)
            
    ## ENTRENAR
    def train(self):
        self.load_images()
        self.bag = rosbag.Bag(self.bag_file, 'w')
        
        while not rospy.is_shutdown():
            
            if self.isfinish == 3: # entrenamento terminado
                break


            current_time = rospy.Time.now()
            if self.image is not None and self.stop_manual == 0:
                
                self.current_state = u.find_closest_state(image=self.image, stored_images=self.stored_images) # encontrar estado actual
                print("estado actual: ", self.current_state)

                # first_frame = self.image
            
                if self.current_state is not None:
                    message = String()
                    message.data = f"{self.current_state}, x: {self.robot_position.position.x}, y: {self.robot_position.position.y}, z: {self.robot_position.position.z}"
                    topic = f"/state_{str(self.current_state).zfill(2)}"
                    self.bag.write(topic, message, current_time)
                    
                    action = self.select_action() # seleccionar accion

                    self.execute_action(action=action) # executar accion selecionada
                    print("accion: ", action)

                    message.data = f"action: {action},  state: {self.current_state}"
                    topic = f"/action_{action}"
                    self.bag.write(topic, message, current_time)

                    self.last_action = action 
                    
                    self.image = None

                    while self.image is None:
                        pass

                    reward = u.check_ref_in_images(image=self.image) # obter recompensa

                    # last_frame = self.image

                    # distance = numpy.sum((cv2.absdiff(first_frame.flatten(), last_frame.flatten()) ** 2))

                    new_state = u.find_closest_state(self.image, self.stored_images) # obter o estado despois de executar a accion
                    
                    if new_state is None: # estado novo (crear)
                        print(new_state)
                        # self.stop_robot() # eliminar no real
                        self.append_states()
                        new_state = u.find_closest_state(self.image, self.stored_images)

                    print("estado siguiente: ", new_state)
                    

                    if new_state != self.current_state or reward == -1:
                        self.update_q_values(reward, new_state) # actualizar q_values

                    if reward == -1: # recompensa negativa
                        self.isfinish = 0
                        
                        message = String()
                        now = datetime.datetime.now()
                        message.data = f"{now.strftime('%m-%d_%H-%M-%S')}: negative reinforcement detected in state {self.current_state} when applying action {action}"
                        self.reinforcement_publisher.publish(message)
                        self.bag.write(u.TOPIC_REINFORCEMENT, message, current_time)
                        
                        self.reset_position() # reiniciar a ultima posicion gardada
                        
                    else:
                        self.append_pos() # engadir nova posicion a cola

                    
                    self.check_finish_pos() # comprobar se terminamos o entrenamento

                    self.image = None # reiniciar a imaxe capturada para obligar a ter unha nova

                else:
                    self.stop_robot()

                    self.append_states() # crear novo estado

                    topic = "/images"
                    self.bag.write(topic, self.img_msg, current_time)

                    self.image = None
            else:
                self.image = None

                    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    node = QLNode(foldername)

    def signal_handler(sig, frame):
        #print("Programa detenido")
        node.write_images() # escribir imaxes en disco

        node.stop_robot() # parar robot

        node.bag.close() # cerrar ficheiro bag

        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.train() # executar entrenamento

    node.stop_robot()

    node.write_images() 

    node.bag.close() # cerrar ficheiro bag
    
    