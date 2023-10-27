
# import utilities as u
from utilities import *

# Ctrl+C to quit

# msg = """
# Posibles Accions 
# ---------------------------
# 1 Xiro Esquerda Mais Brusco
# 2 Xiro Esquerda Brusco
# 3 Xiro Esquerda Suave
# 4 Avanzar Recto
# 5 Xiro Dereita Suave
# 6 Xiro Dereita Brusco
# 7 Xiro Dereita Mais Brusco

# enter reinterar calcular
# """

class ManualNode(TrainingNode):
    def __init__(self, foldername):

        rospy.init_node('randomLearning')
        TrainingNode.__init__(self, foldername=foldername)

        self.state_action = []
    
    def load_images(self):
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
            return
        for filename in os.listdir(self.folder):
            if(filename.endswith('.png')):
                img = cv2.imread(os.path.join(self.folder, filename))
                
                self.stored_images.append((img, datetime.datetime.now()))

                img = pyexiv2.Image(os.path.join(self.folder, filename))
                metadata = img.read_exif()
                text = metadata['Exif.Photo.UserComment']               
                linear_vel = float(text.split("=")[1].split("\n")[0])
                angular_vel = float(text.split("=")[2])
                self.state_action.append((linear_vel, angular_vel))
        
        self.number_states = len(self.stored_images)
    

    def append_states(self, action):
        now = datetime.datetime.now()
        if self.image is not None:
            self.stored_images.append((self.image, now))
            self.state_action.append(action)
            self.current_state = len(self.stored_images) - 1
            # print("salvados", len(self.stored_images))
        self.number_states = len(self.stored_images)

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
            if isinstance(self.state_action[i], tuple):
                linear = self.state_action[i][0]
                angular = self.state_action[i][1]
            elif isinstance(self.state_action[i], int):
                linear = self.actions[self.state_action[i]][0]
                angular = self.actions[self.state_action[i]][1]
            metadata['Exif.Photo.UserComment'] = f"linear={linear}\nangular={angular}"
            img_data.modify_exif(metadata)

    # def sav_image(self, prefix):
    #     now = datetime.datetime.now()
    #     filename = f"{self.folder}/{prefix}_image_{now.strftime('%Y-%m-%d_%H-%M-%S-%f')}.png"
    #     filepath = os.path.join(os.getcwd(), filename)
    #     self.lastSavedFilename = filepath
    #     cv2.imwrite(filepath, self.image)
    #     # Leer imagen
    #     # img_data = pyexiv2.Image(filepath)
    #     # metadata = img_data.read_exif()
    #     # # Agregar metadato
    #     # metadata['Exif.Photo.UserComment'] = f"linear={self.state_action[i][0]}\nangular={self.state_action[i][1]}"
    #     # img_data.modify_exif(metadata)
    
    def train(self):
        self.load_images()
        self.bag = rosbag.Bag(self.bag_file, 'w')
        #  first = True
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            if self.isfinish == 3: # entrenamento terminado
                break
            
            if self.image is not None:
                
                # Comentar esta parte para realizar probas sin detectar reforzo
                result = check_ref_in_images(image=self.image)
                if result == NEGATIVE_REWARD :
                    self.isfinish = 0
                    
                    message = String()
                    message.data = f"negative reinforcement detected in state {self.current_state}"
                    self.reinforcement_publisher.publish(message)
                    self.bag.write(TOPIC_REINFORCEMENT, message, current_time)
                    print(message.data)
                    # self.sav_image("reinf")
                    
                    if self.current_state is not None: 
                        del self.stored_images[self.current_state]
                        del self.state_action[self.current_state]

                    if IS_SIM_ROBOT:
                        self.reset_position()
                    else:
                        self.stop_robot()

                    self.image = None


                elif IS_SIM_ROBOT:
                    self.append_pos() # engadir nova posicion a cola

                
                while self.image is None:
                    pass
                #######################################################################
               
                self.current_state = find_closest_state(self.image, self.stored_images)
            
                if self.current_state is not None:
                    message = String()
                    if IS_SIM_ROBOT:
                        message.data = f"{self.current_state}, x: {self.robot_position.position.x}, y: {self.robot_position.position.y}, z: {self.robot_position.position.z}"
                    topic = f"/state_{str(self.current_state).zfill(2)}"
                    self.bag.write(topic, message, current_time)

                    self.execute_action(self.state_action[self.current_state])

                    if IS_SIM_ROBOT:
                        self.check_finish_pos()

                    self.image = None
                else:
                    self.stop_robot()

                    key = input(msg)
                    if key == "":
                        pass
                    elif int(key) < 1 or int(key) > self.n_actions:
                        print("Tecla no valida\n")
                    else:

                        self.isfinish = 0
                        if self.robot_position is not None:
                            self.finish_position = self.robot_position
                            
                        action = int(key) - 1
                        self.execute_action(action=action)
                        
                        message = String()
                        message.data = f"action: {action}"
                        topic = f"/action_{action}"
                        self.bag.write(topic, message, current_time)

                        topic = "/images"
                        self.bag.write(topic, self.img_msg, current_time)
                        
                        self.append_states(action)

                        self.image = None
                        self.write = False

                    

                    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Debe ingresar el nombre de la carpeta a utilizar")
        sys.exit()
    foldername = sys.argv[1]

    node = ManualNode(foldername)

    def signal_handler(sig, frame):
        #print("Programa detenido")
        node.write_images()

        node.stop_robot()
    
        node.bag.close()

        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    node.train()

    node.stop_robot()

    node.write_images() 

    node.bag.close() # cerrar ficheiro bag
    
    