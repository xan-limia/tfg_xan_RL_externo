VELOCITY_FACTOR = 1

# PIXELES
N_PX = 60*80

# THRESHOLDS
TH_DIST_IMAGE = 160000

# # AREA REFORZO (Sigue carril)
# W = 8
# H = 6
# X = 36
# Y = 52

# COLOR = 0
# TH_R_IMAGE = 0.8

# MASCARA
MASK = (3,0,5,5)

# AREA REFORZO (Sigue lineas) MASK = (3,0,5,5)
W = 48
H = 12
X = 16
Y = 47

COLOR = 255 # Color co que se mide o reforzo
TH_R_IMAGE = 0.05

# ACIONS
N_ACTIONS = 5
ACTIONS = [(0.15, 0.90), (0.15, 0.54), (0.15, 0.0), (0.15, -0.54), (0.15, -0.90)]
# ACTIONS = [(0.15, 1.20), (0.15, 0.90), (0.15, 0.54), (0.15, 0.0), (0.15, -0.54), (0.15, -0.90), (0.15, -1.20)]


msg = """
Posibles Accions 
---------------------------
1 Xiro Esquerda Brusco
2 Xiro Esquerda Suave
3 Avanzar Recto
4 Xiro Dereita Suave
5 Xiro Dereita Brusco

enter reinterar calcular
"""

# PARAMETROS ROBOT
MODEL = 'turtlebot3_burger'
TOPIC_VEL = '/cmd_vel'
TOPIC_CAMERA = '/camera/image'
TOPIC_MODEL_STATE = '/gazebo/model_states'
TOPIC_SET_MODEL_STATE = '/gazebo/set_model_state'
TOPIC_REINFORCEMENT = '/reinforcement'
TOPIC_IMG_MASK = '/img_mask'
TOPIC_STOP_ROBOT = '/stop_robot'
TOPIC_START_ROBOT = '/start_robot'