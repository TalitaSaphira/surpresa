#!/usr/bin/env python3

"""
Universidade Federal de Minas Gerais (UFMG) - 2023
Laboratorio CORO
Contact:
Joao Baiao, <baiaojfr.eng@gmail.com>
"""

# Classe com métodos gerais para navegação 

import rospy
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
from scipy.linalg import norm
from scipy.interpolate import interp1d
import numpy as np


class Navigation_class:
    def __init__(self):
        # Navigation
        self.desired_velocity_b = [0.0 , 0.0]
        self.desired_point_w = [0.0, 0.0]
        self.dist_ang = 0.0
        self.dist = 0.0
        self.origin_odom = [0.0, 0.0]
        self.C = []

        # Odometria
        self.pos = np.array([0.0, 0.0])
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        #ROS
        # rospy.init_node('Navigation_node')
        self.freq = 30
        self.time_last = rospy.Time.now().to_sec()
        self.time_step = 0.1
        self.read_params()
        self.odom_sub = rospy.Subscriber(self.odometry_topic_name, PoseWithCovarianceStamped, self.pose_cb)
        self.inputs_pub = rospy.Publisher(self.input_topic_name, Float32MultiArray, queue_size=10)

    def read_params(self):
        # Obtain the parameters
        # Robot
        self.l = float(rospy.get_param("~l", 0.235145))
        self.w = float(rospy.get_param("~w", 0.12854)) 
        self.r = float(rospy.get_param("~r", 0.50))    

        # Control Parameters
        self.Kv = float(rospy.get_param("~speed_gain", 0.10))   
        self.Krot = float(rospy.get_param("~speed_rot_gain", 0.50))  
        self.tol_trans = float(rospy.get_param("~position_tolerance", 0.1)) 
        self.tol_angle = float(rospy.get_param("~orientation_tolerance", 1))  

        # Arena
        self.square_side = float(rospy.get_param("~square_side", 1.0)) 

        # Topics
        self.input_topic_name = (rospy.get_param("~input_topic_name", "robot/input")) 
        self.input_topic_msg_type = (rospy.get_param("~input_topic_msg_type", "Float32MultiArray")) 
        self.odometry_topic_name = (rospy.get_param("~odometry_topic_name", "gt")) 
        self.odometry_topic_msg_type = (rospy.get_param("~odometry_topic_msg_type", "Odometry")) 
        self.path_topic_name = (rospy.get_param("~path_topic_name", "path")) 

        # Vector Field Parameters
        self.beta = int(rospy.get_param("~beta", 1)) 
        self.closed_path_flag = False
        self.obstacles_pos = [] #[[1,1],[1,2]]
        self.obstacles_r = [] #[1.2,1.2]
        self.flag_follow_obstacle = False
        self.switch_dist_0 = 1.2
        self.epsilon = 0.5


    def pose_cb(self, msg):
        # Obtain pose
        self.pos = np.array([msg.pose.pose.position.x + self.origin_odom[0], msg.pose.pose.position.y + self.origin_odom[1]])
        self.quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def set_path(self, path):
        # tratamento dos dados de caminho a*
        for ponto in path:
            ponto[0] = self.square_side*(6 - ponto[0])
            ponto[1] = self.square_side*(ponto[1] - 1)

        x_coords, y_coords = zip(*path)

        distancia_maxima = 0.1

        numero_de_pontos_interpolados = int(np.sum(np.sqrt(np.diff(x_coords) ** 2 + np.diff(y_coords) ** 2) / distancia_maxima)) + 1

        interp_x = interp1d(np.arange(len(x_coords)), x_coords, kind='linear')
        interp_y = interp1d(np.arange(len(y_coords)), y_coords, kind='linear')

        indices_interpolados = np.linspace(0, len(x_coords) - 1, numero_de_pontos_interpolados)
        x_interpolados = interp_x(indices_interpolados)
        y_interpolados = interp_y(indices_interpolados)

        self.C = [y_interpolados,x_interpolados]

    def set_origin(self):
        self.origin_odom = self.pos
    
    def field_from_points(self):
        d = [] # Distances

        for i in list(range(0,len(self.C[1]))):
            d.append(norm([self.pos[0] - self.C[0][i] , self.pos[1] - self.C[1][i]]))
        
        dmin = min(d)

        j = d.index(min(d)) # Indíce correspondente à distância mínima
        
        p_star = [self.C[0][j],self.C[1][j]] # Ponto mais próximo

        norma = norm(np.array([p_star[0] - self.pos[0],p_star[1] - self.pos[1]])) + 0.0001 # Norma da distância p p*
        
        N = [(p_star[0]-self.pos[0])/norma,(p_star[1]-self.pos[1])/norma] # Vetor normal

        if j == (len(self.C[1])-1):
            T = [self.C[0][0] - self.C[0][j] , self.C[1][0] - self.C[1][j]] # Caso especial
        else:
            T = [self.C[0][j+1] - self.C[0][j] , self.C[1][j+1] - self.C[1][j]] 

        T = np.array(T)/norm(T) # Tangent Vector

        G = (2/np.pi)*np.arctan(self.beta*dmin)
        H = np.sqrt(1-G**2)
        v = G*np.array(N)+H*np.array(T)

        # if not self.closed_path_flag:
        #     if j >= len(self.C[1]) - 2:
        #         v = [0.0,0.0]

        return [v[0],v[1]]

    def get_obstacle_avoidance(self, v):
        #Compute closest point - with respect to the world frame
        n_obst = len(self.obstacles_pos)
        D_close = float("inf")
        o_close = 0

        for o in range(n_obst):
            Dvec = [self.pos[0]-self.obstacles_pos[o][0], self.pos[1]-self.obstacles_pos[o][1]]
            D = np.sqrt(Dvec[0]**2 + Dvec[1]**2) - self.obstacles_r[o]

            if (D < D_close):
                o_close = o # D_close: distance to the closest obstacle
                D_close = D # o_close: index of the closest obstacle

        closest_world = self.obstacles_pos[o_close]

        if(self.flag_follow_obstacle, v):

            closest_vec = [closest_world[0]-self.pos[0], closest_world[1]-self.pos[1]]

            Do = np.sqrt(closest_vec[0]**2 + closest_vec[1]**2)

            closest_hat = [closest_vec[0]/(Do+1e-8), closest_vec[1]/(Do+1e-8)]
       
            # Doesn't take into account obstacle's radius
            if(Do < self.switch_dist_0 and (closest_vec[0]*v[0] + closest_vec[1]*v[1]) > 0): 
                D_vec2 = [-(closest_vec[0] - closest_hat[0]*self.epsilon), -(closest_vec[1] - closest_hat[1]*self.epsilon)]
                D2 = np.sqrt(D_vec2[0]**2 + D_vec2[1]**2 + D_vec2[2]**2)
                grad_D2 = [D_vec2[0]/D2, D_vec2[1]/D2]

                G2 = -(2 / np.pi) * np.arctan(self.beta * D2)  # convergence
                H2 = np.sqrt(1 - G2 ** 2)  # circulation

                alpha = 1
                V = [v[0]/self.v_r, v[1]/self.v_r]
   
                V_dot_gad_D2 = V[0]*grad_D2[0] + V[1]*grad_D2[1]

                T2 = [V[0] - alpha*V_dot_gad_D2*grad_D2[0], V[1] - alpha*V_dot_gad_D2*grad_D2[1]]
                norm_T2 = np.sqrt(T2[0]**2 + T2[1]**2)
                T2 = [T2[0]/norm_T2, T2[1]/norm_T2]

                Vx_o = self.v_r * (G2 * grad_D2[0] + H2 * T2[0])
                Vy_o = self.v_r * (G2 * grad_D2[1] + H2 * T2[1])

                if(Do < self.switch_dist):
                    Vx = Vx_o
                    Vy = Vy_o
                else:
                    theta = (Do - self.switch_dist)/(self.switch_dist_0 - self.switch_dist)
                    Vx = theta*v[0] + (1-theta)*Vx_o
                    Vy = theta*v[1] + (1-theta)*Vy_o
                    norma = np.sqrt(Vx**2 + Vy**2)
                    Vx = self.v_r*Vx/norma
                    Vy = self.v_r*Vy/norma

                v = [Vx, Vy]

        return v

    def setMovement(self, vector, yaw):
        # Movimento Imediato do robo

        u = [0.0,0.0,0.0,0.0]

        try:

            v = np.array([[vector[0]], [vector[1]], [yaw]])

            v = self.Kv* v / (np.linalg.norm(v) + 0.00001)

            M = [[1, -1, -(self.l + self.w)], [1, 1, (self.l + self.w)],
                [1, -1, (self.l + self.w)],[1, 1, -(self.l + self.w)]]
            
            u = Float32MultiArray()

            u.data = (1/self.r) * (M @ v)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Some exception has occured trying to obtain the frame reference")

        self.inputs_pub.publish(u)

    def walk_this(self, vec_b):
        # Move o robo para vetor posicao vec_b (referencial do robo)
        vec_b = [self.square_side*vec_b[0], self.square_side*vec_b[1]]
        rot = R.from_quat(self.quaternion)
        
        vec_w = rot.apply(np.array([vec_b[0],vec_b[1],0.0]))

        self.desired_point_w = [self.pos[0] + vec_w[0], self.pos[1] + vec_w[1]] 

        self.dist = np.sqrt((self.pos[0] - self.desired_point_w[0])**2 + (self.pos[1] - self.desired_point_w[1])**2)

        while (self.dist > self.tol_trans):
            self.setMovement(vec_b, 0)
            self.dist = np.sqrt((self.pos[0] - self.desired_point_w[0])**2 + (self.pos[1] - self.desired_point_w[1])**2)
                
        self.setMovement([0.0,0.0], 0)
    
    def set_quaternion(self, quaternion):
        # Verifique se o quaternion é válido
        if np.linalg.norm(quaternion) > 0:
            self.quaternion = quaternion
        else:
            # Se o quaternion não for válido, defina um valor padrão ou trate o erro conforme necessário
            print("Quaternions inválidos. Usando quaternion padrão.")
            self.quaternion = np.array([1, 0, 0, 0])  # Quaternion padrão
    
    def rotate_to_this(self, yaw):
        # Muda orientacao do robo para yaw (referencial do mundo)
        rot = R.from_quat(self.quaternion)

        self.set_quaternion()  # Atualize o quaternion

        curr_ang = rot.as_euler('zxy', degrees=True)[0]
        if curr_ang < 0:
            curr_ang += 360

        self.dist_ang = np.sqrt((curr_ang - yaw)**2)

        aux = 1

        if curr_ang > yaw:
            if self.dist_ang < 180:
                aux = -1
        else:
            if self.dist_ang > 180:
                aux = -1
        print(curr_ang)

        while (self.dist_ang > self.tol_angle):
            self.setMovement([0.0,0.0],self.Krot*aux)

            rot = R.from_quat(self.quaternion)
            curr_ang = rot.as_euler('zxy', degrees=True)[0]

            if curr_ang < 0:
                curr_ang += 360
            print(curr_ang)

            self.dist_ang = np.sqrt((curr_ang - yaw)**2)
        
        self.setMovement([0.0,0.0],0)

    def follow_field(self):
        self.desired_point_w = [self.C[0][-1],self.C[1][-1]]
        self.dist = np.sqrt((self.pos[0] - self.desired_point_w[0])**2 + (self.pos[1] - self.desired_point_w[1])**2)

        while (self.dist > self.tol_trans):
            # print(self.dist)
            vetor_w = self.field_from_points()

            rot_wb = R.from_quat(self.quaternion).inv()
            vetor_b = rot_wb.apply(np.array([vetor_w[0], vetor_w[1], 0.0]))

            self.setMovement(vetor_b, 0)
            self.dist = np.sqrt((self.pos[0] - self.desired_point_w[0])**2 + (self.pos[1] - self.desired_point_w[1])**2)

        self.setMovement([0.0,0.0], 0)
    