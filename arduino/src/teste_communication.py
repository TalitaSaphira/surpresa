#! /usr/bin/env python3
# publisher.py
# Importe as bibliotecas ROS necessárias
import rospy
from std_msgs.msg import Float32MultiArray

# Inicialize o nó
rospy.init_node("teste_comunicação")

# Crie um objeto de publicação para publicar mensagens no tópico 'meu_topico'
teste_pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)

# Crie uma mensagem do tipo MeuMsg
teste_velocidades = Float32MultiArray()
teste_velocidades.data = [1.0, 2.0, 3.0, 4.0]

# Publique a mensagem a cada 1 segundo
rate = rospy.Rate(1)  # 1 Hz
while not rospy.is_shutdown():
    teste_pub.publish(teste_velocidades)
    print("velocidade publicada\n")
    rate.sleep()
