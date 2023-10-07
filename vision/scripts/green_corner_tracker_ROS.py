#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Adicionando importação para mensagem de string
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv

# Nome do arquivo de vídeo ou dispositivo de captura (0 para câmera padrão)
filename = 2
cap = cv.VideoCapture(filename)

# verde
lower_range1 = np.array([80, 165, 70])
upper_range1 = np.array([97, 255, 172])

# Inicializar o ROS node
rospy.init_node('green_finder_node')
# Criar um publisher para a imagem processada
image_pub = rospy.Publisher('processed_image_green', Image, queue_size=1)
# Criar um publisher para o conteúdo do terminal
terminal_pub = rospy.Publisher('greencorner', bool, queue_size=1)
# Criar um objeto CvBridge para converter entre o formato OpenCV e o formato ROS
bridge = CvBridge()

def green(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_range1, upper_range1)
    _, mask1 = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)
    cnts, _ = cv.findContours(mask1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    for c in cnts:
        # Defina o tamanho mínimo do contorno em pixels
        tamanho_minimo = 4000

        if cv.contourArea(c) > tamanho_minimo:
            x, y, w, h = cv.boundingRect(c)
            cv.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv.putText(img, "GOTCHA!", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            #print("Achou o plano verde!")
            terminal_content = True
            terminal_pub.publish(terminal_content)
            

    # Converter a imagem OpenCV para uma mensagem ROS
    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    # Publicar a imagem processada
    image_pub.publish(img_msg)

while cap.isOpened():
    # Capturar um quadro do vídeo
    ret, frame = cap.read()

    if not ret:
        break

    green(frame)

    # Remover a exibição da imagem (substituída pela publicação ROS)
    cv.imshow("green finder", frame)

    # Verifique se a tecla 'Esc' foi pressionada para sair do loop
    if cv.waitKey(1) & 0xFF == 27:
        break

# Liberar a captura de vídeo e fechar todas as janelas
cap.release()
cv.destroyAllWindows()

