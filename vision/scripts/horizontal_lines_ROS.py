#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import time

# Nome do arquivo de vídeo ou dispositivo de captura (0 para câmera padrão)
filename = 0
cap = cv.VideoCapture(filename)

start_time = time.time()
lines_data = []
averages = []

# Inicializar o ROS node
rospy.init_node('line_counter_node')
# Criar um publisher para a imagem processada
image_pub = rospy.Publisher('processed_image', Image, queue_size=1)
# Criar um publisher para o conteúdo do terminal
terminal_pub = rospy.Publisher('horizontal_lines', String, queue_size=1)
# Criar um objeto CvBridge para converter entre o formato OpenCV e o formato ROS
bridge = CvBridge()

while cap.isOpened():
    # Capturar um quadro do vídeo
    ret, frame = cap.read()

    if not ret:
        break

    # Converter o quadro em escala de cinza
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    edges = cv.Canny(gray, 50, 200, None, 3)

    lines = cv.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 10)

    cannycolored = np.zeros_like(frame)  # Crie uma imagem em branco

    contagem_linhas = 0

    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            a1, b1, a2, b2 = l

            angle1 = np.arctan2(b2 - b1, a2 - a1)
            angle_deg1 = np.degrees(angle1)

            line_length = np.sqrt((a2 - a1)**2 + (b2 - b1)**2)

            if (315 <= angle_deg1 <= 360 or 0 <= angle_deg1 <= 45) or (-45 <= angle_deg1 <= 0):
                color = (0, 0, 255)
                if line_length >= 50:
                    contagem_linhas += 1
            else:
                color = (0, 255, 0)

            cv.line(cannycolored, (a1, b1), (a2, b2), color, 3, cv.LINE_AA)

    gray = np.float32(gray)

    dst = cv.cornerHarris(gray, 2, 3, 0.04)

    dst = cv.dilate(dst, None)

    frame[dst > 0.01 * dst.max()] = [0, 0, 255]

    #cv.imshow("Leitura real", frame)
    cv.imshow("Original Frame", cannycolored)

    # Verifique se passou 0.5 segundos
    elapsed_time = time.time() - start_time
    if elapsed_time >= 0.5:
        # Calcule a média das últimas 4 linhas horizontais
        if len(lines_data) >= 4:
            average_lines = np.mean(lines_data[-4:])
            averages.append(average_lines)
            #print(f"Média das últimas 4 linhas horizontais: {average_lines}")

            # Imprima o array a cada 4 publicações de médias
            if len(averages) % 4 == 0:
                last_four_averages = averages[-4:]
                last_four_averages.sort()  # Ordene a lista
                trimmed_averages = last_four_averages[1:-1]  # Exclua o maior e o menor
                final_average = np.mean(trimmed_averages)
                #print(f"Array das últimas 4 médias: {last_four_averages}")
                terminal_content = f"FINAL: {final_average}"
                #print(terminal_content)
                terminal_pub.publish(terminal_content)

        # Reinicie o contador e a lista de dados
        start_time = time.time()

    # Adicione as informações relevantes à lista
    lines_data.append(contagem_linhas)

    # Converter a imagem OpenCV para uma mensagem ROS
    img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    # Publicar a imagem processada
    image_pub.publish(img_msg)

    # Verifique se a tecla 'Esc' foi pressionada para sair do loop
    if cv.waitKey(1) & 0xFF == 27:
        break

# Liberar a captura de vídeo e fechar todas as janelas
cap.release()
cv.destroyAllWindows()