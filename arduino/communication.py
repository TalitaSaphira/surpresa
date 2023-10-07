#! /usr/bin/env python3
#Primeiro deve-se receber os dados do arduino:
#   - imu mpu 6050
#   - encoders AS5600
#Depois vamos enviar informações pro arduino controlar as rodas:
#   - drivers A4988

#Implementar publishers e subscriber para a manipulação dos dados

import serial
import time
#plkhtkrokg erlçlçkaspthkr#lrçeg,kçerjgkoertkhkprt
import rospy
from std_msgs.msg import Float32MultiArray


#Configuração da porta serial (substitua '/dev/ttyS0' pelo seu dispositivo serial)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(3)
ser.reset_input_buffer()

while ser.isOpen() == False:
    print('Waiting')

print("Serial OK")

def callback(data):
    global velRB, velLB, velRF, velLF
    velLB = data.data[0]
    velRB = data.data[1]
    velLF = data.data[2]
    velRF = data.data[3]
    enviar_dados_serial()

def enviar_dados_serial():
    # Configuração da porta serial (substitua '/dev/ttyS0' pelo seu dispositivo serial)
    with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
        time.sleep(3)
        ser.reset_input_buffer()

        try:
            while True:
                # Formate os valores em uma string com delimitadores (por exemplo, vírgula)
                dados = f"{velLB},{velLF},{velRB},{velRF}\n"
                print(dados)

                # Envie a string pela porta serial
                ser.write(dados.encode())

        except KeyboardInterrupt:
            print ('Close Serial communication')
            ser.close()

rospy.init_node('Communication', anonymous=True)
rospy.Subscriber('/wheel_velocities', Float32MultiArray, callback)

rospy.spin()

