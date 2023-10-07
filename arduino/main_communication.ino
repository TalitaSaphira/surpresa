//Primeiro deve-se enviar os dados para a rasp:
//   - imu mpu 6050
//   - encoders AS5600
//Depois vamos receber informações para o controle das rodas:
//   - drivers A4988

//#include <Wire.h>
//#include <MPU6050.h>

//const int MPU6050_addr = 0x68; // Endereço I2C do MPU6050

//===drivers===
const int stepLB = 13;
const int stepRB = 12;
const int stepLF = 11;
const int stepRF = 10;

const int dirLB = 33;
const int dirRB = 32;
const int dirLF = 31;
const int dirRF = 30;

#define ENABLE_PIN 4

//===imu===
//MPU6050 imu;

void setup() {
  //===Início da comunicação serial===
  //Wire.begin();
  Serial.begin(115200);
  while (!Serial) {}
//  initializeMPU6050();
}

void loop() {
//  float quaternion[4];
//  float covariance[9];
//
//  readMPU6050Data(quaternion, covariance);
//
//  // Enviar dados para a Raspberry Pi
//  Serial.write((uint8_t *)quaternion, sizeof(quaternion));
//  Serial.write((uint8_t *)covariance, sizeof(covariance));
  
  //Rotate motors forward
  digitalWrite(dirLB, HIGH); // Set direction
  digitalWrite(dirRB, HIGH);
  digitalWrite(dirLF, HIGH);
  digitalWrite(dirRF, HIGH);

  if(Serial.available() > 0){
    String dados = Serial.readStringUntil('\n');

    // Separe os valores usando a vírgula como delimitador
    float velLB = dados.substring(0, dados.indexOf(',')).toFloat();
    dados.remove(0, dados.indexOf(',') + 1);
    float velRB = dados.substring(0, dados.indexOf(',')).toFloat();
    dados.remove(0, dados.indexOf(',') + 1);
    float velLF = dados.substring(0, dados.indexOf(',')).toFloat();
    dados.remove(0, dados.indexOf(',') + 1);
    float velRF = dados.toFloat();

    // Use os valores float como desejado
    Serial.print("VelLB: "); Serial.println(velLB, 2);
    Serial.print("VelRB: "); Serial.println(velRB, 2);
    Serial.print("VelLF: "); Serial.println(velLF, 2);
    Serial.print("VelLF: "); Serial.println(velRF, 2);

    // Generate PWM signal for motor steps (adjust the delay for speed control)
    for (int i = 0; i < 1000; i++) {
      digitalWrite(stepLB, velLB);
      digitalWrite(stepRB, velLF);
      digitalWrite(stepLF, velRB);
      digitalWrite(stepRF, velRF);
      delayMicroseconds(500); // Adjust the delay for speed control
      digitalWrite(stepLB, LOW);
      digitalWrite(stepRB, LOW);
      digitalWrite(stepLF, LOW);
      digitalWrite(stepRF, LOW);
      delayMicroseconds(500); // Adjust the delay for speed control
    }  
        //Use isso apenas ver se o código funciona e depois exclua
        //Serial.println(message);
  }
  
  delay(1000);  // Aguarda 1 segundo antes de ler os dados novamente
}

//void initializeMPU6050() {
//  Wire.beginTransmission(MPU6050_addr);
//  Wire.write(0x6B); // PWR_MGMT_1 register
//  Wire.write(0);    // Ativa o MPU-6050
//  Wire.endTransmission(true);
//}
//
//void readMPU6050Data(float *quaternion, float *covariance) {
//  
//  
//  // Skip temperature data
//  Wire.read(); Wire.read();
//
//  gyroscope[0] = Wire.read() << 8 | Wire.read();
//  gyroscope[1] = Wire.read() << 8 | Wire.read();
//  gyroscope[2] = Wire.read() << 8 | Wire.read();
//  // Substitua o código abaixo com sua implementação real
//  for (int i = 0; i < 4; ++i) {
//    quaternion[i] = 0.0;
//  }
//
//  for (int i = 0; i < 9; ++i) {
//    covariance[i] = 0.0;
//  }
//}
