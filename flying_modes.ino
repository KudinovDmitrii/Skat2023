#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SD.h>
File myFile;
MPU6050 mpu;
Servo serv1;
Servo serv2;
Servo mot1;
Servo mot2;

//521 part
uint8_t fifoBuffer[45];         // буфер
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define BUFFER_SIZE 100
bool Gotted = 0;
float tang1;
float kren1;
float ypr[3];          //z, y, x

//control part
unsigned long t = 0;             //time
bool target_flag = 0;
int pin_switch = 2
int pin_mots = 3;
int pin_tang = 4;
int pin_kren = 5;
int mot1_pin = 6;
int mot2_pin = 7;
int serv1_pin = 8;
int serv2_pin = 9;
int pin_mode = 11;



//SD card
unsigned long global_time = 0;
int sd_pin = 10;

void setup() {
  Serial.begin(9600);
  //control
  serv1.attach(serv1_pin);
  serv2.attach(serv2_pin);
  mot1.attach(mot1_pin);
  mot2.attach(mot2_pin);
  pinMode(pin_mots, INPUT);
  pinMode(pin_tang, INPUT);
  pinMode(pin_kren, INPUT);
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("initialization SD failed!");
}
  else Serial.println("initialization SD done.");
  //GY521 
  Wire.begin();
  //Wire.setClock(1000000UL);   // разгоняем шину на максимум
  mpu.initialize();
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  /* Serial.println(F("Send any character to start sketch"));
  delay(100);
  while (1) {                     //входим в бесконечный цикл
    if (Serial.available() > 0) { //если нажата любая кнопка
      Serial.read();              //прочитать (чтобы не висел в буфере)
      break;                      //выйти из цикла
    }
  }
  delay(1000);
  */
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  /*
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.println(gz);
  */
  calibration();
  // выводим значения после калибровки
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  delay(20);
  // инициализация DMP
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mot1.writeMicroseconds(800);
  mot2.writeMicroseconds(800);
}

void loop() {

//SD Write
  if(millis() - global_time >= 200){
      global_time = millis();
      myFile = SD.open("data.txt", FILE_WRITE);
      int speed2 = map(pulseIn(pin_mots, HIGH),990,2000,770,2300);
      int tang2 = map(pulseIn(pin_tang, HIGH),950,1950,40,140);
      int kren2 = map(pulseIn(pin_kren, HIGH),990,2000,40,140);
      myFile.print("Time: ");
      myFile.print(global_time);
      myFile.print(", angle z, tang, kren: ");
      myFile.print(ypr[0]);
      myFile.print(", ");
      myFile.print(ypr[1]);
      myFile.print(", ");
      myFile.print(ypr[2]); 
      myFile.print(", INPUT speed, tang, kren: ");
      myFile.print(speed2);
      myFile.print(", ");
      myFile.print(tang2);
      myFile.print(", ");
      myFile.println(kren2);
      myFile.close();
  }

/*
  if(millis() - t >= 11){
   int sensorRead = analogRead(pin);
   // (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
   analogWrite(pin, computePID(sensorRead, 30, 5.0, 2.0, 3.0, 0.011, 0, 255));
  }
*/

//Piloting 
if((pulseIn(pin_switch, HIGH) > 1350) && (pulseIn(pin_switch, HIGH) < 1650)){
  target_flag = 0;
  int speed2 = map(pulseIn(pin_mots, HIGH),990,2000,770,2300);
  int tang2 = map(pulseIn(pin_tang, HIGH),950,1950,40,140);
  int kren2 = map(pulseIn(pin_kren, HIGH),990,2000,40,140);
  int sum1 = constrain(tang1 + (90-kren1), 30, 150);
  int sum2 = constrain(tang1 - (90-kren1), 30, 150);
  mot1.writeMicroseconds(speed1);
  mot2.writeMicroseconds(speed1);
  serv1.write(sum1);
  serv2.write(sum2);
}

//Take off mod
if((pulseIn(pin_switch, HIGH) > 850) && (pulseIn(pin_switch, HIGH) < 1150)){
  target_flag = 0;
  mot1.writeMicroseconds(2300);
  mot2.writeMicroseconds(2300);
  if(millis() - t >= 11){
     angle();
     if(Gotted == 1){
      Gotted = 0;
      float tang = ypr[1];
      float kren = ypr[2];
      int res1 = computePID(tang, 45, 5.0, 2.0, 3.0, 0.011, -50, 50);
      int res2 = computePID(kren, 0, 5.0, 2.0, 3.0, 0.011, -50, 50);
      int sum1 = constrain(res1 + res2, -60, 60);
      int sum2 = constrain(res1 - res2, -60, 60);
      serv1.write(90+sum1);
      serv2.write(90+sum2);
     }
    }
}
//

//Stabilization
if((pulseIn(pin_switch, HIGH) > 1850) && (pulseIn(pin_switch, HIGH) < 2150)){
  target_flag = 0;
  mot1.writeMicroseconds(2300);
  mot2.writeMicroseconds(2300);
  if(millis() - t >= 11){
       angle();
       if(Gotted == 1){
        Gotted = 0;
        float tang = ypr[1];
        float kren = ypr[2];
        int res1 = computePID(tang, 0, 5.0, 2.0, 3.0, 0.011, -50, 50);
        int res2 = computePID(kren, 0, 5.0, 2.0, 3.0, 0.011, -50, 50);
        int sum1 = constrain(res1 + res2, -60, 60);
        int sum2 = constrain(res1 - res2, -60, 60);
        serv1.write(90+sum1);
        serv2.write(90+sum2);
      }
  }
}
//

//Hold the target
if((pulseIn(pin_switch, HIGH) == -1){
  mot1.writeMicroseconds(2300);
  mot2.writeMicroseconds(2300);
  if(millis() - t >= 11){
    angle();
    if(Gotted == 1){
      Gotted = 0;
      if(target_flag = 0){
        target_flag = 1;
        tang1 = ypr[1];
        kren1 = ypr[2];
      }
      float tang = ypr[1];
      float kren = ypr[2];
      int res1 = computePID(tang, tang1, 5.0, 2.0, 3.0, 0.011, -50, 50);
      int res2 = computePID(kren, kren1, 5.0, 2.0, 3.0, 0.011, -50, 50);
      int sum1 = constrain(res1 + res2, -60, 60);
      int sum2 = constrain(res1 - res2, -60, 60);
      serv1.write(90+sum1);
      serv2.write(90+sum2);
    }
  }
}
//
}

int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void calibration() {
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];
  // используем стандартную точность
//  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
 // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  // обнуляем оффсеты
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);
 // Serial.println("Calibration start. It will take about 5 seconds");
  for (byte n = 0; n < 10; n++) {     // 10 итераций калибровки
    for (byte j = 0; j < 6; j++) {    // обнуляем калибровочный массив
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) { // делаем BUFFER_SIZE измерений для усреднения
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
      if (i >= 99) {                         // пропускаем первые 99 измерений
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];   // записываем в калибровочный массив
        }
      }
    }
    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // учитываем предыдущую калибровку
      if (i == 2) offsets[i] += 16384;                               // если ось Z, калибруем в 16384
      offsetsOld[i] = offsets[i];
    }
    // ставим новые оффсеты
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    delay(2);
  }
}

void angle() {
     if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;
      float ypr_temp[3];

      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr_temp, &q, &gravity);
      ypr[0] = degrees(ypr_temp[0]);
      ypr[1] = degrees(ypr_temp[1]);
      ypr[2] = degrees(ypr_temp[2]);
      // выводим результат в градусах (-180, 180)
      /*
      Serial.print(ypr[0]); // вокруг оси Z
      Serial.print(',');
      Serial.print(ypr[1]); // вокруг оси Y
      Serial.print(',');
      Serial.print(ypr[2]); // вокруг оси X
      Serial.println();
      // для градусов можно использовать degrees()
      */
      t = millis();
      Gotted = 1;
    }
  }
