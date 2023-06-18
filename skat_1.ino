#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
Servo serv1;
Servo serv2;
Servo cargo1;
Servo cargo2;
Servo mots;
Servo impel;

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
unsigned long ts = 0;            //micros
unsigned long val = 1;

int S1 = 0, S2 = 0, Sp = 0;

int pin_switch = 2; // open cargo, pilot, stab

int pin_mots = 3;
int pin_impel = 4;
int pin_tang = 5;
int pin_kren = 6;

int mots_pin = 9;
int impel_pin = 10;
int serv1_pin = 11;
int serv2_pin = 12;

int cargo1_pin = 7;
int cargo2_pin = 8;



void setup() {
  Serial.begin(9600);
  //control
  serv1.attach(serv1_pin);
  serv2.attach(serv2_pin);
  cargo1.attach(cargo1_pin);
  cargo2.attach(cargo2_pin);
  mots.attach(mots_pin);
  impel.attach(impel_pin);
  pinMode(pin_mots, INPUT);
  pinMode(pin_impel, INPUT);
  pinMode(pin_tang, INPUT);
  pinMode(pin_kren, INPUT);
  
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

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  calibration();
  // выводим значения после калибровки
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  delay(20);
  // инициализация DMP
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mots.writeMicroseconds(1100);
  impel.writeMicroseconds(1100);
  cargo1.write(10);
  cargo2.write(10);
}

void loop() {
Serial.println(pulseIn(pin_switch, HIGH));
//cargo drop
if(pulseIn(pin_switch, HIGH) > 800 && pulseIn(pin_switch, HIGH) < 1200) {
  cargo1.write(100);
  cargo2.write(100);
}

//


//Piloting 
if((pulseIn(pin_switch, HIGH) > 1350) && (pulseIn(pin_switch, HIGH) < 1650)){
  cargo1.write(10);
  cargo2.write(10);
  if(millis() - t >= 22){
  int speed1 = map(pulseIn(pin_mots, HIGH),990,2000,1000,1940);
  int tang2 = map(pulseIn(pin_tang, HIGH),1100,1750,60,120);
  int kren2 = map(pulseIn(pin_kren, HIGH),1100,1750,60,120);
  int sum1 = constrain(tang2 + (90 - kren2), 60, 120);
  int sum2 = constrain(tang2 - (90 - kren2), 60, 120);

  if(pulseIn(pin_impel, HIGH) > 1200 && pulseIn(pin_impel, HIGH) < 2100){
  mots.writeMicroseconds(speed1);
  }
  else impel.writeMicroseconds(speed1);
  //impel.writeMicroseconds(speed1);
  serv1.write(sum1);
  serv2.write(sum2);

  Serial.println("speed, serv1, serv2, switch");
  Serial.println(speed1);
  Serial.println(sum1);
  Serial.println(sum2);
  Serial.println(pulseIn(pin_switch, HIGH));
  t = millis();
  }
}


//Stabilization
if((pulseIn(pin_switch, HIGH) > 1850) && (pulseIn(pin_switch, HIGH) < 2150)){
  cargo1.write(10);
  cargo2.write(10);
  mots.writeMicroseconds(700);
  impel.writeMicroseconds(900);
  if(millis() - t >= 12){
       angle();
       if(Gotted == 1){
        Gotted = 0;
        float tang = ypr[1];
        float kren = ypr[2];
        int res1 = computePID(tang, 0, 4.0, 2.0, 0.0, 0.012, -30, 30);
        int res2 = computePID(kren, 0, 4.0, 2.0, 0.0, 0.012, -30, 30);
        int sum1 = constrain(res1 + res2, -30, 30);
        int sum2 = constrain(res1 - res2, -30, 30);
        serv1.write(90+sum1);
        serv2.write(90+sum2);
        
        Serial.println("speed, serv1, serv2, switch");
        Serial.println(1000);
        Serial.println(90 + sum1);
        Serial.println(90 + sum2);
        Serial.println(pulseIn(pin_switch, HIGH));
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

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);

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
      t = millis();
      Gotted = 1;
    }
  }
