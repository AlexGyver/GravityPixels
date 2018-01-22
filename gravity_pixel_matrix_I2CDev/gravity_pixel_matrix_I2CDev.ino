// ---------- НАСТРОЙКИ -----------
int offsets[6] = { -2433, -157, 819, 12, 24, 37};   // оффсеты для акселерометра
#define BRIGHTNESS 20
#define MATR_X 16
#define MATR_Y 16
#define MATR_X_M 0.16
#define MATR_Y_M 0.16
#define PIN 6
#define PIXEL_AMOUNT 3
#define G_CONST 9.81

#define DIST_TO_LED_X (int)MATR_X / MATR_X_M
#define DIST_TO_LED_Y (int)MATR_Y / MATR_Y_M
// ---------- НАСТРОЙКИ -----------

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

uint16_t colors[] = {
  BLUE,
  RED,
  GREEN,
  CYAN,
  MAGENTA,
  YELLOW,
  WHITE,
};

// ---------- БИБЛИОТЕКИ -----------
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
// ---------- БИБЛИОТЕКИ -----------

// ---------- СОЗДАЁМ МАТРИЦУ -----------
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(MATR_X, MATR_Y, PIN,
                            NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
                            NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
                            NEO_GRB            + NEO_KHZ800);
// ---------- СОЗДАЁМ МАТРИЦУ -----------

// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------
float x_vel[PIXEL_AMOUNT];
float y_vel[PIXEL_AMOUNT];
float x_dist[PIXEL_AMOUNT];
float y_dist[PIXEL_AMOUNT];
float friction[PIXEL_AMOUNT];
float bounce[PIXEL_AMOUNT];
byte color[PIXEL_AMOUNT];

#define PITCH 1
#define ROLL 2
#define YAW 0

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;
uint8_t mpuIntStatus, devStatus, fifoBuffer[64];
uint16_t packetSize, fifoCount;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned long integrTimer;
float stepTime, loopTime;
// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------

void setup() {
  Serial.begin(9600);
  mpuSetup();

  matrix.begin();
  matrix.setBrightness(BRIGHTNESS);
  matrix.clear();

  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    x_vel[i] = 0;
    y_vel[i] = 0;
    x_dist[i] = MATR_X_M / 2;
    y_dist[i] = MATR_Y_M / 2;
  }
  randomSeed(analogRead(0));
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    friction[i] = (float)random(1, 10) / 10;  // получить в диапазоне 0.1 - 0.9
    bounce[i] = (float)random(1, 10) / 10;
    color[i] = random(0, 7);
  }

}

void loop() {
  loopTime = millis();
  integrate();
  matrix.clear();
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    byte nowDistX = floor(x_dist[i] * DIST_TO_LED_X);
    byte nowDistY = floor(y_dist[i] * DIST_TO_LED_Y);
    matrix.drawPixel(nowDistX, nowDistY, colors[color[i]]);
  }
  matrix.show();

  Serial.println(millis() - loopTime);
}

void integrate() {
  // получить углы с mpu
  get_angles();

  // расчёт времени шага интегрирования
  stepTime = (float)((long)micros() - integrTimer) / 1000000;
  integrTimer = micros();

  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    float thisAccel_x = (float)G_CONST * (sin(mpuPitch)) * (1 - friction[i]);
    float thisAccel_y = (float)G_CONST * (sin(mpuRoll)) * (1 - friction[i]);

    x_vel[i] = x_vel[i] + thisAccel_x * stepTime;
    y_vel[i] = y_vel[i] + thisAccel_y * stepTime;

    x_dist[i] = x_dist[i] + x_vel[i] * stepTime + (float)thisAccel_x * stepTime * stepTime / 2;
    y_dist[i] = y_dist[i] + y_vel[i] * stepTime + (float)thisAccel_y * stepTime * stepTime / 2;

    if (x_dist[i] <= 0) {
      x_dist[i] = 0;
      x_vel[i] = -x_vel[i] * bounce[i];
    }
    if (!(x_dist[i] < MATR_X_M)) {
      x_dist[i] = MATR_X_M - 0.001;
      x_vel[i] = -x_vel[i] * bounce[i];
    }
    if (y_dist[i] <= 0) {
      y_dist[i] = 0;
      y_vel[i] = -y_vel[i] * bounce[i];
    }
    if (!(y_dist[i] < MATR_Y_M)) {
      y_dist[i] = MATR_Y_M - 0.001;
      y_vel[i] = -y_vel[i] * bounce[i];
    }
  }  
}

void get_angles() {
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize)
    return;
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  mpu.resetFIFO();
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // получаем углы В РАДИАНАХ!
  mpuPitch = ypr[PITCH];
  mpuRoll = ypr[ROLL];
  mpuYaw  = ypr[YAW];
  mpu.resetFIFO();
}

void mpuSetup() {
  Wire.begin();
  TWBR = 24;
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();

  // ставим оффсеты
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
  /*
    // Acceleration range: ± 2   ± 4  ± 8  ± 16 g
    // 1G value:           16384 8192 4096 2048
    // MAX G value:        32768
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    // Gyroscope range: 250   500  1000 2000 °/s
    // MAX value:       32768
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  */
}
