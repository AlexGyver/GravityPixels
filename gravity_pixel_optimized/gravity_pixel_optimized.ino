// --------------------- НАСТРОЙКИ ----------------------
// МАТРИЦА
#define PIN 6                // пин ленты Din
#define BRIGHTNESS 150        // яркость (0 - 255)
#define MATR_X 16            // число светодиодов по х
#define MATR_Y 16            // число светодиодов по у 
#define PIXELSIZE 10         // размер пикселя мм

// ФИЗИКА
#define G_CONST 9.81         // ускорение свободного падения
#define FRICTION 1           // трение
#define MIN_STEP 30          // минимальный шаг интегрирования (миллисекункды)
// при сильном уменьшении шага всё идёт по п*зде, что очень странно для Эйлера...

// ЭФФЕКТЫ
#define PIXEL_AMOUNT 30      // число "живых" пикселей
#define GLOW 0               // свечение
#define ALL_BLUE 0           // все синим

// оффсеты для акселерометра
int offsets[6] = { -3214, -222, 1324, -2, -67, -12};

/*
  ПОСМОТРИ, КАК ВЕДУТ СЕБЯ ШАРИКИ НА ДРУГИХ ПЛАНЕТАХ!!!
  Земля     9.81  м/с2
  Солнце    273.1 м/с2
  Луна      1.62  м/с2
  Меркурий  3.68  м/с2
  Венера    8.88  м/с2
  Марс      3.86  м/с2
  Юпитер    23.95 м/с2
  Сатурн    10.44 м/с2
  Уран      8.86  м/с2
  Нептун    11.09 м/с2
*/
// --------------------- НАСТРОЙКИ ----------------------

// цвета
#define BLUE     0x000088
#define RED      0x880000
#define GREEN    0x00ff00
#define CYAN     0x008888
#define MAGENTA  0xaa0088
#define YELLOW   0x888800
#define WHITE    0x505050

#define GLOW_FADE1 0x000004  // свечение ближнее
#define GLOW_FADE2 0x000002  // свечение дальнее

uint32_t colors[] = {
  BLUE,
  RED,
  GREEN,
  CYAN,
  MAGENTA,
  YELLOW,
  WHITE,
};

// ---------- БИБЛИОТЕКИ -----------
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MATR_X * MATR_Y, PIN, NEO_GRB + NEO_KHZ800);
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 mpu;
// ---------- БИБЛИОТЕКИ -----------

// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------
#define MATR_X_M MATR_X*PIXELSIZE         // размер матрицы в миллиметрах х
#define MATR_Y_M MATR_Y*PIXELSIZE         // размер матрицы в миллиметрах у

int x_vel[PIXEL_AMOUNT];     // МИЛЛИМЕТРЫ В СЕКУНДУ
int y_vel[PIXEL_AMOUNT];     // МИЛЛИМЕТРЫ В СЕКУНДУ
int x_dist[PIXEL_AMOUNT];    // МИЛЛИМЕТРЫ
int y_dist[PIXEL_AMOUNT];    // МИЛЛИМЕТРЫ
byte friction[PIXEL_AMOUNT];
byte bounce[PIXEL_AMOUNT];
byte color[PIXEL_AMOUNT];

float mpuPitch;
float mpuRoll;
int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long integrTimer, loopTimer;
float stepTime;
// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------

void setup() {
  Serial.begin(9600);
  mpuSetup();
  strip.begin();
  strip.setBrightness(BRIGHTNESS);

  // начальные условия. Центр матрицы, Скорость нулевая
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    x_vel[i] = 0;
    y_vel[i] = 0;
    x_dist[i] = (MATR_X_M / 2);
    y_dist[i] = (MATR_Y_M / 2);
  }

  randomSeed(analogRead(0));   // зерно для генератора псевдослучайных чисел

  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    // получаем случайные величины
    friction[i] = random(0, 30);   // ТРЕНИЕ. В дальнейшем делится на 100
    bounce[i] = random(60, 95);     // ОТСКОК. В дальнейшем делится на 100

    // здесь хитро, чтобы вся палитра цветов досталась всем пикселям
    color[i] = map(i, 0, PIXEL_AMOUNT, 0, 7);
    if (ALL_BLUE) color[i] = 0;
  }
}

void loop() {
  if (millis() - loopTimer > MIN_STEP) {
    loopTimer = millis();
    integrate();
    strip.clear();
    for (byte i = 0; i < PIXEL_AMOUNT; i++) {
      byte nowDistX = floor(x_dist[i] / PIXELSIZE);   // перевести миллиметры в пиксели
      byte nowDistY = floor(y_dist[i] / PIXELSIZE);   // перевести миллиметры в пиксели

      if (GLOW) {
        glowDraw(nowDistX - 1, nowDistY, GLOW_FADE1);        // нарисовать точку
        glowDraw(nowDistX + 1, nowDistY, GLOW_FADE1);        // нарисовать точку
        glowDraw(nowDistX, nowDistY - 1, GLOW_FADE1);        // нарисовать точку
        glowDraw(nowDistX, nowDistY + 1, GLOW_FADE1);        // нарисовать точку

        glowDraw(nowDistX - 1, nowDistY + 1, GLOW_FADE2);       // нарисовать точку
        glowDraw(nowDistX - 1, nowDistY - 1, GLOW_FADE2);      // нарисовать точку
        glowDraw(nowDistX + 1, nowDistY - 1, GLOW_FADE2);      // нарисовать точку
        glowDraw(nowDistX + 1, nowDistY + 1, GLOW_FADE2);      // нарисовать точку

        glowDraw(nowDistX - 2, nowDistY, GLOW_FADE2);        // нарисовать точку
        glowDraw(nowDistX + 2, nowDistY, GLOW_FADE2);        // нарисовать точку
        glowDraw(nowDistX, nowDistY - 2, GLOW_FADE2);        // нарисовать точку
        glowDraw(nowDistX, nowDistY + 2, GLOW_FADE2);        // нарисовать точку
      }

      pixelDraw(nowDistX, nowDistY, color[i]);            // нарисовать точку

    }
    strip.show();
  }
}

void pixelDraw(byte x, byte y, byte colorNum) {
  if (y % 2 == 0)                                                         // если чётная строка
    strip.setPixelColor(y * MATR_X + x, colors[colorNum]);                // заливаем в прямом порядке
  else                                                                    // если нечётная
    strip.setPixelColor(y * MATR_X + MATR_X - x - 1, colors[colorNum]);   // заливаем в обратном порядке
}
void glowDraw(byte x, byte y, byte color) {
  if (y % 2 == 0)                                                         // если чётная строка
    strip.setPixelColor(y * MATR_X + x, color);                // заливаем в прямом порядке
  else                                                                    // если нечётная
    strip.setPixelColor(y * MATR_X + MATR_X - x - 1, color);   // заливаем в обратном порядке
}

void integrate() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                     // получить ускорения
  mpuPitch = (float)ax / 16384;                                     // 16384 это величина g с акселерометра
  mpuRoll = (float)ay / 16384;

  stepTime = (float)((long)millis() - integrTimer) / 1000;            // расчёт времени шага интегрирования
  integrTimer = millis();
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {                           // для каждого пикселя
    int thisAccel_x, thisAccel_y;                                     // текущее ускорение
    int grav, frict;

    ///////////////////// ОСЬ Х /////////////////////
    grav = (float)G_CONST * mpuPitch * 1000;    // сила тяжести
    if (FRICTION) {
      frict = (float)G_CONST * (1 - mpuPitch) * friction[i] * 10; // сила трения
      if (x_vel[i] > 0) frict = -frict;   // знак силы трения зависит от направления вектора скорости
      if (x_vel[i] == 0 && abs(grav) < frict) thisAccel_x = 0;  // трение покоя
      else thisAccel_x = (grav + frict);                        // ускорение
    } else thisAccel_x = grav;

    /////////////////////// ОСЬ У /////////////////////
    grav = (float)G_CONST * mpuRoll * 1000;
    if (FRICTION) {
      frict = (float)G_CONST * (1 - mpuRoll) * friction[i] * 10;
      if (y_vel[i] > 0) frict = -frict;
      if (y_vel[i] == 0 && abs(grav) < frict) thisAccel_y = 0;
      else thisAccel_y = (grav + frict);
    } else thisAccel_y = grav;

    ///////////////////// ИНТЕГРИРУЕМ ///////////////////
    // скорость на данном шаге V = V0 + ax*dt
    x_vel[i] += (float)thisAccel_x * stepTime;
    y_vel[i] += (float)thisAccel_y * stepTime;

    // координата на данном шаге X = X0 + Vx*dt
    x_dist[i] += (float)x_vel[i] * stepTime;
    y_dist[i] += (float)y_vel[i] * stepTime;

    /////////////////// ПОВЕДЕНИЕ У СТЕНОК /////////////////
    // рассматриваем 4 стенки матрицы
    if (x_dist[i] < 0) {     // если пробили край матрицы
      x_dist[i] = 0;         // возвращаем на край
      x_vel[i] = -x_vel[i] * (float)bounce[i] / 100;    // скорость принимаем с обратным знаком и * на коэффициент отскока
    }
    if (x_dist[i] > MATR_X_M - PIXELSIZE) {
      x_dist[i] = MATR_X_M - PIXELSIZE;
      x_vel[i] = -x_vel[i] * (float)bounce[i] / 100;
    }

    if (y_dist[i] < 0) {
      y_dist[i] = 0;
      y_vel[i] = -y_vel[i] * (float)bounce[i] / 100;
    }
    if (y_dist[i] > MATR_Y_M - PIXELSIZE) {
      y_dist[i] = MATR_Y_M - PIXELSIZE;
      y_vel[i] = -y_vel[i] * (float)bounce[i] / 100;
    }
  }
}

void mpuSetup() {
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // ставим оффсеты
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
}
