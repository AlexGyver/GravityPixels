// ---------- НАСТРОЙКИ -----------
int offsets[6] = { -3291, -401, 1301, -12, -61, 3};   // оффсеты для акселерометра
#define BRIGHTNESS 20
#define MATR_X 16
#define MATR_Y 16
#define MATR_X_M 0.16
#define MATR_Y_M 0.16
#define PIN 6
#define PIXEL_AMOUNT 33
#define G_CONST 9.81

#define DIST_TO_LED_X (int)MATR_X / MATR_X_M
#define DIST_TO_LED_Y (int)MATR_Y / MATR_Y_M
// ---------- НАСТРОЙКИ -----------

// Color definitions
#define BLUE     0x0000ff
#define RED      0xff0000
#define GREEN    0x00ff00
#define CYAN     0x007777
#define MAGENTA  0x770077
#define YELLOW   0x777700
#define WHITE    0xFFFFFF

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

// ---------- БИБЛИОТЕКИ -----------

// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------
float x_vel[PIXEL_AMOUNT];
float y_vel[PIXEL_AMOUNT];
float x_dist[PIXEL_AMOUNT];
float y_dist[PIXEL_AMOUNT];
float friction[PIXEL_AMOUNT];
float bounce[PIXEL_AMOUNT];
byte color[PIXEL_AMOUNT];

unsigned long integrTimer;
float stepTime, loopTime;

// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------

void setup() {
  Serial.begin(9600);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);

  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    x_vel[i] = (float)random(1, 8) / 10;  // получить в диапазоне 0.1 - 0.9;
    y_vel[i] = (float)random(1, 8) / 10;  // получить в диапазоне 0.1 - 0.9;
    x_dist[i] = MATR_X_M / 2;
    y_dist[i] = MATR_Y_M / 2;
  }
  randomSeed(analogRead(0));
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    friction[i] = (float)random(1, 8) / 10;  // получить в диапазоне 0.1 - 0.9
    bounce[i] = (float)random(1, 8) / 10;
    color[i] = map(i, 0, PIXEL_AMOUNT, 0, 7);
    Serial.println(color[i]);
  }
}

void loop() {
  loopTime = millis();
  integrate();
  strip.clear();
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    byte nowDistX = floor(x_dist[i] * DIST_TO_LED_X);
    byte nowDistY = floor(y_dist[i] * DIST_TO_LED_Y);
    pixelDraw(nowDistX, nowDistY, color[i]);
  }
  strip.show();
  Serial.println(millis() - loopTime);
}

void pixelDraw(byte x, byte y, byte colorNum) {
  if (y % 2 == 0)                                              // если чётная строка
    strip.setPixelColor(y * MATR_X + x, colors[colorNum]);                // заливаем в прямом порядке
  else                                                         // если нечётная
    strip.setPixelColor(y * MATR_X + MATR_X - x - 1, colors[colorNum]);    // заливаем в обратном порядке
}

void integrate() {
  // получить углы с mpu


  // расчёт времени шага интегрирования
  stepTime = (float)((long)micros() - integrTimer) / 1000000;

  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    float thisAccel_x = 0;
    float thisAccel_y = 0;

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
  integrTimer = micros();
}


