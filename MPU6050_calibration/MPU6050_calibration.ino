// --------------------- НАСТРОЙКИ ----------------------
#define buffersize = 70;   // количество замеров для определения средних значений
#define acel_deadzone = 10;  // точность калибровки акселерометра (по умолчанию 8)
#define gyro_deadzone = 6;   // точность калибровки гироскопа (по умолчанию 2)
// --------------------- НАСТРОЙКИ ----------------------

// I2Cdev и MPU6050 библиотеки должны быть установлены
#include "I2Cdev.h"
#include "MPU6050.h"
// в дефолтном конструкторе MPU6050 библиотеки используется I2C адрес 0x68
// однако можно передать в перегруженный конструктор явный адрес
// 0x68  address pin low (GND), default for InvenSense evaluation board
// 0x69  address pin high (VCC)

MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {
  // initialize serial communication
  Serial.begin(9600);

  // initialize device
  accelgyro.initialize();

  // start message
  Serial.println(F("MPU6050 Calibration Sketch"));
  Serial.println(F("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n"));
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);

  // confirm start
  while (Serial.available() && Serial.read()); // empty buffer
  Serial.println(F("\nSend any character to start calibration.\n"));
  while (!Serial.available()) {
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again

  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  if (state == 0) {
    Serial.println("\nReading sensors for the first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state == 1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state == 2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.println("\nSensor readings with offsets:");
    Serial.print("acelX,\tacelY,\tacelZ,\tgiroX,\tgiroY,\tgiroZ");
    Serial.print(mean_ax);
    Serial.print(",\t");
    Serial.print(mean_ay);
    Serial.print(",\t");
    Serial.print(mean_az);
    Serial.print(",\t");
    Serial.print(mean_gx);
    Serial.print(",\t");
    Serial.print(mean_gy);
    Serial.print(",\t");
    Serial.println(mean_gz);
    Serial.println("Your offsets:");
    Serial.println("acelX,\tacelY,\tacelZ,\tgiroX,\tgiroY,\tgiroZ");
    Serial.print(ax_offset);
    Serial.print(",\t");
    Serial.print(ay_offset);
    Serial.print(",\t");
    Serial.print(az_offset);
    Serial.print(",\t");
    Serial.print(gx_offset);
    Serial.print(",\t");
    Serial.print(gy_offset);
    Serial.print(",\t");
    Serial.println(gz_offset);
    Serial.println(F("Check that your sensor readings are close to 0 0 16384 0 0 0"));
    Serial.println(F("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)"));
    while (1);
  }
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors() {
  long buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  for (int i = 0; i < (buffersize + 101); i++) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  int ready;
  while (ready != 6) {
    ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= gyro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);

    if (abs(mean_gy) <= gyro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);

    if (abs(mean_gz) <= gyro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);
  }
}
