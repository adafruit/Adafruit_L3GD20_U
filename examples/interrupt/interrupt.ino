/*
 * L3GD20H Interrupt example.
 *
 * Breakout: https://www.adafruit.com/product/1032
 * 10DOF: https://www.adafruit.com/product/1604
 *
 * Connect DRDY (or GRDY on the 10DOF) to interrupt 0
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>


#define DEBUG

// Set the output rate.  This is independent of the sensor sampling.
#define SERIAL_OUTPUT_HZ 10

Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

unsigned int loop_count;
unsigned int gyro_count;

unsigned long serial_output_timer;

float gx, gy, gz;

volatile boolean gyroDataReady = false;


void setup() {
  Serial.begin(115200);

  attachInterrupt(0, gyroDataReadyISR, RISING);

  if (!gyro.begin(GYRO_RANGE_250DPS)) {
    Serial.print(F("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  gyro.enableDRDYInterrupt(true);
  gyro.setOutputDataRate(GYRO_ODR_190);
}


void gyroDataReadyISR () {
  gyroDataReady = true;
}


void loop() {
#ifdef DEBUG
  if (millis() > 1000) {
    Serial.println();
    Serial.print(F("Loop count: ")); Serial.println(loop_count);
    Serial.print(F("L3GD20 reads: ")); Serial.println(gyro_count);

    while(1) {}
  }
#endif

  loop_count++;

  if (gyroDataReady) {
    gyro_count++;
    gyroDataReady = false;

    sensors_event_t event;

    gyro.getEvent(&event);
    gx = event.gyro.x;
    gy = event.gyro.y;
    gz = event.gyro.z;
  }

  if (SERIAL_OUTPUT_HZ > 0 && millis() > serial_output_timer + (1000 / SERIAL_OUTPUT_HZ)) {
    serial_output_timer = millis();

    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
  }

  // Guard against millis() rollover
  if (serial_output_timer > millis()) {
    serial_output_timer = 0;
  }
}
