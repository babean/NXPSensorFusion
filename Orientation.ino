// Full orientation sensing using NXP's advanced sensor fusion algorithm.
//
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.

#if defined (PARTICLE)
#include "application.h"
#define NO_MPL3115 // remove this if MPL3115 is polulated
#else
#include <Wire.h>
#include <EEPROM.h>
#endif
#include <NXPMotionSense.h>

NXPMotionSense imu;
NXPSensorFusion filter;

char szInfo[30];
unsigned long last_time=0;
bool timer;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, heading;

void setup() {
  Serial.begin(9600);
  imu.begin();
  filter.begin(100);
}

void loop() {
  System.sleep(SLEEP_MODE_CPU);
  if (Particle.connected() == false) {
    Particle.connect();
  }
  timer = (millis()-last_time) > 500;
  if (imu.available() && timer) {
      // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("YPR: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    sprintf(szInfo, "YPR: %.2f,%.2f,%.2f", heading, pitch, roll);
    Particle.publish(szInfo);
    last_time=millis();
  }
}

