#include <ArduinoJson.h>
#include <Wire.h>
#include <troyka-imu.h>
#include <Servo.h>
#include <SoftwareSerial.h>

const double compassCalibrationBias[3] = {
  524.21,
  3352.214,
  -1402.236
};

const double compassCalibrationMatrix[3][3] = {
  {1.757, 0.04, -0.028},
  {0.008, 1.767, -0.016},
  { -0.018, 0.077, 1.782}
};

float x_zero;
float y_zero;
float z_zero;

float gx_zero = 520;
float gy_zero = 520;
float gz_zero = 520;

float xinp;
float yinp;
float zinp;
float xmax = -1000000;
float xmin = 1000000;
float ymax = -1000000;
float ymin = 1000000;
float zmax = -1000000;
float zmin = 1000000;

float pitch;
float roll;

float x;
float y;
float z;
float xr;
float yr;
float az;
float az_temp;
float az_temp1;

float x1;
float y1;
float z1;
float x1p;
float y1p;
float z1p;

float xp;
float yp;
float zp;
float xpr;
float ypr;
float azp;

float gx;
float gy;
float gz;
float gz2;

float gxp;
float gyp;
float gzp;
float gz2p;

float yaw_write;

float yaw_zero = 0;
float yaw_offset = 0;

double k = 0.3;
double k1 = 0.3;

int init_countdown = 0;

Compass compass;
Servo YAW;
Servo PITCH;

SoftwareSerial conn(10, 11);

float filter (float inp, float prev, double kal)
{
  return (kal * inp + (1 - kal) * prev);
}


void print_g()
{
  Serial.print(gx);
  Serial.print(" ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.print(gz);
  Serial.print(" | ");
}


void print_c()
{
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.print(" | ");
}


void print_r()
{
  Serial.print(xr);
  Serial.print(" ");
  Serial.print(yr);
  Serial.print(" | ");
}


void print_i()
{
  Serial.print((int)(pitch * 180 / M_PI));
  Serial.print(" ");
  Serial.print((int)(roll * 180 / M_PI));
  Serial.print(" | ");
}


void yaw_calibration()
{
  Serial.println("Initiating yaw calibration...");
  Serial.println("Perform 360 degrees horizontal turn.");
  Serial.println();
  digitalWrite(13, HIGH);
  delay(1000);

  while ((Serial.read() == -1) && (digitalRead(2) == HIGH)) {
    compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);

    xinp = compass.readCalibrateX();
    yinp = compass.readCalibrateY();

    xmax = max(xmax, xinp);
    ymax = max(ymax, yinp);
    xmin = min(xmin, xinp);
    ymin = min(ymin, yinp);

    x_zero = (xmax + xmin) / 2;
    y_zero = (ymax + ymin) / 2;

    Serial.print(xmax);
    Serial.print(" ");
    Serial.print(xmin);
    Serial.print(" ");
    Serial.print(x_zero);
    Serial.print(" ");
    Serial.print(ymax);
    Serial.print(" ");
    Serial.print(ymin);
    Serial.print(" ");
    Serial.println(y_zero);
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(50);
  }

  Serial.println();
  Serial.println("Check zero values.");
  Serial.println(x_zero);
  Serial.println(y_zero);
  Serial.println();

  digitalWrite(13, HIGH);
  delay(100);

  while ((Serial.read() == -1) && (digitalRead(2) == HIGH)) {};

  Serial.println("Perform 360 degrees vertical turn. Send anything when done.");
  Serial.println();

  digitalWrite(13, HIGH);
  delay(1000);

  while ((Serial.read() == -1) && (digitalRead(2) == HIGH)) {
    compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);

    zinp = compass.readCalibrateZ();

    zmax = max(zmax, zinp);
    zmin = min(zmin, zinp);

    z_zero = (zmax + zmin) / 2;

    Serial.println(z_zero);

    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(50);
  }
  Serial.println();
  Serial.println("Check zero value. Send anything when done.");
  Serial.println(z_zero);
  Serial.println();

  digitalWrite(13, HIGH);
  delay(100);

  while ((Serial.read() == -1) && (digitalRead(2) == HIGH)) {};

  Serial.println("Calibration complete. Send anything to launch.");
  Serial.println();

  delay(100);

  while ((Serial.read() == -1) && (digitalRead(2) == HIGH)) {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  };
  delay(1000);
}


void sync_reset() {
  Serial.println("Yaw sync initiated. Place the pointer forward.");
  Serial.println("3 seconds standby...");
  for (int i = 2048; i >= 2; i /= 2) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(i);
  }

  if (yaw_zero < 70) yaw_offset = 70;
  if (yaw_zero > 290) yaw_offset = -70;
  if ((yaw_zero >= 70) && (yaw_zero <= 290)) yaw_offset = 0;

  //az += yaw_offset;
  yaw_zero = az - 70;
  if (yaw_zero < 0) yaw_zero += 360;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Begin init...");
  compass.begin();
  Serial.println("Init completed");
  compass.setRange(RANGE_4_GAUSS);

  compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);

  pinMode(2, INPUT);

  pinMode(13, OUTPUT);

  yaw_calibration();

  xp = compass.readCalibrateX() - x_zero;
  yp = compass.readCalibrateY() - y_zero;
  zp = compass.readCalibrateZ() - z_zero;

  gxp = -1*(analogRead(A0) - gx_zero);
  gyp = analogRead(A1) - gy_zero;
  gzp = -1*(analogRead(A2) - gz_zero);

  roll = atan2(gyp, gzp);
  gz2p = gyp * sin(roll) + gzp * cos(roll);
  pitch = atan2(-gxp, gz2p);

  xpr = xp * cos(pitch) - zp * sin(pitch);
  ypr = yp * cos(roll) - zp * sin(roll);

  y1p = zp * sin(roll) - yp * cos(roll);
  z1p = yp * sin(roll) + zp * cos(roll);
  x1p = xp * cos(pitch) + zp * sin(pitch);

  float heading = atan2(y1p, x1p);

  if (heading < 0)
    heading += 2 * PI;

  if (heading > 2 * PI)
    heading -= 2 * PI;

  azp = heading * 180 / M_PI;

  YAW.attach(9);
  PITCH.attach(8);

  conn.begin(57600);

}

void loop()
{
  compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);

  x = filter (compass.readCalibrateX() - x_zero, xp, k);
  y = filter (compass.readCalibrateY() - y_zero, yp, k);
  z = filter (compass.readCalibrateZ() - z_zero, zp, k);

  gx = filter(-1*(analogRead(A0) - gx_zero), gxp, k);
  gy = filter(analogRead(A1) - gy_zero, gyp, k);
  gz = filter(-1*(analogRead(A2) - gz_zero), gzp, k);

  //pitch = atan2(gx, gz); //- 30*PI/180;
  //roll = atan2(gy, gz);

  roll = atan2(gy, gz);
  gz2 = gy * sin(roll) + gz * cos(roll);
  pitch = atan2(-gx, gz2);

  //xr = filter(x*cos(pitch) + y*sin(roll)*sin(pitch) + z*cos(roll)*sin(pitch), xpr, k1);
  xr = filter(x * cos(pitch) - z * sin(pitch), xpr, k1);
  yr = filter(y * cos(roll) - z * sin(roll), ypr, k1);

  y1 = z * sin(roll) - y * cos(roll);
  z1 = y * sin(roll) + z * cos(roll);
  x1 = x * cos(pitch) + z * sin(pitch);

  float heading = atan2(-y1, x1);

  //float heading = atan2(-yr, xr);

  if (heading < 0)
    heading += 2 * PI;

  if (heading > 2 * PI)
    heading -= 2 * PI;

  az = heading * 180 / M_PI;

  az = filter (az, azp, k);

  az_temp = az;

  if (yaw_zero > 220) {
    if ((az_temp > 220) && (az_temp < 360)) az_temp -= 220; else az_temp += 140;
    az_temp1 = az_temp - yaw_zero + 220;
  } else az_temp1 = az_temp - yaw_zero;

  //az_temp -= yaw_zero;

  yaw_write = az_temp1 * 110 / 140;

  //print_g();
  //print_c();
  //print_i();
  //print_r();
  //Serial.print(az);
  //Serial.print(" ");
  //Serial.print(az_temp);
  //Serial.print(" ");
  //Serial.print(az_temp1);
  //Serial.print(" ");
  //Serial.print(yaw_zero);
  //Serial.print(" ");
  //Serial.print(yaw_write);
  //Serial.print(" ");
  //Serial.print(yaw_offset);
  //Serial.print(" ");
  //Serial.print(yaw_zero);
  //Serial.println(" ");



  if ((yaw_write > 0) && (yaw_write < 90)) YAW.write(yaw_write);
  if ((pitch * 180 / PI * 120 / 150 > 0) && (pitch * 180 / PI * 120 / 150 < 120)) PITCH.write(pitch * 180 / PI * 120 / 150);

  if (init_countdown > 20) {
    /*
      conn.write((int)yaw_write);
      Serial.print((int)(yaw_write));
      Serial.print(" ");
      conn.write((int)(pitch*180/PI*120/150));
      Serial.println((int)((pitch*180/PI-90)*120/150));
    */
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["pitch"] = pitch * 180 / PI * 120 / 150;
    root["yaw"] = yaw_write;
    root.printTo(Serial);
    Serial.println();
    root.printTo(conn);
    conn.println();
  }

  xp = x;
  yp = y;
  zp = z;
  gxp = gx;
  gyp = gy;
  gzp = gz;
  azp = az;
  xpr = xr;
  ypr = yr;


  if ((init_countdown == 20) || (Serial.read() != -1) || (digitalRead(2) == LOW)) sync_reset();
  init_countdown += 1;

  digitalWrite(13, HIGH);
  delay(10);
  digitalWrite(13, LOW);
  delay(5);
}


