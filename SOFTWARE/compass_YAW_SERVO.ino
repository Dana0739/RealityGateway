// библиотека для работы I²C
#include <Wire.h>
// библиотека для работы с модулями IMU
#include <troyka-imu.h>
#include <Servo.h>

float x_zero = 13921;
float y_zero = 7514;
float z_zero = -9947;
float g_zero = 330;
float gx_zero = 330;

float xmax;
float xmin;
float ymax;
float ymin;
float zmax;
float zmin;

float pitch;
float roll;

float x;
float y;
float z;
float xr;
float yr;
float az;

float xp;
float yp;
float zp;
float xpr;
float ypr;
float azp;

float gx;
float gy;
float gz;

float gxp;
float gyp;
float gzp;

double k = 0.3;
double k1 = 0.3;

// создаём объект для работы с компасом
Compass compass;
Servo yaw;

void setup()
{
  // открываем последовательный порт
  Serial.begin(9600);

  // выводим сообщение о начале инициализации
  Serial.println("Begin init...");
  // инициализация компаса
  compass.begin();
  // выводим сообщение об удачной инициализации
  Serial.println("Init completed");
  // устанавливаем чувствительность
  compass.setRange(RANGE_4);

  yaw.attach(9);

  compass.readXYZ_Calib();
  
  xp = compass.x_cal - x_zero;
  yp = compass.y_cal - y_zero;
  zp = compass.z_cal - z_zero;

  gxp = analogRead(A0) - gx_zero + 6;
  gyp = analogRead(A1) - g_zero + 8;
  gzp = analogRead(A2) - g_zero;
  

  pitch = atan2(gxp, gzp);
  roll = atan2(gyp, gzp);
  //roll -= M_PI/2;
  //pitch -= M_PI/2;

  /*if(pitch < 0)
    pitch += 2*PI;

  if(pitch > 2*PI)
    pitch -= 2*PI;

  if(roll < 0)
    roll += 2*PI;

  if(roll > 2*PI)
    roll -= 2*PI;*/


  xpr = xp*cos(pitch) + yp*sin(roll)*sin(pitch) + zp*cos(roll)*sin(pitch);
  ypr = yp*cos(roll) - zp*sin(roll);

  float heading = atan2(-ypr, xpr);

  if(heading < 0)
    heading += 2*PI;

  if(heading > 2*PI)
    heading -= 2*PI;

  azp = heading * 180/M_PI;
}

void loop()
{
  compass.readXYZ_Calib();
  
  x = filter (compass.x_cal - x_zero, xp, k);
  y = filter (compass.y_cal - y_zero, yp, k);
  z = filter (compass.z_cal - z_zero, zp, k);

  /*if (x > xmax) xmax = x;
  if (x < xmin) xmin = x;
  if (y > xmax) ymax = y;
  if (y < xmin) ymin = y;
  if (z > xmax) zmax = z;
  if (z < xmin) zmin = z;
  x_zero = (xmax + xmin)/2;
  y_zero = (ymax + ymin)/2;
  z_zero = (zmax + zmin)/2;*/

  gx = filter(analogRead(A0) - gx_zero + 6, gxp, k);
  gy = filter(analogRead(A1) - g_zero + 8, gyp, k);
  gz = filter(analogRead(A2) - g_zero, gzp, k);

  pitch = atan2(gx, gz);
  roll = atan2(gy, gz);
  //roll -= M_PI/2;
  //pitch -= M_PI/2;

  /*if(pitch < 0)
    pitch += 2*PI;

  if(pitch > 2*PI)
    pitch -= 2*PI;

  if(roll < 0)
    roll += 2*PI;

  if(roll > 2*PI)
    roll -= 2*PI;*/

  //xr = filter(x*cos(pitch) + y*sin(roll)*sin(pitch) + z*cos(roll)*sin(pitch), xpr, k1);
  xr = filter(x*cos(pitch) - z*sin(pitch), xpr, k1);
  yr = filter(y*cos(roll) - z*sin(roll), ypr, k1);

  float heading = atan2(-yr, xr);

  if(heading < 0)
    heading += 2*PI;

  if(heading > 2*PI)
    heading -= 2*PI;

  az = heading * 180/M_PI;

  az = filter (az, azp, k);

  Serial.print(gx);
  Serial.print(" ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.print(gz);
  Serial.print(" | ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.print(" | ");
  Serial.print((int)(pitch*180/M_PI));
  Serial.print(" ");
  Serial.print((int)(roll*180/M_PI));
  Serial.print(" | ");
  /*Serial.print(sin(pitch));
  Serial.print(" ");
  Serial.print(cos(pitch));
  Serial.print(" ");
  Serial.print(sin(roll));
  Serial.print(" ");
  Serial.print(cos(roll));
  Serial.print(" ");*/
  Serial.print(xr);
  Serial.print(" ");
  Serial.print(yr);
  Serial.print(" | ");
  Serial.print(az);
  Serial.println(" ");

  yaw.write(az);

  xp = x;
  yp = y;
  zp = z;
  gxp = gx;
  gyp = gy;
  gzp = gz;
  azp = az;
  xpr = xr;
  ypr = yr;

  delay(0);
}

float filter (float inp, float prev, double kal){
  return (kal*inp + (1-kal)*prev);
}

