#include <TroykaIMU.h>

Gyroscope gyro(GYRO_ADDRESS_V1);
Accelerometer accel(ACCEL_ADDRESS_V1);
Compass compass(COMPASS_ADDRESS_V1);
Barometer barometer(BARO_ADDRESS_V1);

float maxx, maxy, maxz;
float minx, miny, minz;

double g[3];
double h[3];

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

const double compassZero[3] = {
  //  -1.09,
  //  -3.64,
  //  2.36
  -0.53,
  -1.04,
  1.54,
};

double turnXX(double a[], double angle) {
  return a[0];
}

double turnXY(double a[], double angle) {
  return a[1] * cos(angle) - a[2] * sin(angle);
}

double turnXZ(double a[], double angle) {
  return a[1] * sin(angle) + a[2] * cos(angle);
}

double turnYX(double a[], double angle) {
  return a[0] * cos(angle) + a[2] * sin(angle);
}

double turnYY(double a[], double angle) {
  return a[1];
}

double turnYZ(double a[], double angle) {
  return - a[0] * sin(angle) + a[2] * cos(angle);
}

double getAzimuth(double g[], double h[]) {
  double gn[3], hn[3];

  double angleX = atan(g[1] / g[2]);

  gn[0] = g[0];
  gn[1] = g[1];
  gn[2] = g[2];
  hn[0] = h[0];
  hn[1] = h[1];
  hn[2] = h[2];

  gn[0] = turnXX(g, angleX);
  gn[1] = turnXY(g, angleX);
  gn[2] = turnXZ(g, angleX);
  hn[0] = turnXX(h, angleX);
  hn[1] = turnXY(h, angleX);
  hn[2] = turnXZ(h, angleX);

  g[0] = gn[0];
  g[1] = gn[1];
  g[2] = gn[2];
  h[0] = hn[0];
  h[1] = hn[1];
  h[2] = hn[2];

  double angleY = -atan(g[0] / g[2]);

  gn[0] = g[0];
  gn[1] = g[1];
  gn[2] = g[2];
  hn[0] = h[0];
  hn[1] = h[1];
  hn[2] = h[2];

  gn[0] = turnYX(g, angleY);
  gn[1] = turnYY(g, angleY);
  gn[2] = turnYZ(g, angleY);
  hn[0] = turnYX(h, angleY);
  hn[1] = turnYY(h, angleY);
  hn[2] = turnYZ(h, angleY);

  g[0] = gn[0];
  g[1] = gn[1];
  g[2] = gn[2];
  h[0] = hn[0];
  h[1] = hn[1];
  h[2] = hn[2];

  Serial.print(g[0]);
  Serial.print(" ");
  Serial.print(g[1]);
  Serial.print(" ");
  Serial.print(g[2]);
  Serial.print(" ");

  Serial.print(h[0]);
  Serial.print(" ");
  Serial.print(h[1]);
  Serial.print(" ");
  Serial.print(h[2]);
  Serial.print(" ");

  double aziTan = h[0] / h[1];
  double azimuth = atan(aziTan) * 180 / PI;

  return azimuth;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Begin init...");

  gyro.begin();
  accel.begin();
  compass.begin();
  barometer.begin();

  compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);
  compass.setRange(RANGE_4_GAUSS);

  Serial.println("Initialization completed");
  Serial.println("Gyroscope\t\t\tAccelerometer\t\t\tCompass\t\tBarometer");

  minx = 100;
  miny = 100;
  minz = 100;
  maxx = -100;
  maxy = -100;
  maxz = -100;
}

void loop() {
  g[0] = accel.readAX();
  g[1] = accel.readAY();
  g[2] = accel.readAZ();

  h[0] = compass.readCalibrateGaussX() - compassZero[0];
  h[1] = compass.readCalibrateGaussY() - compassZero[1];
  h[2] = compass.readCalibrateGaussZ() - compassZero[2];

  //    maxx = max(h[0], maxx);
  //    maxy = max(h[1], maxy);
  //    maxz = max(h[2], maxz);
  //
  //    minx = min(h[0], minx);
  //    miny = min(h[1], miny);
  //    minz = min(h[2], minz);

  //  Serial.print(g[0]);
  //  Serial.print(" ");
  //  Serial.print(g[1]);
  //  Serial.print(" ");
  //  Serial.print(g[2]);
  //  Serial.print(" ");
  //
  //  Serial.print(h[0]);
  //  Serial.print(" ");
  //  Serial.print(h[1]);
  //  Serial.print(" ");
  //  Serial.print(h[2]);
  //  Serial.print(" ");

  //    Serial.print((maxx + minx) / 2);
  //    Serial.print(" ");
  //    Serial.print((maxy + miny) / 2);
  //    Serial.print(" ");
  //    Serial.print((maxz + minz) / 2);
  //    Serial.print(" ");

  Serial.print(getAzimuth(g, h));

  Serial.println();

  delay(10);
}
