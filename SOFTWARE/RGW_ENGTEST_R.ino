#include <ArduinoJson.h>
#include <AFMotor.h>

AF_DCMotor L(3, MOTOR12_64KHZ);
AF_DCMotor R(4, MOTOR12_64KHZ);

String receive;
boolean symb;

int prev_left;
int prev_right;
int left;
int right;

int EngageTime = 100;

int Pace_1 = 75;
int Pace_2 = 150;
int Pace_3 = 200;
int Pace_4 = 255;

int Lim_1 = 150;
int Lim_2 = 250;
int Lim_3 = 350;
int Lim_4 = 450;
int Lim_5 = 511;

void setup() {
  Serial.begin(9600);
}

void loop() {
  StaticJsonBuffer<200> jsonBuffer;
  receive = "";
  symb = ' ';
  while (Serial.available()) {
    if (symb != '\n') {
      symb = Serial.read();
      receive += symb;
    } else {
      JsonObject& root = jsonBuffer.parseObject(receive);

      left = root["L"];
      right = root["R"];

      if ((prev_left < Lim_1) && (left > Lim_1)) {
        engage(0, true);
      }
      if ((prev_left > -Lim_1) && (left < -Lim_1)) {
        engage(0, false);
      }
      if ((prev_right < Lim_1) && (right > Lim_1)) {
        engage(1, true);
      }
      if ((prev_right > -Lim_1) && (right < -Lim_1)) {
        engage(1, false);
      }

      //LEFT
      if (inRange(abs(left), 0, Lim_1)) {
        neutralize(0);
      }
      if (inRange(abs(left), Lim_1, Lim_2)) {
        L.setSpeed(Pace_1);
      }
      if (inRange(abs(left), Lim_2, Lim_3)) {
        L.setSpeed(Pace_2);
      }
      if (inRange(abs(left), Lim_3, Lim_4)) {
        L.setSpeed(Pace_3);
      }
      if (inRange(abs(left), Lim_4, Lim_5)) {
        L.setSpeed(Pace_4);
      }

      //RIGHT
      if (inRange(abs(right), 0, Lim_1)) {
        neutralize(1);
      }
      if (inRange(abs(right), Lim_1, Lim_2)) {
        R.setSpeed(Pace_1);
      }
      if (inRange(abs(right), Lim_2, Lim_3)) {
        R.setSpeed(Pace_2);
      }
      if (inRange(abs(right), Lim_3, Lim_4)) {
        R.setSpeed(Pace_3);
      }
      if (inRange(abs(right), Lim_4, Lim_5)) {
        R.setSpeed(Pace_4);
      }

      val_update();
    }
  }
}

void neutralize (int side) {
  if (side == 0) {
    L.run(RELEASE);
    L.setSpeed(0);
  } else {
    R.run(RELEASE);
    R.setSpeed(0);
  }
}

void engage (int side, boolean heading) {
  if (side == 0) {
    if (heading) {
      L.run(FORWARD);
    } else {
      L.run(BACKWARD);
    }
    L.setSpeed(255);
    delay(100);
  } else {
    if (heading) {
      R.run(FORWARD);
    } else {
      R.run(BACKWARD);
    }
    R.setSpeed(255);
    delay(EngageTime);
  }
}

boolean inRange (int x, int a, int b) {
  if ((a <= x) && (x < b)) {
    return true;
  } else {
    return false;
  }
}

void val_update() {
  prev_left = left;
  prev_right = right;
  receive = "";
  symb = ' ';
}

