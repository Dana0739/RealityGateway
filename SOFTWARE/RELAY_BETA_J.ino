#include <ArduinoJson.h>

double k = 0.2;
double left;
double prev_left = -1000;
double right;
double prev_right = -1000;
double zero_l;
double zero_r;

void setup() {
  Serial.begin(9600);
//  pinMode(7, INPUT);
//  pinMode(A0, INPUT);
//  pinMode(A1, 
  zero_l = analogRead(0);
  zero_r = analogRead(2);
}

void loop() {
  if (prev_left == -1000) {
    prev_left = (analogRead(0) - zero_l);
    prev_right = (analogRead(2) - zero_r);
  } else {
    left = (analogRead(0) - zero_l);
    right = (analogRead(2) - zero_r);

    left = k * left + (1 - k) * prev_left;
    right = k * right + (1 - k) * prev_right;

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["L"] = (int)left;
    root["R"] = (int)right;
    root.printTo(Serial);
    Serial.println();

    prev_left = left;
    prev_right = right;
  }
  delay(100);
}
