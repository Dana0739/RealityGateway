#include <ArduinoJson.h>

double k = 0.2;
double left;
double prev_left = -1000;
double right;
double prev_right = -1000;

void setup() {
  Serial.begin(9600);
//  pinMode(7, INPUT);
//  pinMode(A0, INPUT);
//  pinMode(A1, 
}

void loop() {
  if (prev_left == -1000) {
    prev_left = (analogRead(1) - 512);
    prev_right = (analogRead(2) - 512);
  } else {
    left = (analogRead(1) - 512);
    right = (analogRead(2) - 512);

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
  delay(1);
}
