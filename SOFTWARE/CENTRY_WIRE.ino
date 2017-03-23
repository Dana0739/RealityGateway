#include <ArduinoJson.h>

#include <Servo.h>
#include <SoftwareSerial.h>

Servo YAW;
Servo PITCH;

SoftwareSerial conn(10, 11);

float pos_YAW;
float pos_PITCH;
String s = "";
char ch;

void setup() {
  YAW.attach(9);
  PITCH.attach(8);
  conn.begin(57600);
  Serial.begin(57600);
}

void loop() {
  StaticJsonBuffer<200> jsonBuffer;
  
  if (conn.available()) {
    ch = conn.read();
    if (ch != '\r')
      s += ch;
    else {
      JsonObject& root = jsonBuffer.parseObject(s);
      pos_YAW = root["yaw"];
      pos_PITCH = root["pitch"];
    
      Serial.print(pos_YAW);
      Serial.print(" ");
      Serial.print(pos_PITCH);
      Serial.print(" ");
      Serial.println(s);
      
      if ((pos_YAW > 0)&&(pos_YAW < 115)) YAW.write(pos_YAW);
      if ((pos_PITCH > 0)&&(pos_PITCH < 120)) PITCH.write(pos_PITCH);

      s = "";
    }
  }
  /*
  JsonObject& root = jsonBuffer.parseObject(inp);

  pos_YAW = root["yaw"];
  pos_PITCH = root["pitch"];

  Serial.print(pos_YAW);
  Serial.print(" ");
  Serial.print(pos_PITCH);
  Serial.print(" ");
  Serial.println(inp);
  
  if ((pos_YAW > 0)&&(pos_YAW < 115)) YAW.write(pos_YAW);
  if ((pos_PITCH > 0)&&(pos_PITCH < 120)) PITCH.write(pos_PITCH);*/
}

