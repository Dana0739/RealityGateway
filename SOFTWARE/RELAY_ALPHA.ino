#include <ArduinoJson.h>

String receive;
char symb;

long cur_t;
long prev_t;
long inter; 
long per = 30;

int l;
int r;

double ctrl_l;
boolean engaged_l;
double ctrl_r;
boolean engaged_r;

long eng_t_l = 0;
long idl_t_l = 0;
long eng_t_r = 0;
long idl_t_r = 0;

int l_pin = 7;
int r_pin = 8;

long zero_l;
long zero_r;

void setup() {
  Serial.begin(9600);
  pinMode(l_pin, OUTPUT);
  pinMode(r_pin, OUTPUT);
  zero_l = analogRead(1);
  zero_r = analogRead(2);
}

void turnOff(int x){
  digitalWrite(x, HIGH);
}

void turnOn(int x){
  digitalWrite(x, LOW);
}

void loop() {
//  StaticJsonBuffer<200> jsonBuffer;
//  receive = "";
//  symb = ' ';
//  while (Serial.available()) {
//    if (symb != '\n') {
//      symb = Serial.read();
//      receive += symb;
//    } else {
//      JsonObject& root = jsonBuffer.parseObject(receive);
//      receive = "";
       
      cur_t = millis();
      inter = cur_t - prev_t;
      prev_t = cur_t;

//      l = root["L"];
//      r = root["R"];

      l = (analogRead(1) - zero_l);
      r = (analogRead(2) - zero_r);
     
      ctrl_r = r;
      ctrl_l = l;
      ctrl_r /= 512;
      ctrl_l /= 512;
      ctrl_r *= 30;
      ctrl_l *= 30;

//      if (ctrl_l < 20) ctrl_l = 0;
//      if (ctrl_r < 20) ctrl_r = 0;

//      Serial.print(r);
//      Serial.print(" ");
//      Serial.print(l);
//      Serial.print(" ");
//      Serial.print(ctrl_r);
//      Serial.print(" ");
//      Serial.print(inter);
//      Serial.println();
//
//      if (ctrl_l < 20) ctrl_l = 0;
//      if (ctrl_r < 20) ctrl_r = 0;
      
      if (engaged_l) {
        eng_t_l += inter;
        if (eng_t_l > ctrl_l) {
          engaged_l = false;
          turnOff(l_pin);
          idl_t_l = 0;
        }
      } else {
        idl_t_l += inter;
        if (idl_t_l > per-ctrl_l) {
          engaged_l = true;
          turnOn(l_pin);
          eng_t_l = 0;
        }
      }

      if (engaged_r) {
        eng_t_r += inter;
        if (eng_t_r > ctrl_r) {
          engaged_r = false;
          turnOff(r_pin);
          idl_t_r = 0;
        }
      } else {
        idl_t_r += inter;
        if (idl_t_r > per-ctrl_r) {
          engaged_r = true;
          turnOn(r_pin);
          eng_t_r = 0;
        }
      }
//    }
//  }
}




