#include <ArduinoJson.h>

String receive;
char symb;

long cur_t;
long prev_t;
long inter;
long per = 50;
long zero_edge = 5;

int l;
int r;

double ctrl_l;
boolean engaged_l;
double ctrl_l_prev;
double ctrl_r;
boolean engaged_r;
double ctrl_r_prev;

boolean front_r;
boolean front_l;

long eng_t_l = 0;
long idl_t_l = 0;
long eng_t_r = 0;
long idl_t_r = 0;

int pow_l = 12;
int ctr_l_p = 11;
int ctr_l_n = 10;
int l_pin = 9;

int pow_r = 5;
int ctr_r_p = 4;
int ctr_r_n = 3;
int r_pin = 2;

long zero_l;
long zero_r;

boolean cold_start_l;
boolean cold_start_r;

int zero_countdown = 0;

void setup() {
  Serial.begin(9600);

  pinMode(pow_l, OUTPUT);
  pinMode(ctr_l_p, OUTPUT);
  pinMode(ctr_l_n, OUTPUT);
  pinMode(l_pin, OUTPUT);

  pinMode(pow_r, OUTPUT);
  pinMode(ctr_r_p, OUTPUT);
  pinMode(ctr_r_n, OUTPUT);
  pinMode(r_pin, OUTPUT);

  receive = "";
  symb = ' ';

  turnOff(l_pin);
  turnOff(r_pin);

  direct(0, 1);
  direct(1, 1);

  front_r = true;
  front_l = true;
}

void turnOff(int x) {
  digitalWrite(x, HIGH);
}

void turnOn(int x) {
  digitalWrite(x, LOW);
}

void direct (int side, int head) {
  if (side == 0){
    digitalWrite(pow_l, 0);
    delay(2);
    if (head == 0){
      digitalWrite(ctr_l_p, 0);
      digitalWrite(ctr_l_n, 1);
    } else {
      digitalWrite(ctr_l_p, 1);
      digitalWrite(ctr_l_n, 0);
    }
    digitalWrite(pow_l, 1);
  } else {
    digitalWrite(pow_r, 0);
    delay(2);
    digitalWrite(pow_r, 0);
    if (head == 0){
      digitalWrite(ctr_r_p, 0);
      digitalWrite(ctr_r_n, 1);
    } else {
      digitalWrite(ctr_r_p, 1);
      digitalWrite(ctr_r_n, 0);
    }
    digitalWrite(pow_r, 1);
  }
}

void loop() {
  StaticJsonBuffer<200> jsonBuffer;
  zero_countdown++;
  if (Serial.available()) {
    symb = Serial.read();
    if (symb != '$') {
      receive += symb;
    } else {
      JsonObject& root = jsonBuffer.parseObject(receive);
      receive = "";
      l = root["L"];
      r = root["R"];
      zero_countdown = 0;
    }
  }
  if (zero_countdown > 250) {
    l = 0;
    r = 0;
  }

  cur_t = millis();
  inter = cur_t - prev_t;
  prev_t = cur_t;

  ctrl_r = r;
  ctrl_l = l;
  ctrl_r /= 512;
  ctrl_l /= 512;
  ctrl_r *= per;
  ctrl_l *= per;

  if (abs(ctrl_l) < zero_edge) ctrl_l = 0;
  if (abs(ctrl_r) < zero_edge) ctrl_r = 0;

  if ((ctrl_l_prev < zero_edge)&&(ctrl_l >= zero_edge)){
    ctrl_l = per;
  }
  if ((ctrl_l_prev > -zero_edge)&&(ctrl_l <= -zero_edge)){
    ctrl_l = -per;
  }
  if ((ctrl_r_prev < zero_edge)&&(ctrl_r >= zero_edge)){
    ctrl_r = per;
  }
  if ((ctrl_r_prev > -zero_edge)&&(ctrl_r <= -zero_edge)){
    ctrl_r = -per;
  }
  
  if ((front_r)&&(ctrl_r < 0)){
    direct(1, 0);
    front_r = false;
  }
  if ((!front_r)&&(ctrl_r > 0)){
    direct(1, 1);
    front_r = true;
  }
  if ((front_l)&&(ctrl_l < 0)){
    direct(0, 0);
    front_l = false;
  }
  if ((!front_l)&&(ctrl_l > 0)){
    direct(0, 1);
    front_l = true;
  }

  if (engaged_l) {
    eng_t_l += inter;
    if (eng_t_l > ctrl_l) {
      engaged_l = false;
      turnOff(l_pin);
      idl_t_l = 0;
    }
  } else {
    idl_t_l += inter;
    if (idl_t_l > per - ctrl_l) {
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
    if (idl_t_r > per - ctrl_r) {
      engaged_r = true;
      turnOn(r_pin);
      eng_t_r = 0;
    }
  }

  ctrl_l_prev = ctrl_l;
  ctrl_r_prev = ctrl_r;
}
