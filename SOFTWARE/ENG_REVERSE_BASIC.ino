int pow_l = 13;
int ctr_l_p = 12;
int ctr_l_n = 11;
int mia_l = 10;

int pow_r = 5;
int ctr_r_p = 4;
int ctr_r_n = 3;
int mia_r = 2;

void setup() {
  Serial.begin(9600);
  
  pinMode(pow_l, OUTPUT);
  pinMode(ctr_l_p, OUTPUT);
  pinMode(ctr_l_n, OUTPUT);
  pinMode(mia_l, OUTPUT);
  
  pinMode(pow_r, OUTPUT);
  pinMode(ctr_r_p, OUTPUT);
  pinMode(ctr_r_n, OUTPUT);
  pinMode(mia_r, OUTPUT);

  digitalWrite(mia_l, 1);
  digitalWrite(mia_r, 1);
}

void loop() {
  reverse(0, 0);
  reverse(1, 1);
  delay(3000);
  reverse(0, 1);
  reverse(1, 0);
  delay(3000);
}

void reverse (int side, int head) {
  if (side == 0){
    digitalWrite(pow_l, 0);
    delay(10);
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
    delay(5);
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


