#include <FourBitLedDigitalTube.h>
#include <Arduino.h>

int l1 = 2, l2 = 3, l3 = 4, l4 = 5, l5 = 6;
#define b1 A5
#define b2 A4
#define b3 A3
#define b4 A2
#define b5 A1
int buz = 12;

const uint8_t DIO {11};
const uint8_t RCLK {10};
const uint8_t SCLK {9};
TM74HC595LedTube display(SCLK, RCLK, DIO);

int buz1 = 220, buz2 = 247, buz3 = 262, buz4 = 294, buz5 = 330;

int runde = 0;
const byte maxrunde = 50;
bool done = true;
int leddelay = 500;
int step = 0;
bool correct = true;
int score = 0, highscore = 0;

int b1val = 1,b2val = 1,b3val = 1,b4val = 1,b5val = 1;
int b1last = 1, b2last = 1, b3last = 1, b4last = 1, b5last = 1;
int pressval = 0;

int lst[maxrunde];
int buzlst[5] = {buz1, buz2, buz3, buz4, buz5};
int leds[5] = {l1,l2,l3,l4,l5};

void setup() {
  display.begin();
  display.print(score, true);

  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(l3, OUTPUT);
  pinMode(l4, OUTPUT);
  pinMode(l5, OUTPUT);
  
  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, INPUT_PULLUP);
  pinMode(b3, INPUT_PULLUP);
  pinMode(b4, INPUT_PULLUP);
  pinMode(b5, INPUT_PULLUP);

  pinMode(buz, OUTPUT);
  pinMode(A0, INPUT);

  noTone(buz);
  Serial.begin(9600);
  allLeds(1);
  starttone();
  allLeds(0);

  randomSeed(analogRead(A0)); //legge til potentiometer hastighet + seed
}

void allLeds(int mode) {
  for(int i = 0; i < sizeof(leds); i++) {
    digitalWrite(leds[i], mode);
  }
}

void beep(int note, int duration) {
  tone(buz, note, duration);
  delay(50); 
}

void deadtone() {
  beep(261, 300); // C4
  beep(220, 300); // A3
  beep(174, 300); // F3
  beep(261, 500); // C4
  beep(220, 300); // A3

  delay(1000);
}

void nextroundtone() {
  beep(523, 150); // C5
  beep(659, 150); // E5
  beep(784, 150); // G5
  beep(1047, 300); // C6
  beep(784, 150); // G5
  beep(988, 150); // B5
  beep(1175, 150); // D6
  beep(1568, 400); // G6

  delay(1000);
}

void starttone() {
  beep(523, 250); // C5
  beep(659, 250); // E5
  beep(784, 250); // G5
  beep(1047, 350); // C6
  beep(659, 250); // E5
  beep(784, 250); // G5
  beep(880, 250); // A5
  beep(698, 250); // F5
  beep(587, 250); // D5
  beep(523, 350); // C5
  beep(493, 250); // B4
  beep(523, 250); // C5
  beep(587, 250); // D5
  beep(392, 500); // G4
  beep(523, 250); // C5

  delay(1000);
}

void victorytone(){
  beep(523, 500); // C5
  beep(659, 500); // E5
  beep(784, 500); // G5
  beep(1047, 600); // C6
  beep(659, 500); // E5
  beep(784, 500); // G5
  beep(1047, 800); // C6
  beep(1319, 1000); // E6

  delay(1000);
}

void loop() {
  b1val = digitalRead(b1);
  b2val = digitalRead(b2);
  b3val = digitalRead(b3);
  b4val = digitalRead(b4);
  b5val = digitalRead(b5);
  pressval = -1;

  if (b1val == 1 && b1last == 0) {
    pressval = 0;
  }
  b1last = b1val;
  if (b2val == 1 && b2last == 0) {
    pressval = 1;
  }
  b2last = b2val;
  if (b3val == 1 && b3last == 0) {
    pressval = 2;
  }
  b3last = b3val;
  if (b4val == 1 && b4last == 0) {
    pressval = 3;
  }
  b4last = b4val;
  if (b5val == 1 && b5last == 0) {
    pressval = 4;
  }
  b5last = b5val;
  
  allLeds(0);
  noTone(buz);
  if(b1val == LOW) {
    digitalWrite(l1, HIGH);
    tone(buz, buz1);
  }
  if(b2val == LOW) {
    digitalWrite(l2, HIGH);
    tone(buz, buz2);
  }
  if(b3val == LOW) {
    digitalWrite(l3, HIGH);
    tone(buz, buz3);
  }
  if(b4val == LOW) {
    digitalWrite(l4, HIGH);
    tone(buz, buz4);
  }
  if(b5val == LOW) {
    digitalWrite(l5, HIGH);
    tone(buz, buz5);
  }

  if (done) {
    runde ++;
    done = false;
    step = 0;
    lst[runde-1] = random(0,5);
    for(int i = 0; i < runde; i++) {
      leddelay = map(analogRead(A0), 0,1024, 50, 2000);
      delay(leddelay);
      Serial.println(lst[i]);
      digitalWrite(leds[lst[i]], HIGH);
      tone(buz, buzlst[i]);
      delay(leddelay);
      digitalWrite(leds[lst[i]], LOW);
      noTone(buz);
    }
  }
  if (done == false) {
    if (pressval != -1) {
      if(pressval != lst[step]) {
        Serial.println("Wrong !!!");
        correct = false;
        step = runde;
      }
      else if(pressval == lst[step]) {
        Serial.println("Correct");
        step++;
      }
    }
    if(step == runde) {
      done = true;
      allLeds(1);
      delay(400);
      allLeds(0);
      if (correct) {
        score ++;
        display.print(score, true);
        nextroundtone();
      }
    }
  }

  if (correct == false) {
    display.print("----");
    deadtone();
    display.print(score, true);
    display.blink(800, 200);
    while(digitalRead(b1) == HIGH && digitalRead(b2) == HIGH && digitalRead(b3) == HIGH && digitalRead(b4) == HIGH && digitalRead(b5) == HIGH) {
    }
    display.noBlink();
    display.print("HI");
    delay(800);
    if (score >= highscore) {
      highscore = score;
    }
    display.print(highscore);
    if (score >= highscore) {
      victorytone();
    }
    delay(2000);
    correct = true;
    int lst[maxrunde];
    runde = 0;
    score = 0;
    display.print(score, true);
    allLeds(1);
    starttone();
    allLeds(0);
    delay(1000);
  }
  delay(50);
}
