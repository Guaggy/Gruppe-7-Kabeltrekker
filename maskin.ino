//kode til selve maskinen
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>

//fremgang
#define mot3 A3
#define mot4 A4
#define trinnpin A5
int antalltrinn = 0;
int lastvaltrinn = 1;
float trinnd = 1500;
float trinnl = -trinnd;

//bom
#define motorInterfaceType 1
int dirPin = 2;
int stepPin = 3;
int zerobut = 9;
int steplst[] = {0,0,500,1000,200,100,300,100,100,100,300,100};
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
int sumstep = 0;
int step = 1;
int lstep = 0;

//ned-dytter
#define mot1 A1
#define mot2 A2
#define limbut 5
int push = 0;
int lastval = 0;
bool pushact = false;
int toucht = 0, touchd = 500;
bool pushfin = true;

//batteri
#define batpin A0
float minv = 3.5, maxv = 4.6;
int batper = 0;

//Radio
const byte address[][6] = {"00001","00002"};
#define CE_pin 7
#define CSN_pin 8
RF24 radio(CE_pin, CSN_pin);
float recdelay = 0;
float reclimit = 2*recdelay + 500;
float lastrec = 0;
bool sendable = false;

struct payloadrec {
  byte ch1; //step
  byte ch2; //motot fremgang
  byte ch3; //joystick push
  byte ch4; //b0
  byte ch5; //b1
  byte ch6; //b2
  byte ch7; //modus
  byte ch8; //antall trinn
};
payloadrec payloadrec;

struct payloadsend {
  byte ch1; //battery %
  byte ch2; //antall trinn kjørt
  byte ch3; //kjører 1/0
};
payloadsend payloadsend;

//annet
int l0 = 4;
bool fail = false;
int yval = 126;
int pval = 0;
int bval0 = 0;
int bval1 = 0;
int bval2 = 0;
int modus = 1;
int ptrinn = 0;
int go = 1; //1 = false, 2 = true
int amod = 0;

void setup() {
  Serial.begin(9600);
  pinMode(l0, OUTPUT);
  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);
  pinMode(mot3, OUTPUT);
  pinMode(mot4, OUTPUT);
  pinMode(zerobut, INPUT_PULLUP);
  pinMode(limbut, INPUT_PULLUP);
  pinMode(trinnpin, INPUT_PULLUP);
  pinMode(batpin, INPUT);

  //fremgang oppstart
  lastvaltrinn = digitalRead(trinnpin);

  //bom og ned-dytter oppstart
  digitalWrite(mot1, LOW);
  digitalWrite(mot2, LOW);
  myStepper.setPinsInverted(true);
  myStepper.setMaxSpeed(500);
	myStepper.setAcceleration(500);
  myStepper.setSpeed(200);

  //Nullstille bom
  myStepper.move(80);
  while (myStepper.run()) {
  }
  myStepper.setSpeed(-100);
  while (digitalRead(zerobut) == HIGH) {
    myStepper.runSpeed();
  }
  myStepper.stop();
  myStepper.setCurrentPosition(0);
  myStepper.setSpeed(300);

  //Nullstille ned-dytter
  while (digitalRead(limbut) == HIGH) {
    digitalWrite(mot1, LOW);
    digitalWrite(mot2, HIGH);
  }
  lastval = digitalRead(limbut);
  digitalWrite(mot1, LOW);
  digitalWrite(mot2, LOW);

  //Nullstille fremgang
  digitalWrite(mot3, LOW);
  digitalWrite(mot4, LOW);

  //Radio oppstart
  if (radio.begin()) {
    Serial.print("Boot OK");
  }
  else {
    Serial.print("No connection with NRF24");
    fail = true;
  }
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(0, address[0]);
  lastrec = millis();
}

void loop() {
  //batper = analogRead(batpin) / 4; //test
  batper = int(map(analogRead(batpin), minv*205, maxv*205, 0 ,100));

  myStepper.run();

  //Motta data
  sendable = false;
  radio.setPayloadSize(sizeof(payloadrec));
  radio.startListening();

  delay(5);
  if ((radio.available()) && (millis() >= lastrec + recdelay)) {
    fail = false;
    digitalWrite(l0, 0);
    sendable = true;
    lastrec = millis();

    radio.read(&payloadrec, sizeof(payloadrec));
    Serial.print(payloadrec.ch1);
    Serial.print("   ");
    Serial.print(payloadrec.ch2);
    Serial.print("   ");
    Serial.print(payloadrec.ch3);
    Serial.print("   ");
    Serial.print(payloadrec.ch4);
    Serial.print("   ");
    Serial.print(payloadrec.ch5);
    Serial.print("   ");
    Serial.print(payloadrec.ch6);
    Serial.print("   ");
    Serial.print(payloadrec.ch7);
    Serial.print("   ");
    Serial.println(payloadrec.ch8);
    Serial.println();

    if (payloadrec.ch1 != 0) {
      step = payloadrec.ch1;
    }
    if (payloadrec.ch2 != 0) {
      yval = payloadrec.ch2;
    }
    pval = payloadrec.ch3;
    bval0 = payloadrec.ch4;
    bval1 = payloadrec.ch5;
    bval2 = payloadrec.ch6;
    if (payloadrec.ch7 != 0) {
      modus = payloadrec.ch7;
    }
    if (payloadrec.ch8 != 0) {
      ptrinn = payloadrec.ch8;
    }
  }

  myStepper.run();

  //Sende data
  if (sendable) {
    radio.stopListening();
    radio.setPayloadSize(sizeof(payloadsend));
    delay(5);
    payloadsend.ch1 = batper;
    payloadsend.ch2 = antalltrinn;
    payloadsend.ch3 = go;
    radio.write(&payloadsend, sizeof(payloadsend));
    Serial.print(payloadsend.ch1);
    Serial.print("   ");
    Serial.print(payloadsend.ch2);
    Serial.print("   ");
    Serial.println(payloadsend.ch3);
  }

  myStepper.run();

  //auto start
  if ((ptrinn != 0) && (pval ==1)) {
    go = 2;
    amod = 0;
  }

  //fremgang
  if ((digitalRead(trinnpin) == 0) && (lastvaltrinn == 1) && (millis() >= trinnl + trinnd)) {
    antalltrinn++;
    toucht = millis();
    trinnl = millis();
  }
  lastvaltrinn = digitalRead(trinnpin);
  
  if (modus == 1) { //admin
    digitalWrite(mot3, LOW);
    digitalWrite(mot4, LOW);
    if (yval >= 200) {
      digitalWrite(mot3, LOW);
      digitalWrite(mot4, HIGH);
    }
    if (yval <= 55) {
      digitalWrite(mot3, HIGH);
      digitalWrite(mot4, LOW);
    }
    if (bval2 == 1) {
      antalltrinn = 0;
    }
  }

  if ((modus == 2) && (go == 2)) { //auto
    if (amod == 0) {
      digitalWrite(mot3, LOW);
      digitalWrite(mot4, HIGH);
      if (antalltrinn >= ptrinn) {
        amod++;
        antalltrinn = 0;
      }
    }
    if (amod == 1) {
      if (pushfin) {
        digitalWrite(mot3, HIGH);
        digitalWrite(mot4, LOW);
      }
      if((millis() >= toucht + touchd) && (pushfin)) {
        pushfin = false;
        digitalWrite(mot3, LOW);
        digitalWrite(mot4, LOW);
        push = 0;
      }
      if ((push == 1) && (!pushfin)) {
        digitalWrite(mot1, HIGH);
        digitalWrite(mot2, LOW);
      }
      if ((push == 2) && (!pushfin)) {
        digitalWrite(mot1, LOW);
        digitalWrite(mot2, HIGH);
      }
      if ((push >= 3) && (!pushfin)) {
        digitalWrite(mot1, LOW);
        digitalWrite(mot2, LOW);
        pushfin = true;
      }
      if ((digitalRead(limbut) == 0) && (lastval == 1) && (!pushfin)) {
        push = push + 1;
      }
      lastval = digitalRead(limbut);
     
      if ((antalltrinn >= ptrinn) && (pushfin)) {
        amod++;
        antalltrinn = 0;
      }
    }
    if (amod == 2) {
      digitalWrite(mot3, LOW);
      digitalWrite(mot4, HIGH);
      if (antalltrinn >= ptrinn) {
        amod++;
        antalltrinn = 0;
      }
    }
    if (amod == 3) {
      digitalWrite(mot3, LOW);
      digitalWrite(mot4, LOW);
      go = 1;
    }
  }

  //Mistet signal
  if ((millis() >= reclimit + lastrec) && fail == false) {
    safevalues();
    Serial.println("Lost connection");
    fail = true;
    digitalWrite(l0, 1);
  }

  //flytte bom
  if (step != lstep) {
    sumstep = 0;
    for (int i = 0; i <= step; i++) {
      sumstep += steplst[i];
    }
    myStepper.moveTo(sumstep);
    lstep = step;
  }
  myStepper.run();

  //ned-dytter
  if (modus == 1) { //admin
      if(bval1 == 1) {
      push = 1;
      pushact = true;
    }
    if ((push == 1) && (pushact)) {
      digitalWrite(mot1, HIGH);
      digitalWrite(mot2, LOW);
    }
    if ((push == 2) && (pushact)) {
      digitalWrite(mot1, LOW);
      digitalWrite(mot2, HIGH);
    }
    if ((push >= 3) && (pushact)) {
      digitalWrite(mot1, LOW);
      digitalWrite(mot2, LOW);
      pushact = false;
    }
    if ((digitalRead(limbut) == 0) && (lastval == 1) && (pushact)) {
      push = push + 1;
    }
    lastval = digitalRead(limbut);
  }
  if (modus == 2) { //auto

  }

  myStepper.run();
}

//Verdier ved mistet signal
void safevalues() {
  /*
  yval = 127;
  pval = 0;
  bval0 = 0;
  */
}
