#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>
#include <FastLED.h>

//led lys
#define RB_NUM_LEDS 2
#define LB_NUM_LEDS 2
#define B_NUM_LEDS 5
#define RB_DATA_PIN 33
#define LB_DATA_PIN 35
#define B_DATA_PIN 37
CRGB rightBlinker[RB_NUM_LEDS];
CRGB leftBlinker[LB_NUM_LEDS];
CRGB backLight[B_NUM_LEDS];
bool wantLights = true;

//fremgang
int moveMotorpin1 = 21;
int moveMotorpin2 = 20;
int stepCounterpin = 13;
int numStepsDriven = 0;
int lastvalStepCounterpin = 1;
int stepCounterDelay = 1200;
long lastStepCountTime = -stepCounterDelay;

//aktuator
int aktuatorpin1 = 19;
int aktuatorpin2 = 18;
bool aktuatorWantPress = false;

//bom
#define motorInterfaceType 1
int directionPin = 5;
int stepPin = 4;
int zeroButtonBom = 8;
int stepsToTrackList[] = {0,0,500,1000,200,100,300,100,100,100,300,100};
AccelStepper myStepper(motorInterfaceType, stepPin, directionPin);
long sumStepsBom = 0;
int selectedTrack = 1;
int lastTrack = 0;

//ned-dytter
int pushMotorLeftpin1 = 14;
int pushMotorLeftpin2 = 15;
int pushMotorRightpin1 = 16;
int pushMotorRightpin2 = 17;
int limitButtonUpRight = 12;
int limitButtonUpLeft = 9;
int limitButtonDownRight = 11;
int limitButtonDownLeft = 10;
int autoPushDelayFirst = 1500;
int autoPushDelayRest = 100;
int autoPushDelay = 0;
long lastAutoPush = 0;
int autoPushDriveDelay = 1000;

//batteri
#define batteryMeasurePin A14
float batteryMinVoltage = 3.5*205;
float batteryMaxVoltage = 4.6*205;
int batteryPercentage = 0;
int batteryPercentageTotal = 0;
int batterySampleSize = 20;
int batteryCurrentSample = 0;
int lowBatteryLimit = 20;

//Radio
const byte radioAdressList[][6] = {"00001","00002"};
#define CE_pin 7
#define CSN_pin 6
RF24 radio(CE_pin, CSN_pin);
int radioSendDelay = 100;
long radioLastSendTime = 0;
int radioRecieveTimeLimit = 600;
long radioLastRecieveTime = 0;
bool radioIsSendable = false;

struct payloadrec {
  byte ch1; //selectedTrack
  byte ch2; //motot fremgang
  byte ch3; //joystick push
  byte ch4; //b0
  byte ch5; //b1
  byte ch6; //b2
  byte ch7; //driveMode
  byte ch8; //antall trinn
};
payloadrec payloadrec;

struct payloadsend {
  byte ch1; //battery %
  byte ch2; //antall trinn kjørt
  byte ch3; //kjører 1/0
  byte ch4; //runde som kjøres 
};
payloadsend payloadsend;

//verdier fra fjernkontroll
int joystickYval = 126;
int joystickPushval = 0;
int button0val = 0;
int button1val = 0;
int button2val = 0;
int driveMode = 0; //0 = admin, 1 = auto
int numStepsToDrive = 0;
int autoGoMode = 0; //0 = ikke kjør, 1 = kjør frem-tilbake, 2 = kjør frem-tilbake-frem
int autoRound = 0; //hvilken runde den er på

//annet
int led0Pin = 31; //koblingstatus
int led1Pin = 29; //lavt batteri
bool failure = false;

void setup() {
	//generell oppstart
  Serial.begin(9600);
  pinMode(led0Pin, OUTPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(pushMotorLeftpin1, OUTPUT);
  pinMode(pushMotorLeftpin2, OUTPUT);
  pinMode(pushMotorRightpin1, OUTPUT);
  pinMode(pushMotorRightpin2, OUTPUT);
  pinMode(aktuatorpin1, OUTPUT);
  pinMode(aktuatorpin2, OUTPUT);
  pinMode(moveMotorpin1, OUTPUT);
  pinMode(moveMotorpin2, OUTPUT);
  pinMode(zeroButtonBom, INPUT_PULLUP);
  pinMode(limitButtonUpRight, INPUT_PULLUP);
  pinMode(limitButtonUpLeft, INPUT_PULLUP);
  pinMode(limitButtonDownRight, INPUT_PULLUP);
  pinMode(limitButtonDownLeft, INPUT_PULLUP);
  pinMode(stepCounterpin, INPUT_PULLUP);
  pinMode(batteryMeasurePin, INPUT);

  //fremgang oppstart
  lastvalStepCounterpin = digitalRead(stepCounterpin);
  moveStop();

  //ledlys oppstart
  FastLED.addLeds<WS2812B, RB_DATA_PIN, GRB>(rightBlinker, RB_NUM_LEDS);
  FastLED.addLeds<WS2812B, LB_DATA_PIN, GRB>(rightBlinker, LB_NUM_LEDS);
  FastLED.addLeds<WS2812B, B_DATA_PIN, GRB>(backLight, B_NUM_LEDS);
  FastLED.setBrightness(255);
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(150,150,150));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(150,150,150));
  fill_solid(backLight, B_NUM_LEDS, CRGB(255,0,0));
  FastLED.show();
  
  //aktuator oppstart
  aktuatorNoPress();
  delay(1000);
  aktuatorStop();

  //bom oppstart
  myStepper.setPinsInverted(true);
  myStepper.setMaxSpeed(500);
	myStepper.setAcceleration(400);
  myStepper.setSpeed(500);
  myStepper.move(80);
  while (myStepper.run()) {
  }
  myStepper.setSpeed(-400);
  while (digitalRead(zeroButtonBom) == HIGH) {
    myStepper.runSpeed();
  }
  myStepper.stop();
  myStepper.setCurrentPosition(0);
  myStepper.setSpeed(500);

  //ned-dytter oppstart
  while ((digitalRead(limitButtonUpRight) == HIGH) || (digitalRead(limitButtonUpLeft) == HIGH)) {
    if (digitalRead(limitButtonUpRight) == LOW) {
      pushStopRight();
    }
    else {
      pushUpRight();
    }
    if (digitalRead(limitButtonUpLeft) == LOW) {
      pushStopLeft();
    }
    else {
      pushUpLeft();
    }
  }
  pushStopLeft();
  pushStopRight();

  //Radio oppstart
  if (radio.begin()) {
    Serial.print("Radio boot OK");
  }
  else {
    Serial.print("No connection with NRF24");
		failure = true;
  }
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(radioAdressList[1]);
  radio.openReadingPipe(0, radioAdressList[0]);
  radioLastRecieveTime = millis();
}

void loop() {

  //måle gjennomsnitt batteri %
  batteryPercentageTotal += map(analogRead(batteryMeasurePin), batteryMinVoltage, batteryMaxVoltage, 0 ,100);
  batteryCurrentSample++;
  if (batteryCurrentSample >= batterySampleSize) {
    batteryPercentage = batteryPercentageTotal / batterySampleSize;
    batteryCurrentSample = 0;
    batteryPercentageTotal = 0;
  }
  digitalWrite(led1Pin, LOW);
  if (batteryPercentage <= lowBatteryLimit) {
    digitalWrite(led1Pin, HIGH);
  }

  //ledlys
  if (joystickPushval == 1) {
    wantLights = !wantLights;
    if (wantLights) {
      lightsOn();
    }
    else {
      lightsOff();
    }
    joystickPushval = 0;
  }

  //kjøre bom
  myStepper.run();

  //Motta data
  radioIsSendable = false;
  radio.setPayloadSize(sizeof(payloadrec));
  radio.startListening();
  //delay(5);
  if (radio.available()) {
		readRadio();
	}
	//kjøre bom
  myStepper.run();

  //Sende data
  if ((radioIsSendable) && (millis() >= radioLastSendTime + radioSendDelay)) {
    sendRadio();
  }

	//kjøre bom
  myStepper.run();

  //auto start
  if ((driveMode == 1) && (numStepsToDrive != 0) && (button1val == 1)) {
    autoGoMode = 1;
    autoRound = 0;
    autoPushDelay = autoPushDelayFirst;
    numStepsDriven = 0;
  }
	if ((driveMode == 1) && (numStepsToDrive != 0) && (button0val == 1)) {
    autoGoMode = 2;
    autoRound = 0;
    autoPushDelay = autoPushDelayFirst;
    numStepsDriven = 0;
  }

  //fremgang
  if ((digitalRead(stepCounterpin) == 0) && (lastvalStepCounterpin == 1) && (millis() >= lastStepCountTime + stepCounterDelay)) {
    numStepsDriven++;
    lastStepCountTime = millis();
  }
  lastvalStepCounterpin = digitalRead(stepCounterpin);
  
  if (driveMode == 0) { //admin
    if (joystickYval >= 200) {
      moveFWRD();
    }
    else if (joystickYval <= 55) {
      moveBWRD();
    }
    else {
      moveStop();
    }
    if (button0val == 1) {
      numStepsDriven = 0;
      button0val = 0;
    }
  }

	//kjøre stepper
  myStepper.run();

  //auto kjør
  if ((driveMode == 1) && (autoGoMode >= 1)) { //auto
    if (autoRound == 0) {
      moveFWRD();
      if (numStepsDriven >= numStepsToDrive) {
        autoRound++;
        numStepsDriven = 0;
        moveStop();
        aktuatorNoPress();
        delay(1000);
      }
    }
    if (autoRound == 1) {
      moveBWRD();
      if ((digitalRead(stepCounterpin) == 0) && (millis() >= lastAutoPush + autoPushDriveDelay)) {
        delay(autoPushDelay);
        autoPushDelay = autoPushDelayRest;
				//stop og press
        moveStop();
        pushDownLeft();
        pushDownRight();
        while((digitalRead(limitButtonDownRight) == HIGH) || (digitalRead(limitButtonDownLeft) == HIGH)) {
          if (digitalRead(limitButtonDownRight) == LOW) {
            pushStopRight();
          }
          if (digitalRead(limitButtonDownLeft) == LOW) {
            pushStopLeft();
          }
        }
        pushUpLeft();
        pushUpRight();
        while((digitalRead(limitButtonUpRight) == HIGH) || (digitalRead(limitButtonUpLeft) == HIGH)) {
          if (digitalRead(limitButtonUpRight) == LOW) {
            pushStopRight();
          }
          if (digitalRead(limitButtonUpLeft) == LOW) {
            pushStopLeft();
          }
        }
        lastAutoPush = millis();
        lastStepCountTime = millis();
      }
      if (numStepsDriven >= numStepsToDrive) {
        autoRound++;
        numStepsDriven = 0;
      }
    }
    if (autoRound == 2) {
      moveFWRD();
      if (numStepsDriven >= numStepsToDrive) {
        autoRound++;
        numStepsDriven = 0;
      }
      if(autoGoMode == 1) {
        autoRound++;
        numStepsDriven = 0;
      }
    }
    if (autoRound == 3) {
      moveStop();
      if (wantLights) {
        finishlights();
        lightsOn();
      }
      autoGoMode = 0;
    }
  }

  //Mistet signal
	digitalWrite(led0Pin, 0);
  if ((millis() >= radioRecieveTimeLimit + radioLastRecieveTime) || (failure)) {
    safeValues();
    Serial.println("No connection");
    digitalWrite(led0Pin, 1);
  }

  //flytte bom
  if (selectedTrack != lastTrack) {
    sumStepsBom = 0;
    for (int i = 0; i <= selectedTrack; i++) {
      sumStepsBom += stepsToTrackList[i];
    }
    myStepper.moveTo(sumStepsBom);
    lastTrack = selectedTrack;
  }
	//kjøre bom
  myStepper.run();

  //ned-dytter manuell
  if (driveMode == 0) { //admin
    if(button1val == 1) {
      while((digitalRead(limitButtonDownRight) == HIGH) || (digitalRead(limitButtonDownLeft) == HIGH)) {
        if (digitalRead(limitButtonDownRight) == LOW) {
          pushStopRight();
        }
        else {
          pushDownRight();
        }
        if (digitalRead(limitButtonDownLeft) == LOW) {
          pushStopLeft();
        }
        else {
          pushDownLeft();
        }
      }
      pushStopRight();
      pushStopLeft();
      while((digitalRead(limitButtonUpRight) == HIGH) || (digitalRead(limitButtonUpLeft) == HIGH)) {
        if (digitalRead(limitButtonUpRight) == LOW) {
          pushStopRight();
        }
        else {
          pushUpRight();
        }
        if (digitalRead(limitButtonUpLeft) == LOW) {
          pushStopLeft();
        }
        else {
          pushUpLeft();
        }
      }
      button1val = 0;
    }
  }
  //aktuator
  if (button2val == 1) {
    aktuatorWantPress = !aktuatorWantPress;
    if (aktuatorWantPress) {
      aktuatorPress();
      delay(1000);
    }
    if (!aktuatorWantPress) {
      aktuatorNoPress();
      delay(1000);
      aktuatorStop();
    }
    button2val = 0;
  }
	//kjøre stepper
  myStepper.run();
}

void sendRadio() {
	radioLastSendTime = millis();
	radio.stopListening();
	radio.setPayloadSize(sizeof(payloadsend));
	//delay(5);

	payloadsend.ch1 = batteryPercentage;
	payloadsend.ch2 = numStepsDriven;
	payloadsend.ch3 = autoGoMode;
	payloadsend.ch4 = autoRound;
	radio.write(&payloadsend, sizeof(payloadsend));

	Serial.print(payloadsend.ch1);
	Serial.print("   ");
	Serial.print(payloadsend.ch2);
	Serial.print("   ");
	Serial.print(payloadsend.ch3);
	Serial.print("   ");
	Serial.println(payloadsend.ch4);
}

void readRadio() {
	radioIsSendable = true;
	radioLastRecieveTime = millis();
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

	selectedTrack = payloadrec.ch1;
	joystickYval = payloadrec.ch2;
	joystickPushval = payloadrec.ch3;
	button0val = payloadrec.ch4;
	button1val = payloadrec.ch5;
	button2val = payloadrec.ch6;
	driveMode = payloadrec.ch7;
	numStepsToDrive = payloadrec.ch8;
}

//Verdier ved ingen Connection
void safeValues() {
	
}

void moveFWRD() {
	digitalWrite(moveMotorpin1, HIGH);
  digitalWrite(moveMotorpin2, LOW);
}

void moveBWRD() {
	digitalWrite(moveMotorpin1, LOW);
  digitalWrite(moveMotorpin2, HIGH);
}

void moveStop() {
  digitalWrite(moveMotorpin1, LOW);
  digitalWrite(moveMotorpin2, LOW);
}

void pushDownLeft() {
	digitalWrite(pushMotorLeftpin1, HIGH);
  digitalWrite(pushMotorLeftpin2, LOW);
}

void pushUpLeft() {
  digitalWrite(pushMotorLeftpin1, LOW);
  digitalWrite(pushMotorLeftpin2, HIGH);
}

void pushStopLeft() {
  digitalWrite(pushMotorLeftpin1, LOW);
  digitalWrite(pushMotorLeftpin2, LOW);
}

void pushDownRight() {
	digitalWrite(pushMotorRightpin1, HIGH);
  digitalWrite(pushMotorRightpin2, LOW);
}

void pushUpRight() {
  digitalWrite(pushMotorRightpin1, LOW);
  digitalWrite(pushMotorRightpin2, HIGH);
}

void pushStopRight() {
  digitalWrite(pushMotorRightpin1, LOW);
  digitalWrite(pushMotorRightpin2, LOW);
}

void aktuatorPress() {
  digitalWrite(aktuatorpin1, HIGH);
  digitalWrite(aktuatorpin2, LOW);
}

void aktuatorNoPress() {
  digitalWrite(aktuatorpin1, LOW);
  digitalWrite(aktuatorpin2, HIGH);
}

void aktuatorStop() {
  digitalWrite(aktuatorpin1, LOW);
  digitalWrite(aktuatorpin2, LOW);
}

void finishlights() {
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(0,255,0));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(0,255,0));
  fill_solid(backLight, B_NUM_LEDS, CRGB(0,255,0));
  FastLED.show();
  delay(300);
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(backLight, B_NUM_LEDS, CRGB(0,0,0));
  FastLED.show();
  delay(300);
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(0,255,0));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(0,255,0));
  fill_solid(backLight, B_NUM_LEDS, CRGB(0,255,0));
  FastLED.show();
  delay(300);
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(backLight, B_NUM_LEDS, CRGB(0,0,0));
  FastLED.show();
  delay(300);
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(0,255,0));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(0,255,0));
  fill_solid(backLight, B_NUM_LEDS, CRGB(0,255,0));
  FastLED.show();
  delay(300);
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(backLight, B_NUM_LEDS, CRGB(0,0,0));
  FastLED.show();
  delay(300);
}

void lightsOn() {
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(255,255,255));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(255,255,255));
  fill_solid(backLight, B_NUM_LEDS, CRGB(255,0,0));
  FastLED.show();
}

void lightsOff() {
  fill_solid(rightBlinker, RB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(leftBlinker, LB_NUM_LEDS, CRGB(0,0,0));
  fill_solid(backLight, B_NUM_LEDS, CRGB(0,0,0));
  FastLED.show();
}
