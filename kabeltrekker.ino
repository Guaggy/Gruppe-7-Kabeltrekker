#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>

//fremgang
#define moveMotorpin1 A3
#define moveMotorpin2 A4
#define stepCounterpin A5
int numStepsDriven = 0;
int lastvalStepCounterpin = 1;
int stepCounterDelay = 1500;
long lastStepCountTime = -stepCounterDelay;

//bom
#define motorInterfaceType 1
int directionPin = 2;
int stepPin = 3;
int zeroButtonBom = 9;
int stepsToTrackList[] = {0,0,500,1000,200,100,300,100,100,100,300,100};
AccelStepper myStepper(motorInterfaceType, stepPin, directionPin);
long sumStepsBom = 0;
int selectedTrack = 1;
int lastTrack = 0;

//ned-dytter
#define pushMotorpin1 A1
#define pushMotorpin2 A2
#define limitButtonPush 5
int autoPushDelayFirst = 1400;
int autoPushDelayRest = 600;
int autoPushDelay = 0;

//batteri
#define batteryMeasurePin A0
float batteryMinVoltage = 3.5*205;
float batteryMaxVoltage = 4.6*205;
int batteryPercentage = 0;
int batteryPercentageTotal = 0;
int batterySampleSize = 20;
int batteryCurrentSample = 0;

//Radio
const byte radioAdressList[][6] = {"00001","00002"};
#define CE_pin 7
#define CSN_pin 8
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
int joystickPushVal = 0;
int button0val = 0;
int button1val = 0;
int button2val = 0;
int driveMode = 0; //0 = admin, 1 = auto
int numStepsToDrive = 0;
int autoGoMode = 0; //0 = ikke kjør, 1 = kjør frem-tilbake, 2 = kjør frem-tilbake-frem
int autoRound = 0; //hvilken runde den er på

//annet
int led0Pin = 4;
bool failure = false;

void setup() {
	//generell oppstart
  Serial.begin(9600);
  pinMode(led0Pin, OUTPUT);
  pinMode(pushMotorpin1, OUTPUT);
  pinMode(pushMotorpin2, OUTPUT);
  pinMode(moveMotorpin1, OUTPUT);
  pinMode(moveMotorpin2, OUTPUT);
  pinMode(zeroButtonBom, INPUT_PULLUP);
  pinMode(limitButtonPush, INPUT_PULLUP);
  pinMode(stepCounterpin, INPUT_PULLUP);
  pinMode(batteryMeasurePin, INPUT);

  //fremgang oppstart
  lastvalStepCounterpin = digitalRead(stepCounterpin);
  moveStop();

  //bom oppstart
  myStepper.setPinsInverted(true);
  myStepper.setMaxSpeed(500);
	myStepper.setAcceleration(400);
  myStepper.setSpeed(300);
  myStepper.move(80);
  while (myStepper.run()) {
  }
  myStepper.setSpeed(-100);
  while (digitalRead(zeroButtonBom) == HIGH) {
    myStepper.runSpeed();
  }
  myStepper.stop();
  myStepper.setCurrentPosition(0);
  myStepper.setSpeed(500);

  //ned-dytter oppstart
  while (digitalRead(limitButtonPush) == HIGH) {
    pushUp();
  }
  pushStop();

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
  if ((numStepsToDrive != 0) && (button1val == 1)) {
    autoGoMode = 1;
    autoRound = 0;
    autoPushDelay = autoPushDelayFirst;
    numStepsDriven = 0;
  }
	if ((numStepsToDrive != 0) && (button0val == 1)) {
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
    if (button2val == 1) {
      numStepsDriven = 0;
    }
  }

	//kjøre stepper
  myStepper.run();

  if ((driveMode == 1) && (autoGoMode >= 1)) { //auto
    if (autoRound == 0) {
      moveFWRD();
      if (numStepsDriven >= numStepsToDrive) {
        autoRound++;
        numStepsDriven = 0;
      }
    }
    if (autoRound == 1) {
      moveBWRD();
      if(digitalRead(stepCounterpin) == 0) {
        delay(autoPushDelay);
        autoPushDelay = autoPushDelayRest;
				//stop og press
        moveStop();
        pushDown();
        delay(400);
        while(digitalRead(limitButtonPush) == 1) {
        }
        pushUp();
        delay(500);
        while(digitalRead(limitButtonPush) == 1) {
        }
        pushStop();
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
      pushDown();
      delay(400);
      while(digitalRead(limitButtonPush) == 1) {
      }
      pushUp();
      delay(500);
      while(digitalRead(limitButtonPush) == 1) {
      }
      pushStop();
    }
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
	joystickPushVal = payloadrec.ch3;
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
	digitalWrite(moveMotorpin1, LOW);
  digitalWrite(moveMotorpin2, HIGH);
}

void moveBWRD() {
	digitalWrite(moveMotorpin1, HIGH);
  digitalWrite(moveMotorpin2, LOW);
}

void moveStop() {
  digitalWrite(moveMotorpin1, LOW);
  digitalWrite(moveMotorpin2, LOW);
}

void pushDown() {
	digitalWrite(pushMotorpin1, HIGH);
  digitalWrite(pushMotorpin2, LOW);
}

void pushUp() {
  digitalWrite(pushMotorpin1, LOW);
  digitalWrite(pushMotorpin2, HIGH);
}

void pushStop() {
  digitalWrite(pushMotorpin1, LOW);
  digitalWrite(pushMotorpin2, LOW);
}
