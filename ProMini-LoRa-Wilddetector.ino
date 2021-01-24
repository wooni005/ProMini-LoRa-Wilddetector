/*
	LoRa Simple Gateway/Node Exemple

	This code uses InvertIQ function to create a simple Gateway/Node logic.

	Gateway - Sends messages with enableInvertIQ()
					- Receives messages with disableInvertIQ()

	Node		- Sends messages with disableInvertIQ()
					- Receives messages with enableInvertIQ()

	With this arrangement a Gateway never receive messages from another Gateway
	and a Node never receive message from another Node.
	Only Gateway to Node and vice versa.

	This code receives messages and sends a message every second.

	InvertIQ function basically invert the LoRa I and Q signals.

	See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
	for more on InvertIQ register 0x33.

	created 05 August 2018
	by Luiz H. Cassettari
*/

#include <SPI.h>							 // include libraries
#include <LoRa.h>
#include "LowPower.h"

#define DEBUG
#define ACCEPT_REMOTE_COMMANDS

#define SERIAL_BAUD	 				57600
#define NODE_ID			 			3			// NodeId of this LoRa Node
#define MAX_PACKET_SIZE				10

#define MSG_ID_NODE_STARTUP		1 		 	// Node startup notification
#define MSG_ID_STILL_ALIVE			2  		// Node still alive
#define MSG_ID_CMND_REQUEST		3  		// Node wakeup/cmnd request
#define MSG_ID_PIR_MOVEMENT		4  		// PIR detected movement

#define SEND_CMND_REQUEST_MSG_EVERY	228   // 0.5 hour
#define SEND_STILL_ALIVE_MSG_EVERY	1824  // 4 hours

//#define SEND_MSG_EVERY	22		// Watchdog is a timerTick on a avg 8,0 sec timebase
															// SEND_MSG_EVERY=8	-> +- 1min
															// SEND_MSG_EVERY=14 -> +- 2min
															// SEND_MSG_EVERY=23 -> +- 3min
															// SEND_MSG_EVERY=30 -> +- 4min
															// SEND_MSG_EVERY=38 -> +- 5min
volatile word sendMsgTimer = SEND_CMND_REQUEST_MSG_EVERY - 1;
volatile bool movementDetected = false;

//Message max 30 bytes
struct Payload {
	byte nodeId;
	byte msg [MAX_PACKET_SIZE - 1];
} txPayload;

const long loRaFrequency = 866E6;			// LoRa loRaFrequency

const int loRaCsPin = 15;						// LoRa radio chip select
const int loRaResetPin = 14;			 		// LoRa radio reset
const int loRaIrqPin = 2;						// change for your board; must be a hardware interrupt pin

const byte pirIrqPin = 3;
const byte pirEnablePin = 4;
volatile byte state = LOW;
volatile boolean pirEnable = true;

void movementDetectedInt() {
	movementDetected = true;
#ifdef DEBUG
	Serial.println("Movement detected");
#endif
}

void LoRa_rxMode(){
	LoRa.enableInvertIQ();								// active invert I and Q signals
	LoRa.receive();										// set receive mode
}

void LoRa_txMode(){
	LoRa.idle();											// set standby mode
	LoRa.disableInvertIQ();								// normal mode
}

void LoRa_sendMessage(Payload payload, byte payloadLen) {
	LoRa_txMode();											// set tx mode
	LoRa.beginPacket();								 	// start packet
	LoRa.write((byte*) &payload, payloadLen); 	// add payload
	LoRa.endPacket(true);								// finish packet and send it
}

void onReceive(int packetSize) {
	byte rxPayload [MAX_PACKET_SIZE];

	byte i = 0, rxByte;

	while (LoRa.available()) {
		rxByte = (byte)LoRa.read();
		if (i < MAX_PACKET_SIZE) {
			rxPayload[i] = rxByte;
			i++;
		}
	}

	// Only accept messages with our NodeId
	if (rxPayload[0] == NODE_ID) {
#ifdef DEBUG
		Serial.print("Rx packet OK "); // Start received message
		for (char i = 0; i < packetSize; i++) {
				Serial.print(rxPayload[i], DEC);
				Serial.print(' ');
		}
#endif
		if (rxPayload[1] == MSG_ID_CMND_REQUEST) {
			if (rxPayload[2] == 0) {
#ifdef DEBUG
				Serial.println("WildDetector=0");
#endif
				pirEnable = false;
			} else {
#ifdef DEBUG
				Serial.println("WildDetector=1");
#endif
				pirEnable = true;
			}
			digitalWrite(pirEnablePin, pirEnable);
		}
#ifdef DEBUG
		delay(100); // Time to output the serial data when debugging
		Serial.println();
#endif
	}
}

void onTxDone() {
	// Serial.println("TxDone");
	LoRa_rxMode();
}

void sendMovementDetectionMsg() {
	txPayload.nodeId = NODE_ID;
	txPayload.msg[0] = MSG_ID_PIR_MOVEMENT;
	LoRa_sendMessage(txPayload, 2); // send a message
}

void setup() {
#ifdef DEBUG
	Serial.begin(SERIAL_BAUD);									 // initialize serial
	while (!Serial);

	Serial.println();
	Serial.print("[LORA-NODE.");
	Serial.print(NODE_ID);
	Serial.println("]");
#endif

	LoRa.setPins(loRaCsPin, loRaResetPin, loRaIrqPin);

	if (!LoRa.begin(loRaFrequency)) {
#ifdef DEBUG
		Serial.println("LoRa init failed. Check your connections.");
#endif
		while (true);											 // if failed, do nothing
	}

	//LoRa.setTxPower(20);
	LoRa.enableCrc();
	LoRa.onReceive(onReceive);
	LoRa.onTxDone(onTxDone);
	LoRa_rxMode();

	// Send Node startup msg
	txPayload.nodeId = NODE_ID;
	txPayload.msg[0] = MSG_ID_NODE_STARTUP;
	LoRa_sendMessage(txPayload, 2); // send a message
	delay(40); // [ms] Give RFM95W time to send the message

	// PIR power pin
	pinMode(pirEnablePin, OUTPUT);
	digitalWrite(pirEnablePin, pirEnable);
	// PIR movement sensor external interrupt
	pinMode(pirIrqPin, INPUT_PULLUP);
	delay(100); // Short delay to suppress a false PIR movement detection
}

void loop() {
	attachInterrupt(digitalPinToInterrupt(pirIrqPin), movementDetectedInt, RISING);

	// Enter power down state with ADC and BOD module disabled. Wake up when wake up pin is low
	//Serial.println("Sleep for 8s....");
	//delay(100);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

	// Disable external pin interrupt on wake up pin
	detachInterrupt(digitalPinToInterrupt(pirIrqPin));

	// Waked up! From timer or movement (external interrupt)
	if (movementDetected) {
		movementDetected = false;
		// Waked up from external interrupt
		// delay(100); //Litle time to startup serial channel
		// Serial.println("Beweging gezien!");
		sendMovementDetectionMsg();
		delay(40); // [ms] Give RFM95W time to send the message
		LoRa.sleep(); // Put RFM95W in sleep mode
#ifndef ACCEPT_REMOTE_COMMANDS
		sendMsgTimer = 0; // No need to send still alive message
#endif
	} else {
		// Waked up by periodic wakeup timer (8s)
		sendMsgTimer++;

#ifdef ACCEPT_REMOTE_COMMANDS
		if (sendMsgTimer >= SEND_CMND_REQUEST_MSG_EVERY) {
			sendMsgTimer = 0;

#ifdef DEBUG
			Serial.println("Send Command Request message");
#endif
			txPayload.nodeId = NODE_ID;
			txPayload.msg[0] = MSG_ID_CMND_REQUEST;

			LoRa_sendMessage(txPayload, 2); // send a message
			delay(80); 	// [ms] Wait a while for the server response
			LoRa.sleep(); 	// Put RFM95W in sleep mode
		}
#endif
#ifndef ACCEPT_REMOTE_COMMANDS
		if (sendMsgTimer >= SEND_STILL_ALIVE_MSG_EVERY) {
			sendMsgTimer = 0;

#ifdef DEBUG
			Serial.println("Send Still Alive message");
#endif
			txPayload.nodeId = NODE_ID;
			txPayload.msg[0] = MSG_ID_STILL_ALIVE;

			LoRa_sendMessage(txPayload, 2); // send a message
			delay(40); 		// [ms] Give RFM95W time to send the message
			LoRa.sleep(); 	// Put RFM95W in sleep mode
		}
#endif
	}
}
