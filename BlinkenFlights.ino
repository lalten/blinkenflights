#include <adafruit_feather.h>
#include <Adafruit_TLC59711.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include <Wire.h>
#include <SPI.h>

#include <map>
#include <vector>

#include "net.h"
#include "font.h"


Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
Adafruit_TLC59711 tlc = Adafruit_TLC59711(2); // two daisy chained boards

int32_t gz = 0;
int gyro_z_offset = 0; //8;

void setup() {
	pinMode(BOARD_LED_PIN, OUTPUT);

	Serial.begin(9600);
	Serial.println("BLINKENFLIGHTS!!!11!");

	while ( !connectAP() )
	{
		digitalWrite(BOARD_LED_PIN, HIGH);
		delay(50); // delay between each attempt
		digitalWrite(BOARD_LED_PIN, LOW);
		delay(50);
	}
	udp.begin(local_port);

	Wire.begin();
	lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
	tlc.begin();

	digitalWrite(BOARD_LED_PIN, HIGH);
}

void setChar(char c, char col, uint16_t *rgb) {
	for (int row = 0; row < 5; row++) {
		uint8_t led_nr = 5 - row;
		if (text[char_to_index(c)][5 * row + col]) {
			tlc.setLED(led_nr, rgb[0], rgb[1], rgb[2]);
		} else {
			tlc.setLED(led_nr, 0, 0, 0);
		}
	}
	tlc.write();
}

void clearChar() {
	for (int row = 0; row < 6; row++) {
		tlc.setLED(row, 0, 0, 0);
	}
	tlc.write();
}

uint16_t blue[] = { 0, 0, 0x1000 };
uint16_t green[] = { 0x1000, 0x1000, 0 };
uint16_t red[] = { 0x1000, 0, 0 };
uint16_t cyan[] = { 0, 0x1000, 0x1000 };
uint16_t magenta[] = { 0x1000, 0, 0x1000 };
uint16_t yellow[] = { 0x1000, 0x1000, 0 };

uint32_t t_end = 0;

void loop() {

	// Send heartbeat when no other message was sent
	heartbeat();

//	int packetSize = udp.parsePacket();
//
//	if (packetSize) {
//		sprintf(msg, "Received UDP: %i Byte", packetSize);
//		send_msg_via_udp(); //// sends global msg-variable
//		udp.read(packetBuffer, sizeof(packetBuffer)); // char[]
//		if (packetBuffer[0] == 'L') {
//			sprintf(msg, "LAUFLICHT: %i, %s", packetSize, packetBuffer);
//			send_msg_via_udp(); //// sends global msg-variable
//		}
//	}

	lsm.read();
	gz = (int32_t) lsm.gyroData.z * LSM9DS0_GYRO_DPS_DIGIT_2000DPS + gyro_z_offset; // in deg/s
	uint32_t gz_abs = fabs(gz);

	char mystring[] = "TECHFEST";

	double rotation_duration_us = 240000;
	double pixel_duration_us = 1200;
	if (gz_abs > 200) {
		rotation_duration_us = 1e6 * 360.0 / gz_abs;
		pixel_duration_us = 1200; // TODO: from gyro
	}

	// TODO: reverse string and characters on negative gz

	for (unsigned int i = 0; i < strlen(mystring); i++) {

		uint16_t* mycolor = blue;

		for (char col = 0; col < 5; col++) {
			setChar(mystring[i], col, mycolor);
			delayMicroseconds(pixel_duration_us);
		}
		clearChar();
		delayMicroseconds(2 * pixel_duration_us); // inter-char space
	}

	sprintf(msg, "Gyro: %i deg/s (abs: %i), CalcRotDuration: %i us", (int) gz, (int) gz_abs, (int) rotation_duration_us);
	send_msg_via_udp();

	// Wait for rotation to finish
	// TODO: wait for half/third rotation?
	uint32_t t_end_last = t_end;
	t_end = micros();
	delayMicroseconds(rotation_duration_us - (t_end - t_end_last));


}
