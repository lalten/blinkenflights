#include <adafruit_feather.h>
#include <Adafruit_TLC59711.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#include <SPI.h>

#include <map>
#include <vector>

#include "net.h"
#include "font.h"
#include "text.h"
#include "color.h"

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
Adafruit_TLC59711 tlc = Adafruit_TLC59711(2); // two daisy chained boards
Adafruit_NeoPixel strip = Adafruit_NeoPixel(18, PB4,  NEO_RGBW);

double gz = 0;
int gyro_z_offset = 8;

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
	strip.begin();
	strip.show(); // Initialize all pixels to 'off'

	digitalWrite(BOARD_LED_PIN, HIGH);
}

uint32_t t_end = 0;
uint32_t t_new_text = 0;


//uint16_t blue[] = { 0, 0, 0x1000 };
//uint16_t green[] = { 0x1000, 0x1000, 0 };
//uint16_t red[] = { 0x1000, 0, 0 };
//uint16_t cyan[] = { 0, 0x1000, 0x1000 };
//uint16_t magenta[] = { 0x1000, 0, 0x1000 };
//uint16_t yellow[] = { 0x1000, 0x1000, 0 };

uint16_t blue[] = { 0, 0, 0xFFFF };
uint16_t green[] = { 0xFFFF, 0xFFFF, 0 };
uint16_t red[] = { 0xFFFF, 0, 0 };
uint16_t cyan[] = { 0, 0xFFFF, 0xFFFF };
uint16_t magenta[] = { 0xFFFF, 0, 0xFFFF };
uint16_t yellow[] = { 0xFFFF, 0xFFFF, 0 };


void setChar(char c, char col, uint16_t *rgb, int8_t dir = 1) {
	if(dir<0) col = 4-col;
	for (int row = 0; row < 5; row++) {
		uint8_t led_nr = 5 - row;
		if (text[char_to_index(c)][5 * row + col]) {
			tlc.setLED(led_nr, rgb[0], rgb[1], rgb[2]);
		} else {
			tlc.setLED(led_nr, 0, 0, 0);
		}
		tlc.setLED(0, 0, 0xFFFF, 0); // red underline
	}
	tlc.write();
}

void clearChar() {
	for (int row = 1; row < 6; row++) {
		tlc.setLED(row, 0, 0, 0);
	}
	tlc.write();
}

uint8_t ledstate = 0;

uint32_t printText(double gz) {
	const uint8_t NUM_SIDES = 2;
	const uint16_t ROTATION_THRESH_TEXT = 500;

	double gz_abs = fabs(gz);
	char mystring[] = "TECHFEST";

	double rotation_duration_us = 240000 / NUM_SIDES;
	double pixel_duration_us = 800; // TODO: from gyro?

	if(gz_abs <= ROTATION_THRESH_TEXT)
	{
		// clear
		for (int row = 0; row < 6; row++) {
			tlc.setLED(row, 0, 0, 0);
		}
		tlc.write();

		uint16_t r = 0, g = 0, b = 0;
		if(ledstate < 6)
		{
			r = 0xFFFF;
		} else if(ledstate < 12) {
			g = 0xFFFF;
		} else {
			b = 0xFFFF;
		}

		tlc.setLED(ledstate % 6, r, g, b);
		tlc.write();

		ledstate = ++ledstate % 18;

		return micros() + 100*1000;
	} else {
		rotation_duration_us = (1e6 * 360.0 / gz_abs) / NUM_SIDES;
		pixel_duration_us = 800; // TODO: from gyro?
	}
	for (unsigned int i = 0; i < strlen(mystring); i++) {

		uint16_t* mycolor = blue;

		for (char col = 0; col < 5; col++) {
			setChar(mystring[i], col, mycolor, gz<0?1:-1);
			delayMicroseconds(pixel_duration_us);
		}
		clearChar();
		delayMicroseconds(2 * pixel_duration_us); // inter-char space
	}

	// "Wait" for rotation to finish
	uint32_t t_end_last = t_end;
	t_end = micros();
	uint32_t t_new_text = micros() + (rotation_duration_us - (t_end - t_end_last));

	return t_new_text;
}

uint32_t t_last_neopixelset = 0;
float neo_r=0, neo_b=0, neo_g=255, neo_h=0, neo_s=1, neo_v=1;

float gz_last = 0;

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

	lsm.readGyro();
	float gz_new = lsm.gyroData.z * LSM9DS0_GYRO_DPS_DIGIT_2000DPS + gyro_z_offset; // in deg/s
	gz = 0.99 * gz_last + 0.01 * gz_new; // lowpass
	gz_last = gz_new;

	if(micros() >= t_new_text)
	{
		t_new_text = printText(gz);
	}

	if(micros() > t_last_neopixelset + 100000)
	{
		t_last_neopixelset = micros();
		for (int i = 0; i < 18; i++)
		{
			HSVtoRGB(&neo_r, &neo_g, &neo_b, neo_h++, neo_s, neo_v);
			strip.setPixelColor(i, int(neo_g * 255), int(neo_r*255), int(neo_b*255), 0);
		}
		strip.show();
	}

	sprintf(msg, "Gyro: %lf deg/s", fabs(gz));
	send_msg_via_udp();



}
