#include <Adafruit_TLC59711.h>

#include <map>
#include <vector>

char text[][5 * 5] = { { 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0,
		0, 1, 1, 0, 0, 0, 1 }, { 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1,
		0, 0, 0, 1, 1, 1, 1, 1, 0 }, { 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0,
		0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0 }, { 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0,
		0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0 }, { 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 }, { 0, 1, 1, 1, 1, 1,
		0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0 }, { 1, 0, 0, 0,
		1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1 }, { 0, 0,
		1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0 }, {
		0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0,
		0 }, { 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0,
		0, 1, 0 }, { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		1, 1, 1, 1, 1 }, { 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0,
		0, 1, 1, 0, 0, 0, 1 }, { 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1,
		0, 0, 1, 1, 1, 0, 0, 0, 1 }, { 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0,
		1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0 }, { 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1,
		1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 }, { 0, 1, 1, 1, 0, 1, 0, 0, 1, 0,
		1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1 }, { 1, 1, 1, 1, 0, 1, 0, 0,
		0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0 }, { 0, 1, 1, 1, 1, 1,
		0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0 }, { 1, 1, 1, 1,
		1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0 }, { 1, 0,
		0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0 }, {
		1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0,
		0 }, { 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0,
		0, 0, 1 }, { 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0,
		1, 0, 0, 0, 1 }, { 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1,
		0, 0, 0, 0, 1, 0, 0 }, { 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
		1, 0, 0, 0, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0 } };

char char_to_index(char c) {
	if (c >= 'A' && c <= 'Z')
		return c - 'A';
	if (c == ':')
		return 27;
	if (c == ')')
		return 28;
	return 26;
}
Adafruit_TLC59711 tlc = Adafruit_TLC59711(2); // two daisy chained boards

void setup() {
	tlc.begin();
	pinMode(BOARD_LED_PIN, OUTPUT);
	Serial.begin(9600);
}

void setChar(char c, char col, int *rgb) {
	for (int row = 0; row < 5; row++) {
		int led_nr = 5 - row;
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

int blue[] = 	{ 0, 0, 0x1000 };
int green[] = 	{ 0x1000, 0x1000, 0 };
int red[] = 	{ 0x1000, 0, 0 };
int cyan[] = 	{ 0, 0x1000, 0x1000 };
int magenta[] = { 0x1000, 0, 0x1000 };
int yellow[] = 	{ 0x1000, 0x1000, 0 };

void loop() {

	char mystring[] = "TECHFEST";

	std::vector<int*> colors = {blue, green, red, cyan, magenta, yellow};

//  uint32_t pixel_duration_us = 1200; // TODO: from gyro
//  uint32_t rotation_duration_us = 240000;

	uint32_t pixel_duration_us = 1200000; // TODO: from gyro
	uint32_t rotation_duration_us = 240000000;

	uint32_t delays_accumulated = 0;

	for (int i = 0; i < strlen(mystring); i++) {

		int* mycolor = blue;

		for (char col = 0; col < 5; col++) {
			setChar(mystring[i], col, mycolor);
			delayMicroseconds(pixel_duration_us);
			delays_accumulated += pixel_duration_us;
		}
		clearChar();
		delayMicroseconds(2*pixel_duration_us); // inter-char space
		delays_accumulated += pixel_duration_us;
		Serial.print(mystring[i]);
	}

	delayMicroseconds(rotation_duration_us - delays_accumulated);

}
