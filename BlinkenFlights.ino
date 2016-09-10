#include <adafruit_feather.h>
#include <Adafruit_TLC59711.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include <Wire.h>
#include <SPI.h>

#include <map>
#include <vector>

#include "font.h"

#define WLAN_SSID   "AndroidAP"
#define WLAN_PASS   "blinkblink"
/// TODO: get from hostname
IPAddress server_ip(192, 168, 43, 162);
const uint16_t port = 8888;
AdafruitUDP udp;
const uint16_t local_port = 88;
bool with_serial = true;
char packetBuffer[255];

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
Adafruit_TLC59711 tlc = Adafruit_TLC59711(2); // two daisy chained boards

int max_gyro = -1;
int gx = 0, gy = 0, gz = 0;
char msg[100];
float dps_per_lsb_gyro = -1;
float mg_per_lsb_accel = -1;
float ax, ay, az;
float a_norm;

int gyro_z_offset = 8;
long last_heartbeat = 0;

void setupSensor() {
	// 1.) Set the accelerometer range
	lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
	mg_per_lsb_accel = LSM9DS0_ACCEL_MG_LSB_2G / 1000.0;
	//lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
	//lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
	//lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
	//lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

	// 2.) Set the magnetometer sensitivity
	lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
	//lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
	//lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
	//lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

	// 3.) Setup the gyroscope
	// lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
	// dps_per_lsb_gyro = LSM9DS0_GYRO_DPS_DIGIT_245DPS;

	//lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
	// dps_per_lsb_gyro = LSM9DS0_GYRO_DPS_DIGIT_500DPS;

	lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
	dps_per_lsb_gyro = LSM9DS0_GYRO_DPS_DIGIT_2000DPS;
}

bool connectAP(void)
{
  // Attempt to connect to an AP
if (with_serial)
{
  Serial.print("Please wait while connecting to: '" WLAN_SSID "' ... ");
}
  if ( Feather.connect(WLAN_SSID, WLAN_PASS) )
  {
    if (with_serial) {Serial.println("Connected!");}
  }
  else
  {
    if (with_serial){Serial.printf("Failed! %s (%d)", Feather.errstr(), Feather.errno());}
    if (with_serial){Serial.println();}
  }
  if (with_serial){Serial.println();}

  return Feather.connected();
}

void send_msg_via_udp()
{
  udp.beginPacket(server_ip, port);
  udp.println(msg); // send with newline
  udp.endPacket();
  last_heartbeat = millis();
}



char char_to_index(char c) {
	if (c >= 'A' && c <= 'Z')
		return c - 'A';
	if (c == ':')
		return 27;
	if (c == ')')
		return 28;
	return 26;
}

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
	setupSensor();
	tlc.begin();

	digitalWrite(BOARD_LED_PIN, HIGH);
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

int blue[] = { 0, 0, 0x1000 };
int green[] = { 0x1000, 0x1000, 0 };
int red[] = { 0x1000, 0, 0 };
int cyan[] = { 0, 0x1000, 0x1000 };
int magenta[] = { 0x1000, 0, 0x1000 };
int yellow[] = { 0x1000, 0x1000, 0 };

void loop() {

	long now = millis();
	if (now - last_heartbeat > 1000) {
		sprintf(msg, "Heartbeat: %i", last_heartbeat);
		send_msg_via_udp(); //// sends global msg-variable
		last_heartbeat = now;

		digitalWrite(BOARD_LED_PIN, HIGH);
		delay(50); // delay between each attempt
		digitalWrite(BOARD_LED_PIN, LOW);
		delay(50);
	}

	int packetSize = udp.parsePacket();

	if (packetSize) {
		sprintf(msg, "Received UDP: %i Byte", packetSize);
		send_msg_via_udp(); //// sends global msg-variable
		udp.read(packetBuffer, sizeof(packetBuffer)); // char[]
		if (packetBuffer[0] == 'L') {
			sprintf(msg, "LAUFLICHT: %i, %s", packetSize, packetBuffer);
			send_msg_via_udp(); //// sends global msg-variable
		}

	}

	lsm.read();
	gz = (int) lsm.gyroData.z * dps_per_lsb_gyro + gyro_z_offset; // in deg/s

	char mystring[] = "TECHFEST";

	if (true)
	{
	sprintf(msg, "Gyro: %i", gz);
	send_msg_via_udp();
	if (with_serial){ Serial.println(msg); }
	}

	uint32_t rotation_duration_us =240000;
	uint32_t pixel_duration_us = 1200;
	if (gz > 200) {
		rotation_duration_us = 1e6 * 360 / gz;
		pixel_duration_us = 1200; // TODO: from gyro
	}

	uint32_t t_start = micros();

	for (int i = 0; i < strlen(mystring); i++) {

		int* mycolor = blue;

		for (char col = 0; col < 5; col++) {
			setChar(mystring[i], col, mycolor);
			delayMicroseconds(pixel_duration_us);
		}
		clearChar();
		delayMicroseconds(2 * pixel_duration_us); // inter-char space
		Serial.print(mystring[i]);
	}

	uint32_t t_end = micros();

	delayMicroseconds(rotation_duration_us - (t_end - t_start));

}
