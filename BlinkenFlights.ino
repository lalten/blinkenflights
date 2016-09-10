
#include <adafruit_feather.h>
#include <Adafruit_TLC59711.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include <Wire.h>
#include <SPI.h>

#include <map>
#include <vector>

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

int max_gyro = -1;
int gx, gy, gz;
char msg[100];
float dps_per_lsb_gyro = -1;
float mg_per_lsb_accel = -1;
float ax, ay, az;
float a_norm;

int gyro_z_offset = 8;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  mg_per_lsb_accel = LSM9DS0_ACCEL_MG_LSB_2G/1000.0;
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
  setupSensor();
  Wire.begin();
  tlc.begin();
  pinMode(BOARD_LED_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("BLINKENFLIGHTS!!!11!");
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

int blue[] =  { 0, 0, 0x1000 };
int green[] =   { 0x1000, 0x1000, 0 };
int red[] =   { 0x1000, 0, 0 };
int cyan[] =  { 0, 0x1000, 0x1000 };
int magenta[] = { 0x1000, 0, 0x1000 };
int yellow[] =  { 0x1000, 0x1000, 0 };

void loop() {
  lsm.read();
  ax = lsm.accelData.x*mg_per_lsb_accel;
  ay = lsm.accelData.y*mg_per_lsb_accel;
  az = lsm.accelData.z*mg_per_lsb_accel;

  a_norm = sqrt(ax*ax+ay*ay+az*az);

  gx = (int)lsm.gyroData.x*dps_per_lsb_gyro;
  gy = (int)lsm.gyroData.y*dps_per_lsb_gyro;
  gz = (int)lsm.gyroData.z*dps_per_lsb_gyro + gyro_z_offset; // in deg/s


  char mystring[] = "TECHFEST";

//  std::vector<int*> colors = {blue, green, red, cyan, magenta, yellow};

//  uint32_t pixel_duration_us = 1200; // TODO: from gyro
//  uint32_t rotation_duration_us = 240000;

  Serial.println(gz,DEC);

  if(gz > 200)
  {
	  uint32_t rotation_duration_us = 1e6 * 360 / gz;
	  uint32_t pixel_duration_us = 1200; // TODO: from gyro

//	  uint32_t pixel_duration_us = 1200000; // TODO: from gyro
//    uint32_t rotation_duration_us = 240000000;

	  uint32_t t_start = micros();

	  for (int i = 0; i < strlen(mystring); i++) {

		int* mycolor = blue;

		for (char col = 0; col < 5; col++) {
		  setChar(mystring[i], col, mycolor);
		  delayMicroseconds(pixel_duration_us);
		}
		clearChar();
		delayMicroseconds(2*pixel_duration_us); // inter-char space
		Serial.print(mystring[i]);
	  }

	  uint32_t t_end = micros();

	  delayMicroseconds(rotation_duration_us - (t_end - t_start));
  }
}
