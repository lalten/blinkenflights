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

class BumpDetection {
   int c1_thres, c2_thres, dt1, dt2; 
   long c1_entry;
   long c1_exit;
   bool debug;
   
public:

    BumpDetection(int c1_threshold_, int c2_threshold_,int t1_, int t2_):
      c1_thres(c1_threshold_),
      c2_thres(c2_threshold_),
      dt1(t1_),
      dt2(t2_),
      c1_entry(0),
      c1_exit(0), 
      debug(true)
      {}

    bool add_value(int v)
    {
        int dg = abs(v-1);
        if (dg < c1_thres)
        {
          if (c1_entry == 0)
          {
            c1_entry = millis();
          }
          if (debug){
            sprintf(msg,"WITHIN C1: %i %i, dt: %i", dg, c1_thres, (millis()-c1_entry));
            send_msg_via_udp(); //// sends global msg-variable   
          }           
        }
        else 
        {
          if (c1_entry > 0)
          {
            long now = millis();
            if (now-c1_entry>dt1)
            {
              if (debug){
                sprintf(msg,"Exiting C1: %i, c1 entry: %i", dg, c1_entry);
                send_msg_via_udp(); //// sends global msg-variable
              }
              c1_exit = now;
            }
          }
          c1_entry = 0;
        }

        if (dg > c2_thres){
          long now = millis();
          if (c1_exit > 0 && (now-c1_exit) < dt2)
          {
            if (debug){
              sprintf(msg,"XXXXXXXXX Trigger: c1 %i, dt: %i, value: %i", c1_exit, dt2, v);
              send_msg_via_udp(); //// sends global msg-variable
            }
            c1_exit = 0;
            return true;
          }
        }
        return false;
    }
};


BumpDetection bd_gyro_xy(30, 600, 400, 200);
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
Adafruit_TLC59711 tlc = Adafruit_TLC59711(2); // two daisy chained boards
Adafruit_NeoPixel strip = Adafruit_NeoPixel(18, PB4,  NEO_RGBW);

double gz = 0;
int32_t gx = 0;
int32_t gy = 0;

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
bool color_wheel_active = true;

float gz_last = 0;
float ax, ay;
uint32_t color_wheel_time = 0;
uint32_t now = 0;

void loop() {

	now = millis();

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

	lsm.readAccel();
	lsm.readGyro();

	float gz_new = lsm.gyroData.z * LSM9DS0_GYRO_DPS_DIGIT_2000DPS + gyro_z_offset; // in deg/s
	gz = 0.99 * gz_last + 0.01 * gz_new; // lowpass
	gz_last = gz_new;

	ax = lsm.accelData.x*LSM9DS0_ACCEL_MG_LSB_2G;
	ay = lsm.accelData.y*LSM9DS0_ACCEL_MG_LSB_2G;
	
	gy = (int32_t) lsm.gyroData.y * LSM9DS0_GYRO_DPS_DIGIT_2000DPS; // in deg/s
	gz = (int32_t) lsm.gyroData.z * LSM9DS0_GYRO_DPS_DIGIT_2000DPS + gyro_z_offset; // in deg/s

  int gyro_xy = sqrt(gx*gx+gy*gy);
  bool triggered = bd_gyro_xy.add_value(gyro_xy);
  if (triggered)
  {
      sprintf(msg,"TRIGGERED");
      send_msg_via_udp(); //// sends global msg-variable
      color_wheel_active = true;
      color_wheel_time = 0;
  }

  if (color_wheel_active)
  {
    float angle = atan2(ay, ax);
    sprintf(msg, "Current angle: %f, speed: %i", 180.0*angle/M_PI, (int) gz);
    send_msg_via_udp();
//
//    if (abs(gz) < 10)
//    {
//      if (color_wheel_time == 0)
//      {
//        color_wheel_time = now;
//      }
//      if ( (now-color_wheel_time) > 500)
//      {
//        color_wheel_active = false;
//      }
//    }else
//    {
//      color_wheel_time = 0;
//    }
    
    // check end of color sequence:
    /// TODO: use angle as color
  }
  else 
  {
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
  }

	sprintf(msg, "Gyro: %lf deg/s, X/Y: %i", fabs(gz), gyro_xy);
	send_msg_via_udp();



}
