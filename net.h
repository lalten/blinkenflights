/*
 * net.h
 *
 *  Created on: Sep 10, 2016
 *      Author: laurenz
 */

#ifndef NET_H_
#define NET_H_

#define WLAN_SSID   "AndroidAP"
#define WLAN_PASS   "blinkblink"

/// TODO: get from hostname
IPAddress server_ip(192, 168, 43, 162);
const uint16_t port = 8888;
AdafruitUDP udp;
const uint16_t local_port = 88;
char packetBuffer[255];
char msg[100];
long last_heartbeat = 0;

bool connectAP(void)
{
  // Attempt to connect to an AP
  Feather.connect(WLAN_SSID, WLAN_PASS);
  return Feather.connected();
}

void send_msg_via_udp()
{
  udp.beginPacket(server_ip, port);
  udp.println(msg); // send with newline
  udp.endPacket();
  last_heartbeat = millis();
}


void heartbeat()
{
	long t_now = millis();
	if (t_now - last_heartbeat > 1000) {
		sprintf(msg, "Heartbeat: %i", last_heartbeat);
		send_msg_via_udp(); //// sends global msg-variable
		last_heartbeat = t_now;
	}
}


#endif /* NET_H_ */
