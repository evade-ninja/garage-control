/*
 * Copyright (c) 2015, Majenko Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <String.h>
#include "index.h"
#include "narrow.css.h"
#include "garage_status.h"
#include "GarageDoorOpener.h"
#include "wifi.h"
#include <WiFiUdp.h>
#include <TimeLib.h>

#define DOOR_OPEN_SENSOR D1
#define DOOR_CLOSED_SENSOR D5
#define DOOR_RELAY D2

#define DOOR_MOVE_DURATION 10  //sec it takes for door to open/close

int current_state = DOOR_LOST; 

const char *ssid = WIFI_SSID;
const char *password = WIFI_SECRET;

//Time service variables//
IPAddress timeServerIP;
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48;
unsigned int localPort = 2390;
const int timeZone = -6;

byte packetBuffer[ NTP_PACKET_SIZE];
WiFiUDP Udp;

unsigned long next_door_check = 0;
unsigned int door_move_time = DOOR_MOVE_DURATION;

//internal timer object to update status
Ticker statuschecker;
Ticker timeUpdater;

//webserver
ESP8266WebServer server ( 80 );

const int led = 13;

void handleRoot() {
	//digitalWrite ( led, 1 );
  logEvent(door_state_text[current_state]);
  server.send (200, "text/html", index_html);
	//digitalWrite ( led, 0 );
}

void handleNarrowCSS(){
  server.send(200, "text/css", narrow_css);
}

void handleStatus(){

  //build a JSON object and return it to the client

  DynamicJsonBuffer  jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
    root["door_open"] = levelToString(digitalRead(DOOR_OPEN_SENSOR));
    root["door_closed"] = levelToString(digitalRead(DOOR_CLOSED_SENSOR));
    root["current_state"] = getDoorStateChr();

  char buf[1000];
  root.printTo(buf, 1000);

  server.send(200, "application/json", buf);

}

void handleNotFound() {
	digitalWrite ( led, 1 );
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";

	for ( uint8_t i = 0; i < server.args(); i++ ) {
		message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
	}

	server.send ( 404, "text/plain", message );
	digitalWrite ( led, 0 );
}

void setup ( void ) {
	pinMode ( led, OUTPUT );
  pinMode(DOOR_RELAY, OUTPUT);
	digitalWrite ( led, 0 );
	Serial.begin ( 115200 );
	WiFi.begin ( ssid, password );
	Serial.println ( "" );

	// Wait for connection
	while ( WiFi.status() != WL_CONNECTED ) {
		delay ( 500 );
		Serial.print ( "." );
	}

	Serial.println ( "" );
	Serial.print ( "Connected to " );
	Serial.println ( ssid );
	Serial.print ( "IP address: " );
	Serial.println ( WiFi.localIP() );

	if ( MDNS.begin ( "gcc" ) ) {
		Serial.println ( "MDNS responder started" );
	}

//do an initial read of the sensors
  getDoorState();

  statuschecker.attach(5.1, getDoorStateV);

//Setup the Web Server

	server.on ( "/", handleRoot );
	//server.on ( "/test.svg", drawGraph );
  server.on ( "/narrow.css", handleNarrowCSS);
	server.on ( "/inline", []() {
		server.send ( 200, "text/plain", "this works as well" );
	} );
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  server.on("/update", handleUpdate);
	server.onNotFound ( handleNotFound );
	server.begin();
	Serial.println ( "HTTP server started" );

  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(28800);

  if(timeStatus() == timeNotSet){
    Serial.println("time not set - init updater on 30 sec interval");
    timeUpdater.attach(30, syncTime);
  }

  Serial.print("Time updated ");
  Serial.println(now());
  
}

void loop ( void ) {
	server.handleClient();
}

void handleToggle(){
  toggleDoor();

    DynamicJsonBuffer  jsonBuffer;

    JsonObject& root = jsonBuffer.createObject();
    root["success"] = "true";

  char buf[100];
  root.printTo(buf, 100);

  server.send(200, "application/json", buf);

  
  //server.send(200, "text/plain", "toggled");
}

void handleUpdate(){
  getDoorState();
  server.send(200, "text/plain", "updated");
}

void toggleDoor(){
  digitalWrite(DOOR_RELAY, HIGH);
  delay(1000);
  digitalWrite(DOOR_RELAY, LOW);

  switch(current_state){
    case DOOR_OPEN:
      current_state = DOOR_CLOSING;
      logEvent("Toggle: Open -> Closing");
      break;
    case DOOR_CLOSED:
      current_state = DOOR_OPENING;
      logEvent("Toggle: Closed -> Opening");
      break;
    case DOOR_LOST:
      current_state = DOOR_LOST_MOVING;
      logEvent("Toggle: Lost -> Moving");
      break;
    case DOOR_OPENING:
      current_state = DOOR_LOST;
      logEvent("Toggle: Opening -> Lost");
      break;
    case DOOR_CLOSING:
      current_state = DOOR_LOST;
      logEvent("Toggle: Closing -> Lost");
      break;
    case DOOR_LOST_MOVING:
      current_state = DOOR_LOST;
      logEvent("Toggle: Moving -> Lost");
      break;
  }

  //need to check the door status in now + DOOR_MOVE_DURATION

  next_door_check = now() + door_move_time;
  
}

int getDoorState(){
  int ds_new = DOOR_LOST;
  int s_open = digitalRead(DOOR_OPEN_SENSOR);
  int s_closed = digitalRead(DOOR_CLOSED_SENSOR);  

  //both sensors HIGH is always an error
  
  if(s_closed == HIGH && s_open == HIGH){
    //should probably add a log entry or something like that
    logEvent("both sensors are high");
    current_state = DOOR_LOST;
    return current_state;    
  }

  //has the minimum time passed to be able to check the door status?
  if(now() < next_door_check){
    //not enough time has passed, so don't check the status
    return current_state;
  }

  if(s_open == HIGH){
    ds_new = DOOR_OPEN;
  }

  if(s_closed == HIGH){
    ds_new = DOOR_CLOSED;
  }

  if(s_closed == LOW && s_open == LOW){
    //should probably add a log entry or something like that
    logEvent("both sensors are low");
    ds_new = DOOR_LOST;
  }

  if(current_state == DOOR_CLOSING || current_state == DOOR_OPENING || current_state == DOOR_LOST_MOVING){
    String sStr = "Door is now ";
    sStr += door_status_text[ds_new];
    logEvent(sStr);
  }

  current_state = ds_new;
  return current_state;
}

void getDoorStateV(){
  getDoorState();
}

void logEvent(String event){
  Serial.println(event);
}

char* levelToString(int level){
  if(level == HIGH){
    return "true";
  }else{
    return "false";
  }
}

char const* getDoorStateChr(){
  return door_state_text[current_state];
}

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  logEvent("Transmit NTP Request");
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      logEvent("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  logEvent("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

unsigned long sendNTPpacket(IPAddress& address)
{
  logEvent("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void syncTime(){
  if(timeStatus() != timeSet){
    time_t newTime = getNtpTime();
    if(newTime > 0){
      setTime(newTime);
      Serial.print("time now set ");
      Serial.println(now());
    }
  }
}

