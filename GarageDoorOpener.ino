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

#define DOOR_OPEN_SENSOR D3
#define DOOR_CLOSED_SENSOR D2
#define DOOR_RELAY D1

#define DOOR_MOVE_DURATION 30000  //msec it takes for door to open/close

int current_state = DOOR_LOST; 

const char *ssid = WIFI_SSID;
const char *password = WIFI_SECRET;

Ticker statuschecker;

ESP8266WebServer server ( 80 );

const int led = 13;

void handleRoot() {
	digitalWrite ( led, 1 );
  Serial.println(door_state_text[current_state]);
 server.send (200, "text/html", index_html);
	digitalWrite ( led, 0 );
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
  
  //TODO: add code to check door position after specified time
}

int getDoorState(){
  int ds_new = DOOR_LOST;
  int s_open = digitalRead(DOOR_OPEN_SENSOR);
  int s_closed = digitalRead(DOOR_CLOSED_SENSOR);  

  if(s_closed == HIGH && s_open == HIGH){
    //should probably add a log entry or something like that
    logEvent("both sensors are high");
    current_state = DOOR_LOST;
    return current_state;
  }

  if(s_open == HIGH){
    current_state = DOOR_OPEN;
  }

  if(s_closed == HIGH){
    current_state = DOOR_CLOSED;
  }


  if(s_closed == LOW && s_open == LOW){
    //should probably add a log entry or something like that
    logEvent("both sensors are low");
    current_state = DOOR_LOST;
    return current_state;
  }

  return current_state;
}

void getDoorStateV(){
  getDoorState();
}

void logEvent(String event){
  //do nothing yet
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


