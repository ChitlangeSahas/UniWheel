#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

#define APP_DEBUG false

ESP8266WebServer server(80);

/* JSON */
StaticJsonDocument<100> doc;


void setup_wifi() 
{
  delay(1000);
  Serial.begin(115200);                   
  Serial.println('\n');
  WiFi.softAP("Unicycle", "123456789");             
  Serial.print(F("Access Point \""));
  Serial.print(F("Unicycle"));
  Serial.println(F("\" started"));
  Serial.print(F("IP address:\t"));
  Serial.println(WiFi.softAPIP());       

  server.on("/", handleRoot);
  server.begin();
}


void handleRoot() {

  vesc.getVescValues();

//    doc["rpm"] = (int)abs((vesc.data.rpm)/7);
    doc["rpm"] = (vesc.data.wattHours);
    doc["inputVoltage"] = vesc.data.inpVoltage;
    doc["avgInputCurrent"] = vesc.data.avgInputCurrent;
    doc["dutyCycleNow"] = vesc.data.dutyCycleNow;
    doc["temperaturePCB"] = vesc.data.tempMosfet;
  
  if(APP_DEBUG == true)
  {
    doc["rpm"] = random(0, 5000);
    doc["inputVoltage"] = random(0, 25.2);
    doc["avgInputCurrent"] = random(0, 50);
    doc["dutyCycleNow"] = random(0, 100);
    doc["temperaturePCB"] = random(0, 100);
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "text/html", response);
}

void loop_wifi() 
{
  server.handleClient();
  yield();
  delay(20);
  server.handleClient();
  yield();
  delay(20);
  server.handleClient();
  yield();
  delay(20);
}
