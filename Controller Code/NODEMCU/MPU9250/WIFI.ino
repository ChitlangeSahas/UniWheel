#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

#define APP_DEBUG true

/* Wifi server credentials */
const char *ssid = "Unicycle";    // The name of the Wi-Fi network that will be created
const char *password = "123456789";   // The password required to connect to it, leave blank for an open network

ESP8266WebServer server(80);

/* JSON */
StaticJsonDocument<200> doc;

/* OBTAIN VESC VALUES TO SEND THEM OVER */
VESCValues vescValues;

void setup_wifi() 
{
  delay(1000);
  Serial.begin(115200);                   // begin the serial for debugging
  Serial.println('\n');
  WiFi.softAP(ssid, password);             // Start the access point
  Serial.print("Access Point \"");
  Serial.print(ssid);
  Serial.println("\" started");

  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());         // Send the IP address of the ESP8266 to the computer

  server.on("/", handleRoot);
  server.on("/vesc_values/", vesc_values);
  server.begin();

}

void handleRoot() {
  esp8266VESC.getVESCValues(vescValues); // get the values.
  
  doc["rpm"] = vescValues.rpm;
  doc["inputVoltage"] = vescValues.inputVoltage;
  doc["avgInputCurrent"] = vescValues.avgInputCurrent;
  doc["dutyCycleNow"] = vescValues.dutyCycleNow;
  doc["temperaturePCB"] = vescValues.temperaturePCB;

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

void vesc_values() {
  server.send(200, "text/html", "<h1>Speed 100; Voltage: 22.2V </h1>");
}

void loop_wifi() 
{
  server.handleClient();
}
