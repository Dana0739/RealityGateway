#include <WiFi.h>
#include <FS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define ANALOG_PIN_0 32
#define ANALOG_PIN_1 33
int analog_value0 = 0;
int analog_value1 = 0;
 
const char *ssid = "MyESP32AP";
const char *password = "testpassword";
 
AsyncWebServer server(80);
 
void setup(){
  Serial.begin(115200);
 
  WiFi.softAP(ssid, password);
 
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
 
  server.on("/hello", HTTP_GET, [](AsyncWebServerRequest *request){
    analog_value0 = analogRead(ANALOG_PIN_0);
    Serial.print(analog_value0);
    Serial.print(" ");
    analog_value1 = analogRead(ANALOG_PIN_1);
    Serial.print(analog_value1);
    Serial.println();

    char JSON[200];
    sprintf(JSON, "{\"l\":%d,\"r\":%d,\"pitch\":0,\"yaw\":0}", analog_value0, analog_value1);

    request->send(200, "text/plain", JSON);
  });
 
  server.begin();
}
 
void loop(){}
