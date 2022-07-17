#include <Arduino.h>
#include <WiFi.h>
#include "AsyncTCP.h"
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include <SPIFFS.h>

const byte DNS_PORT = 53;
IPAddress apIP(4,3,2,1);
DNSServer dnsServer;
AsyncWebServer server(80);
const char *ssid = "Water Sensor 001";
const char *password = "W7AvrwJJWg83e2";


class CaptiveRequestHandler : public AsyncWebHandler {
  public:
    CaptiveRequestHandler() {
        // Route for unknown pages
        server.onNotFound([](AsyncWebServerRequest *request){
            request->redirect("/index.html");
        });

        // Route for root / web page
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->redirect("/index.html");
            //request->send(SPIFFS, "/index.html");
            //request->send(SPIFFS, "/text.txt", String(), true);
        });
        
        server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/style.css", "text/css");
        });

        server.on("/chart.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/chart.min.js", "text/javascript");
        });

        server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.html");
        });

        server.on("/Water Quality Data.csv", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/Water Quality Data.csv", String(), true);
        });
    }

    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request) {
        return true;
    }

    //Captive Portal Redirect
    void handleRequest(AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(302);
        response->addHeader(F("Location"), F("http://4.3.2.1"));
        request->send(response);
    }
};


void setup() {
    //Serial.begin(9600);
    //Serial.println("Serial Begin");

    // Initialize SPIFFS (ESP32 SPI Flash Storage)
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    WiFi.disconnect();   //added to start with the wifi off, avoid crashing
    WiFi.mode(WIFI_OFF); //added to start with the wifi off, avoid crashing
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    // if DNSServer is started with "*" for domain name, it will reply with
    // provided IP to all DNS request
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", apIP);
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP

    server.begin();
}

void loop() {
  dnsServer.processNextRequest();
  
}