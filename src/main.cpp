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

#define LOG_SIZE 100
String logCache[LOG_SIZE]; 
byte logCacheIndex = 0; 

void log(String msg){
    logCache[logCacheIndex++] = msg;
    if(logCacheIndex >= LOG_SIZE) { logCacheIndex = 0; }
}
String getLogs(){
    String retval = "";

    for (int i=0; i < LOG_SIZE ; i++) {
        String msg = logCache[(logCacheIndex + i) % LOG_SIZE];
        if(msg.length() > 0) {
            retval += msg + "\n";
        }
    }
    return retval;
}

class CaptiveRequestHandler : public AsyncWebHandler {
  public:
    CaptiveRequestHandler() {
        server.onNotFound([](AsyncWebServerRequest *request){
            log("Request: " + request->url() + ", not found redirect");
            request->redirect("/index.html");
        });

        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            log("Request: " + request->url() + ", redirect");
            request->redirect("/index.html");
        });
        
        server.on("^\\/(.*)\\.css$", HTTP_GET, [](AsyncWebServerRequest *request){
            log("Request: " + request->url() + ", response: text/css");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".css", "text/css");
        });

        server.on("^\\/(.*)\\.js$", HTTP_GET, [](AsyncWebServerRequest *request){
            log("Request: " + request->url() + ", response: text/javascript");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".js", "text/javascript");
        });

        server.on("^\\/(.*)\\.html$", HTTP_GET, [](AsyncWebServerRequest *request){
            log("Request: " + request->url() + ", response: html");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".html");
        });

        server.on("^\\/(.*)\\.csv$", HTTP_GET, [](AsyncWebServerRequest *request){
            log("Request: " + request->url() + ", response: csv");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".csv", String(), true);
        });

        server.on("^\\/(.*)\\.png$", HTTP_GET, [](AsyncWebServerRequest *request){
            log("Request: " + request->url() + ", response: image/png");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".png", "image/png");
        });

        server.on("/logs", HTTP_GET, [] (AsyncWebServerRequest *request) {
            log("Request: " + request->url() + ", response: 200, text/plain");
            request->send(200, "text/plain", getLogs());
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