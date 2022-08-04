#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
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
        // server.onNotFound([](AsyncWebServerRequest *request){
        //     //log("Request: " + request->url() + ", not found redirect");
        //     request->redirect("/landing.html");
        // });
        server.onNotFound([](AsyncWebServerRequest *request){
            request->send(404, "text/plain", "The content you are looking for was not found.");
        });

        // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        //     //log("Request: " + request->url() + ", redirect");
        //     request->redirect("/landing.html");
        // });
        
        server.on("^\\/(.*)\\.css$", HTTP_GET, [](AsyncWebServerRequest *request){
            //log("Request: " + request->url() + ", response: text/css");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".css", "text/css");
        });

        server.on("^\\/(.*)\\.js$", HTTP_GET, [](AsyncWebServerRequest *request){
            //log("Request: " + request->url() + ", response: text/javascript");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".js", "text/javascript");
        });

        server.on("^\\/(.*)\\.html$", HTTP_GET, [](AsyncWebServerRequest *request){
            //log("Request: " + request->url() + ", response: html");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".html");
        });

        server.on("^\\/(.*)\\.csv$", HTTP_GET, [](AsyncWebServerRequest *request){
            //log("Request: " + request->url() + ", response: csv");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".csv", String(), true);
        });

        server.on("^\\/(.*)\\.png$", HTTP_GET, [](AsyncWebServerRequest *request){
            //log("Request: " + request->url() + ", response: image/png");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".png");
        });

        server.on("^\\/(.*)\\.svg$", HTTP_GET, [](AsyncWebServerRequest *request){
           log("Request: " + request->url() + ", response: image/svg+xml");
           request->send(SPIFFS, "/" + request->pathArg(0) + ".svg");
        });

        server.on("^\\/(.*)\\.woff2$", HTTP_GET, [](AsyncWebServerRequest *request){
            //log("Request: " + request->url() + ", response: font/woff2");
            request->send(SPIFFS, "/" + request->pathArg(0) + ".woff2");
        });

        // server.on("/logs", HTTP_GET, [] (AsyncWebServerRequest *request) {
        //     //log("Request: " + request->url() + ", response: 200, text/plain");
        //     request->send(200, "text/plain", getLogs());
        // });
    }

    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request) {
        return true;
    }

    //Captive Portal Redirect
    void handleRequest(AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(302);
        response->addHeader(F("Location"), F("http://4.3.2.1/landing.html"));
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
    WiFi.softAP(ssid, password, 4, 0, 8);

    delay(500);//seems like this delay is quite important or not...?
    //ANDROID 10 WORKAROUND==================================================
    //set new WiFi configurations
    WiFi.disconnect();
    /*Stop wifi to change config parameters*/
    esp_wifi_stop(); //stop WIFI
    esp_wifi_deinit(); //"De init"
    /*Disabling AMPDU RX is necessary for Android 10 support*/
    wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();   //We use the default config ...
    my_config.ampdu_rx_enable = 0;                               //... and modify only what we want.
    esp_wifi_init(&my_config); //set the new config = "Disable AMPDU"
    esp_wifi_start(); //Restart WiFi
    delay(500);
    //ANDROID 10 WORKAROUND==================================================

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