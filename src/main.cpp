#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "AsyncTCP.h"
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include <SPIFFS.h>

const byte DNS_PORT = 53;
IPAddress apIP(4, 3, 2, 1);
DNSServer dnsServer;
AsyncWebServer server(80);
const char *ssid = "Water Sensor 001";
const char *password = "W7AvrwJJWg83e2";

struct Button{
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {0, 0, false}; //PIN, key presses, pressed flag

// variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;
unsigned long last_button_time = 0;

void IRAM_ATTR isr(){
    button_time = millis();
    if (button_time - last_button_time > 250){
        button1.numberKeyPresses++;
        button1.pressed = true;
        last_button_time = button_time;
    }
}

class CaptiveRequestHandler : public AsyncWebHandler
{
public:
    CaptiveRequestHandler()
    {

        server.onNotFound([](AsyncWebServerRequest *request)
                          { request->send(404, "text/plain", "The content you are looking for was not found."); });

        server.serveStatic("/", SPIFFS, "/assets/");
    }

    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request)
    {
        return true;
    }

    // Captive Portal Redirect
    void handleRequest(AsyncWebServerRequest *request)
    {
        AsyncWebServerResponse *response = request->beginResponse(302);
        response->addHeader(F("Location"), F("http://4.3.2.1/index.html"));
        request->send(response);
    }
};

void setup(){
    Serial.begin(115200);
    Serial.println("Serial Begin");

    // Initialize SPIFFS (ESP32 SPI Flash Storage)
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    WiFi.disconnect();   // added to start with the wifi off, avoid crashing
    WiFi.mode(WIFI_OFF); // added to start with the wifi off, avoid crashing
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password, 4, 0, 8);

    delay(500); // seems like this delay is quite important or not...?
    // ANDROID 10 WORKAROUND==================================================
    // set new WiFi configurations
    WiFi.disconnect();
    /*Stop wifi to change config parameters*/
    esp_wifi_stop();   // stop WIFI
    esp_wifi_deinit(); //"De init"
    /*Disabling AMPDU RX is necessary for Android 10 support*/
    wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT(); // We use the default config ...
    my_config.ampdu_rx_enable = 0;                             //... and modify only what we want.
    esp_wifi_init(&my_config);                                 // set the new config = "Disable AMPDU"
    esp_wifi_start();                                          // Restart WiFi
    delay(500);
    // ANDROID 10 WORKAROUND==================================================

    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    // if DNSServer is started with "*" for domain name, it will reply with
    // provided IP to all DNS request
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", apIP);

    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); // only when requested from AP
    server.begin();

    // Setup GPIO ==================================================
    pinMode(button1.PIN, INPUT_PULLUP);
    attachInterrupt(button1.PIN, isr, FALLING);
}

void loop(){
    dnsServer.processNextRequest();
    if (button1.pressed){
        Serial.printf("Button has been pressed %u times\n", button1.numberKeyPresses);
        button1.pressed = false;
    }
}