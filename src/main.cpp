#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "AsyncTCP.h"
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"

//Onboard Storage
#include <SPIFFS.h>

//GPS Module
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//Analog Input
#include <driver/adc.h>
#include <esp_adc_cal.h>

char LogFilename[] = "/assets/Water Quality Data.csv";

esp_adc_cal_characteristics_t adc1_chars;

#define ALPHA_SMOOTHING 1
#define ALPHA_SMOOTHING_DIVISOR 100
#define TEMP_PIN ADC1_CHANNEL_0
uint16_t tempValue = 0xFFFF;
#define TDS_PIN ADC1_CHANNEL_1
uint16_t tdsValue = 0xFFFF;

HardwareSerial uart(1);
TinyGPSPlus gps;

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

void appendLineToCSV();
void readADC(adc1_channel_t, uint16_t *);
void readAllAdcChannels();

void IRAM_ATTR isr(){
    button_time = millis();
    if (button_time - last_button_time > 250){
        button1.numberKeyPresses++;
        button1.pressed = true;
        last_button_time = button_time;
    }
}

class CaptiveRequestHandler : public AsyncWebHandler{
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
    
    adc1_config_channel_atten(TEMP_PIN,ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_12);

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        //printf("eFuse Vref: Supported\n");
    } else {
        printf("ADC Vref Factory Setting: NOT Found\n");
    }

    // GPS Module Setup stuff
    uart.begin(9600, SERIAL_8N1, 17, 16);
}

void loop(){
    dnsServer.processNextRequest();

    while (uart.available() > 0){
        // get the byte data from the GPS
        gps.encode(uart.read());
    }

    readAllAdcChannels();    

    if (button1.pressed){

        Serial.print(tempValue);
        Serial.print("mV, ");
        Serial.print(tdsValue);
        Serial.print("mV\n");

        appendLineToCSV();
        button1.pressed = false;
    }
}

void readAllAdcChannels(){
    readADC(TEMP_PIN, &tempValue);
    readADC(TDS_PIN, &tdsValue);
}

void readADC(adc1_channel_t channel, uint16_t *value){
    uint16_t input;
    input = esp_adc_cal_raw_to_voltage(adc1_get_raw(channel), &adc1_chars);
    if(*value & 0xFFFF){ // bitwise and operation = fast way to check if *value == 0xFFFF
        *value = input;
    } else {
        *value = (input * ALPHA_SMOOTHING + (ALPHA_SMOOTHING_DIVISOR-ALPHA_SMOOTHING) * *value) 
                  / ALPHA_SMOOTHING_DIVISOR;
    }
}

void appendLineToCSV(){

    File CSV = SPIFFS.open(F(LogFilename), FILE_APPEND);
    
    if(!CSV){
        Serial.print(F("Error opening "));
        Serial.println(F(LogFilename));
        return;
    }

    CSV.print("\n");

    //UTC_Date(YYYY-MM-DD),UTC_Time(HH:MM:SS),Latitude(Decimal),Longitude(Decimal),Altitude(Meters),Temp(ADC mV),TDS(ADC mV),
    if (!gps.date.isValid()){
        Serial.print(F("****-**-**,"));
    }else{
        char sz[32];
        sprintf(sz, "%02d-%02d-%02d,", gps.date.year(), gps.date.month(), gps.date.day());
        CSV.print(sz);
    }
    
    if (!gps.time.isValid()){
        CSV.print(F("**-**-**,"));
    }else{
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d,", gps.time.hour(), gps.time.minute(), gps.time.second());
        CSV.print(sz);
    }

    if (!gps.location.isValid()){
        CSV.print(F("***.******,***.******,"));
    }else{
        CSV.print(gps.location.lat(),6);//6dp
        CSV.print(F(","));
        CSV.print(gps.location.lng(),6);
        CSV.print(F(","));
    }

    if (!gps.altitude.isValid()){
        CSV.print(F("***,"));
    }else{
        CSV.print(gps.altitude.meters(),0);//0dp
        CSV.print(F(","));
    }

    CSV.print(tempValue,0);//0dp
    CSV.print(F(","));
    CSV.print(tdsValue,0);//0dp
    CSV.print(F(","));
    // TODO print the tempValue & tdsValue & any others

    CSV.close();
}

