#include <Arduino.h>

//Wifi, Webserver and DNS
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

//LED Ring
#include <Adafruit_NeoPixel.h>

#define LED_PIN 2
#define NUMPIXELS 24
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
unsigned long buttonStart = 0; //Time(millis) when the button was pressed
bool recording = false; //are we recording right now?
uint8_t ledIndicatorPos = 0; //how far arround the circle we are right now 0=all off 24=all on
#define RECORDING_TIME 1.00 //time to record in seconds (needs .00 to force floating point)

char LogFilename[] = "/assets/Water Quality Data.csv";



//----- Global Sensor Settings -----
esp_adc_cal_characteristics_t adc1_chars;
#define ALPHA_SMOOTHING 1
#define ALPHA_SMOOTHING_DIVISOR 100

#define TDS_PIN ADC1_CHANNEL_0
uint16_t tdsValue = 0xFFFF;

#define PH_PIN ADC1_CHANNEL_3
uint16_t phValue = 0xFFFF;

#define PH_TEMP_PIN ADC1_CHANNEL_6
uint16_t phTempValue = 0xFFFF;

#define WATER_TEMP_PIN ADC1_CHANNEL_7
uint16_t waterTempValue = 0xFFFF;
#define THERMISTORNOMINAL 10000 // killo ohms value resistance at 25 degrees C     
#define BCOEFFICIENT 3950 // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 9980 // killo ohms value of the 'other' resistor
//Cubic Regression y=a+bx+cx^2+dx^3
float waterTempa = -19.2;
float waterTempb = 1.35;
float waterTempc = 0.00871;
float waterTempd = -0.000257;


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
//unsigned long button_time = 0;
uint32_t last_button_time = 0;

void appendLineToCSV();
void readADC(adc1_channel_t, uint16_t *);
void readAllAdcChannels();
void updateTimerAndLEDS();

void IRAM_ATTR isr(){
    if (millis() - last_button_time > 250 && recording == false){
        //button1.numberKeyPresses++;
        button1.pressed = true;
        last_button_time = millis();
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

    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    strip.setBrightness(255);
    

    Serial.begin(115200);
    Serial.println("Serial Begin");

    // Initialize SPIFFS (ESP32 SPI Flash Storage)
    if (!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        strip.fill(strip.Color(255, 0, 0)); //RED LEDS
        return;
    }

    // Wifi Setup ===========================================================
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF); // added to start with the wifi off, avoid crashing
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password, 4, 0, 8);

    
    // ANDROID 10 WIFI WORKAROUND============================================
    delay(500); // seems like this delay is quite important or not...?
    WiFi.disconnect(); //Stop wifi to change config parameters
    esp_wifi_stop();
    esp_wifi_deinit();
    /*Disabling AMPDU RX is necessary for Android 10 support*/
    wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
    my_config.ampdu_rx_enable = 0;                             
    esp_wifi_init(&my_config);                                
    esp_wifi_start();
    delay(500);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    // DNS Server Setup ===========================================================
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);// if DNSServer is started with "*" (catchall) for domain name
    dnsServer.start(DNS_PORT, "*", apIP);

    // Webserver Setup ===========================================================
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); //setup above function to run when requested from device
    server.begin();

    // Setup GPIO ==================================================
    pinMode(button1.PIN, INPUT_PULLUP);
    attachInterrupt(button1.PIN, isr, FALLING);

    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        //printf("eFuse Vref: Supported\n");
    } else {
        printf("ADC Vref Factory Setting: NOT Found\n");
        strip.fill(strip.Color(0, 255, 0)); //GREEN LEDS
    }
    
    adc1_config_channel_atten(WATER_TEMP_PIN,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PH_TEMP_PIN,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PH_PIN,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(TDS_PIN,ADC_ATTEN_DB_11);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_12);


    // GPS Module serial comms setup
    uart.begin(9600, SERIAL_8N1, 17, 16);

    strip.show();
}

void loop(){
    dnsServer.processNextRequest();

    while (uart.available() > 0){
        gps.encode(uart.read());    // get the byte data from the GPS
    }

    // //-----PH Conversion-----
    float phVolt=phValue*3.300/4095.0/6;
    float ph_act = -5.70 *phVolt + (20.24 - 0.7);

    Serial.print("phValue = ");
    Serial.print(phValue);
    Serial.print("/4095, ");

    Serial.print("phOutput = ");
    Serial.print(ph_act);
    Serial.println(" , ");


    readAllAdcChannels();
    updateTimerAndLEDS();

    if (button1.pressed){
        recording = true;
        button1.pressed = false;
    }
}

void updateTimerAndLEDS(){
    if (recording==true){
        if (((millis()-last_button_time)/((RECORDING_TIME/NUMPIXELS)*1000))+1 >= ledIndicatorPos){
            if (ledIndicatorPos <=NUMPIXELS){
                uint8_t i;
                for(i=0; i< ledIndicatorPos; i++) {
                strip.setPixelColor(i,strip.gamma32(strip.ColorHSV(i*(65535/NUMPIXELS))));
                }
                strip.show();
                ledIndicatorPos++;

            } else {
                appendLineToCSV();
                recording = false;
                ledIndicatorPos = 0;
                strip.clear();
                strip.show();
            }
        }
    } 
}



void readAllAdcChannels(){
    readADC(WATER_TEMP_PIN, &waterTempValue);
    readADC(TDS_PIN, &tdsValue);
    readADC(PH_PIN, &phValue);
    readADC(PH_TEMP_PIN, &phTempValue);
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
        CSV.print(F("****-**-**,"));
    }else{
        char sz[32];
        sprintf(sz, "%02d-%02d-%02d,", gps.date.year(), gps.date.month(), gps.date.day());
        CSV.print(sz);
    }
    
    if (!gps.time.isValid()){
        CSV.print(F("**:**:**,"));
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

    //-----Temperature Conversion-----    
    float resistance = 4095.00 / waterTempValue -1;
    resistance = SERIESRESISTOR / resistance;
    float steinhart = resistance / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25 + 273.15);            // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert absolute temp to C
    //Cubic Regression y=a+bx+cx^2+dx^3
    float waterTempOutput = waterTempa + waterTempb * (steinhart) + waterTempc * pow(steinhart,2) + waterTempd * pow(steinhart,3);

    Serial.print("Water Temperature ");
    Serial.print(waterTempOutput,3);
    Serial.print(" *C     ");


    //-----TDS Conversion-----
    // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float TDSVoltage = tdsValue * (3.3000 / 4095.0);
    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
    float TDScompensationCoefficient = 1.0+0.02*(waterTempOutput-25.0);
    //temperature compensation
    float TDScompensatedVoltage=TDSVoltage/TDScompensationCoefficient;
    //convert voltage value to tds value
    float tdsOutput=(133.42*pow(TDScompensatedVoltage,3) - 255.86*pow(TDScompensatedVoltage,2) + 857.39*TDScompensatedVoltage)*0.5;
    
    Serial.print("tdsValue = ");
    Serial.print(tdsValue);
    Serial.print("/4095, ");

    Serial.print("tdsOutput = ");
    Serial.print(tdsOutput,0);
    Serial.println("ppm");


    //-----PH Conversion-----
    float pHVol = (float)phValue * 5.0 / 1024 / 6;
    float phOutput = -5.70 * pHVol + 21.34;

    Serial.print("phValue = ");
    Serial.print(phValue);
    Serial.print("/4095, ");

    Serial.print("phOutput = ");
    Serial.print(phOutput);
    Serial.print(" , ");

    Serial.print("phTempValue = ");
    Serial.print(phTempValue);
    Serial.println("/4095, ");

    CSV.close();
}

