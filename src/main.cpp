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


#define LED_PIN 13
#define NUMPIXELS 26
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRBW + NEO_KHZ800);
unsigned long buttonStart = 0; //Time(millis) when the button was pressed
unsigned long lastFrame = 0; //Time(millis) when the frame of the leds was displayed
bool recording = false; //are we recording right now?
bool waterPresent = false; //are we in Water right now?
uint16_t ledIndicatorPos = 0; //how far arround the circle we are right now 0=all off 24*255=all on
#define RECORDING_TIME 30.00 //time to record in seconds (needs .00 to force floating point)

char LogFilename[] = "/assets/Water Quality Data.csv";



//----- Global Sensor Settings -----
esp_adc_cal_characteristics_t adc1_chars;
#define ALPHA_SMOOTHING 1
#define ALPHA_SMOOTHING_DIVISOR 100

#define TDS_PIN ADC1_CHANNEL_5
uint16_t tdsValue = 0xFFFF;
// Regression Calibration of pH y=a+bx+cx^2
float tdsa = -54;
float tdsb = 0.368;
float tdsc = 0.0000666;

#define PH_PIN ADC1_CHANNEL_3
uint16_t phValue = 0xFFFF;
// Regression Calibration of pH y=a+bx+cx^2
float pha = 32.8;
float phb = -0.0151;
float phc = 0.00000142;
// Regression Calibration of pH vs Temp y=a+bx
float phta = -0.713;
float phtb = 0.000293;

#define WATER_TEMP_PIN ADC1_CHANNEL_7
uint16_t waterTempValue = 0xFFFF;
//Cubic Regression Calibration y=a+bx+cx^2+dx^3
float waterTempa = -48.3;
float waterTempb = 0.0893;
float waterTempc = -0.00004;
float waterTempd = 0.00000000795;


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
    strip.fill(strip.Color(255, 255, 255)); //White LEDS
    strip.show();
    

    Serial.begin(115200);
    Serial.println("");
    Serial.println("Kea Water 001");
    Serial.print("UTC_Date(YYYY-MM-DD),UTC_Time(HH:MM:SS),Latitude(Decimal),Longitude(Decimal),Altitude(Meters),Water Temperature(Deg C), TDS (PPM), pH (0-14)");

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
    pinMode(19, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(27, OUTPUT);


    digitalWrite(19, HIGH);
    delay(1000);
    digitalWrite(2, HIGH);
    delay(1000);
    digitalWrite(25, HIGH);
    delay(1000);
    digitalWrite(26, HIGH);
    delay(1000);
    digitalWrite(27, HIGH);
    delay(1000);


    pinMode(button1.PIN, INPUT_PULLUP);
    attachInterrupt(button1.PIN, isr, FALLING);

    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        //printf("eFuse Vref: Supported\n");
    } else {
        printf("ADC Vref Factory Setting: NOT Found\n");
        strip.fill(strip.Color(0, 255, 0)); //GREEN LEDS
    }
    
    adc1_config_channel_atten(WATER_TEMP_PIN,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(PH_PIN,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(TDS_PIN,ADC_ATTEN_DB_11);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_12);


    // GPS Module serial comms setup
    uart.begin(9600, SERIAL_8N1, 16, 17);

    for (uint8_t i = 0; i < ALPHA_SMOOTHING_DIVISOR; i++){
        readAllAdcChannels();
    }

    
    

}

void loop(){
    readAllAdcChannels();

    dnsServer.processNextRequest();

    while (uart.available() > 0){
        gps.encode(uart.read());    // get the byte data from the GPS
    }

    if (recording==false){
        if (gps.location.isValid()){
            strip.fill(strip.Color(0, 255, 0)); //Green LEDS
            strip.show();
        }

        if (tdsValue > 250){
            waterPresent = true;
            isr();
        }

        if (button1.pressed){ 
            strip.clear();
            strip.show();
            recording = true;
            button1.pressed = false;
        }
        
    }else{
        updateTimerAndLEDS();
    }
   
}

void updateTimerAndLEDS(){
    if (recording==true && millis() > lastFrame + 20){
        ledIndicatorPos = (millis()-last_button_time)*((NUMPIXELS*255)/(RECORDING_TIME*1000));
        if (ledIndicatorPos<(NUMPIXELS*255)){
            u_int16_t i = ledIndicatorPos;
            u_int8_t pixel = 0;
            while (i != 0){
                if (i > 255){
                    //strip.setPixelColor(pixel,strip.gamma32(strip.ColorHSV(pixel*(65535/NUMPIXELS),255,255)));
                    i = i - 255;
                    pixel++;
                }else{
                    strip.setPixelColor(pixel,strip.gamma32(strip.ColorHSV(pixel*(65535/NUMPIXELS),255,i)));
                    i = 0;
                    strip.show();
                }
                
            }
            lastFrame = millis();
        } else {  
            appendLineToCSV();
            recording = false;
            strip.clear();
            strip.show();
            lastFrame = 0;       
            //esp_sleep_enable_timer_wakeup(60* 1000000);//1hrs
            esp_deep_sleep_start();

        }
    }

}



void readAllAdcChannels(){
    readADC(WATER_TEMP_PIN, &waterTempValue);
    readADC(TDS_PIN, &tdsValue);
    readADC(PH_PIN, &phValue);
    //readADC(PH_TEMP_PIN, &phTempValue);
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
    Serial.print("\n");

    //UTC_Date(YYYY-MM-DD),UTC_Time(HH:MM:SS),Latitude(Decimal),Longitude(Decimal),Altitude(Meters),Water Temperature(Deg C), TDS (PPM), pH (0-14)
    if (!gps.date.isValid()){
        CSV.print(F("****-**-**,"));
        Serial.print(F("****-**-**,"));
    }else{
        char sz[32];
        sprintf(sz, "%02d-%02d-%02d,", gps.date.year(), gps.date.month(), gps.date.day());
        CSV.print(sz);
        Serial.print(sz);
    }

    //-----Time from GPS-----
    if (!gps.time.isValid()){
        CSV.print(F("**:**:**,"));
        Serial.print(F("**:**:**,"));
    }else{
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d,", gps.time.hour(), gps.time.minute(), gps.time.second());
        CSV.print(sz);
        Serial.print(sz);
    }

    //-----Lat and Long from GPS-----
    if (!gps.location.isValid()){
        CSV.print(F("***.******,***.******,"));
        Serial.print(F("***.******,***.******,"));

    }else{
        CSV.print(gps.location.lat(),6);//6dp
        CSV.print(F(","));
        CSV.print(gps.location.lng(),6);
        CSV.print(F(","));
        Serial.print(gps.location.lat(),6);//6dp
        Serial.print(F(","));
        Serial.print(gps.location.lng(),6);
        Serial.print(F(","));
    }

    //-----Altitude from GPS-----
    if (!gps.altitude.isValid()){
        CSV.print(F("***,"));
        Serial.print(F("***,"));
    }else{
        CSV.print(gps.altitude.meters(),0);//0dp
        CSV.print(F(","));
        Serial.print(gps.altitude.meters(),0);//0dp
        Serial.print(F(","));
    }



    //-----Water Temperature Conversion from Raw to *C -----
    if (waterTempValue<=0){//142 is a disconnected sensor
        CSV.print(F("***,"));
        Serial.print(F("***,"));
    }else{
        //Cubic Regression Calibration y=a+bx+cx^2+dx^3
        float waterTempOutput = waterTempa + waterTempb * (waterTempValue) + waterTempc * pow(waterTempValue,2) + waterTempd * pow(waterTempValue,3);
        CSV.print(waterTempOutput);
        CSV.print(",");
        Serial.print(waterTempOutput);
        Serial.print(",");
        
    }
    //------------------------------------------------------


    //-----TDS Conversion from Raw to PPM -----
    if (tdsValue<=142){//141 is a disconnected sensor
        CSV.print(F("0,"));
        Serial.print(F("0,"));
    }else{
        //Cubic Regression Calibration y=a+bx+cx^2+dx^3
        float tdsOutput = tdsa + tdsb * (tdsValue) + tdsc * pow(tdsValue,2);
        CSV.print(tdsOutput);
        CSV.print(",");
        Serial.print(tdsOutput);
        Serial.print(",");
    }
    //------------------------------------------------------


    //-----pH Conversion from Raw to 0-14 -----
    if (phValue<=142){//142 is a disconnected sensor
        CSV.print(F("***,"));
        //Serial.print(F("***,"));
    }else{
        //Regression Calibration y=a+bx+cx^2
        float phNoTempCal = pha + phb * (phValue) + phc * pow(phValue,2);
        //Regression Calibration y=a+bx
        float phOutput = phNoTempCal + phta + phtb * (waterTempValue);
        CSV.print(phOutput);
        CSV.print(",");
        Serial.print(phOutput);
        Serial.print(",");
    }
    //------------------------------------------------------


    CSV.close();
}

