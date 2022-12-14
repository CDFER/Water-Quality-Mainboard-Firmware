#include <Arduino.h>

// Wifi, Webserver and DNS
#include <DNSServer.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"

// Onboard Flash Storage
#include <SPIFFS.h>

// GPS Module
// #include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Analog Input
#include <driver/adc.h>
#include <esp_adc_cal.h>

// Adressable LEDs
#include <Adafruit_NeoPixel.h>

const char *ssid = "captive";  // FYI The SSID can't have a space in it.
const char *password = "12345678";
const IPAddress localIP(4, 3, 2, 1);					// the IP address the webserver, Samsung requires the IP to be in public space
const IPAddress gatewayIP(4, 3, 2, 1);					// IP address of the network
const String localIPURL = "http://4.3.2.1/index.html";	// URL to the webserver

char LogFilename[] = "/Water Quality Data.csv";

// #define RECORDING_TIME 3000 //time to record in milliseconds

// //----- Global Analog Settings -----
// esp_adc_cal_characteristics_t adc1_chars;
// #define ALPHA_SMOOTHING 1
// #define ALPHA_SMOOTHING_DIVISOR 100

// uint16_t tdsValue = 0xFFFF;
// // Regression Calibration of pH y=a+bx+cx^2
// float tdsa = -54;
// float tdsb = 0.368;
// float tdsc = 0.0000666;

// uint16_t phValue = 0xFFFF;
// // Regression Calibration of pH y=a+bx+cx^2
// float pha = 32.8;
// float phb = -0.0151;
// float phc = 0.00000142;
// // Regression Calibration of pH vs Temp y=a+bx
// float phta = -0.713;
// float phtb = 0.000293;

// uint16_t nitrateValue = 0xFFFF;
// //Cubic Regression Calibration y=a+bx+cx^2+dx^3
// float nitratea = 0;
// float nitrateb = 0;
// float nitratec = 0;
// float nitrated = 0;

// //State Machine
enum { STARTUP,
	   FIND_GPS,
	   PRE_IDLE,
	   IDLE,
	   PRE_RECORDING,
	   RECORDING,
	   CHARGING };
uint8_t state = STARTUP;

// void appendLineToCSV();
// void readADC(adc1_channel_t, uint16_t *);
// void readAllAdcChannels();

void Sensors(void *parameter) {
	#define SENSOR_POWER 19

	#define BATTERY_PIN ADC1_CHANNEL_0 //SENVP, VBAT - 33k - BATTERY_PIN - 10k AOGND
	#define TEMP_OW_PIN 33 // GPIO where the DS18B20 Onewire Temp Pin is connected to

	#define TDS_PIN ADC1_CHANNEL_6
	#define PH_PIN ADC1_CHANNEL_7
	#define NITRATE_PIN ADC1_CHANNEL_4

	// Setup GPIO ==================================================
	pinMode(SENSOR_POWER, OUTPUT);
	digitalWrite(SENSOR_POWER, HIGH);

	vTaskDelete(NULL);
}

void debugLEDs(void *parameter) {
	#define RED_LED_PIN 25
	#define YELLOW_LED_PIN 26
	#define GREEN_LED_PIN 27

	// Setup GPIO ==================================================
	pinMode(RED_LED_PIN, OUTPUT);
	pinMode(YELLOW_LED_PIN, OUTPUT);
	pinMode(GREEN_LED_PIN, OUTPUT);

	vTaskDelete(NULL);
}

void GPS(void *parameter) {
	#define GPSTX_PIN 17
	#define GPSRX_PIN 16
	#define GPS_POWER 18

	// // Startup GPS ==================================================
	pinMode(GPS_POWER, OUTPUT);
	digitalWrite(GPS_POWER, HIGH);

	HardwareSerial uart(1);
	TinyGPSPlus gps;

	uart.begin(9600, SERIAL_8N1, 16, 17); //Setup GPS Serial Coms

	vTaskDelete(NULL);
}

void colorWipe(uint32_t color, int wait) {
	
}

void ARGBLEDs(void *parameter) {
	#define LED_POWER 2
	#define ARGB1_PIN 13
	#define ARGB1_PIXELS 15
	#define FRAMETIME 20 //Milliseconds 20 = 50fps

	Adafruit_NeoPixel ARGB1 = Adafruit_NeoPixel(ARGB1_PIXELS, ARGB1_PIN, NEO_GRBW + NEO_KHZ800);

	enum { PULSE_GREEN,
		SOLID_GREEN,
		FADE_OUT,
		BLACK,
		OFF};
	uint8_t ARGB1state = OFF;

	uint32_t colour = ARGB1.Color(0, 0, 0, 0);

	// Setup ARGB LEDs ==================================================
	pinMode(LED_POWER, OUTPUT);
	digitalWrite(LED_POWER, LOW);

	ARGB1.begin();
	ARGB1.setBrightness(255);
	ARGB1.fill(colour);

	digitalWrite(LED_POWER, HIGH);
	ARGB1.show();

	while (true){
		switch (ARGB1state) {
			case PULSE_GREEN:
				
				break;

			case PRE_RECORDING:
				break;

			case CHARGING:
				break;

			case RECORDING:
				break;

			default:
				ESP_LOGE("ARGB1 state machine", "Hit Default, Restarting...");
				vTaskDelay(1000 / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
				ESP.restart();
				break;
		}

		for (int i = 0; i < ARGB1_PIXELS; i++) {		 // For each pixel in strip...
			ARGB1.setPixelColor(i, colour);				 //  Set pixel's color (in RAM)
			ARGB1.show();								 //  Update strip to match
			vTaskDelay(FRAMETIME / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
		}

		for (int i = 0; i < ARGB1_PIXELS; i++) {		   // For each pixel in strip...
			ARGB1.setPixelColor(i, );			   //  Set pixel's color (in RAM)
			ARGB1.show();							   //  Update strip to match
			vTaskDelay(FRAMETIME / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
		}
	}
	

	vTaskDelete(NULL);
}

void turbidity(void *parameter) {
	#define ARGB2_TURBIDITY_PIN 23
	#define ARGB2_TURBIDITY_PIXELS 2
	Adafruit_NeoPixel ARGB2 = Adafruit_NeoPixel(ARGB2_TURBIDITY_PIXELS, ARGB2_TURBIDITY_PIN, NEO_GRBW + NEO_KHZ800);

	ARGB2.begin();
	ARGB2.setBrightness(255);
	ARGB2.fill(ARGB2.Color(0, 0, 0, 0));
	ARGB2.show();

	vTaskDelete(NULL);
}

void accessPoint(void *parameter) {
#define DNS_INTERVAL 10	 // ms between processing dns requests: dnsServer.processNextRequest();

#define MAX_CLIENTS 4
#define WIFI_CHANNEL 6	// 2.4ghz channel 6

	const IPAddress subnetMask(255, 255, 255, 0);

	DNSServer dnsServer;
	AsyncWebServer server(80);

	WiFi.mode(WIFI_AP);
	WiFi.softAPConfig(localIP, gatewayIP, subnetMask);	// Samsung requires the IP to be in public space
	WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_CLIENTS);
	WiFi.setSleep(false);

	dnsServer.setTTL(300);				// set 5min client side cache for DNS
	dnsServer.start(53, "*", localIP);	// if DNSServer is started with "*" for domain name, it will reply with provided IP to all DNS request

	// ampdu_rx_disable android workaround see https://github.com/espressif/arduino-esp32/issues/4423
	esp_wifi_stop();
	esp_wifi_deinit();

	wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();	// We use the default config ...
	my_config.ampdu_rx_enable = false;							//... and modify only what we want.

	esp_wifi_init(&my_config);	// set the new config
	esp_wifi_start();			// Restart WiFi
	delay(100);					// this is necessary don't ask me why

	//======================== Webserver ========================
	// WARNING IOS (and maybe macos) WILL NOT POP UP IF IT CONTAINS THE WORD "Success" https://www.esp8266.com/viewtopic.php?f=34&t=4398
	// SAFARI (IOS) IS STUPID, G-ZIPPED FILES CAN'T END IN .GZ https://github.com/homieiot/homie-esp8266/issues/476 this is fixed by the webserver serve static function.
	// SAFARI (IOS) there is a 128KB limit to the size of the HTML. The HTML can reference external resources/images that bring the total over 128KB
	// SAFARI (IOS) popup browser has some severe limitations (javascript disabled, cookies disabled)

	server.serveStatic("/Water_Quality_Data.csv", SPIFFS, "/Water_Quality_Data.csv").setCacheControl("no-store");  // fetch data file every page reload
	server.serveStatic("/index.html", SPIFFS, "/index.html").setCacheControl("max-age=120");					   // serve html file

	// Required
	server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });	// windows 11 captive portal workaround
	server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });								// Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

	// Background responses: Probably not all are Required, but some are. Others might speed things up?
	// A Tier (commonly used by modern systems)
	server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });		   // android captive portal redirect
	server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			   // microsoft redirect
	server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });  // apple call home
	server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });	   // firefox captive portal call home
	server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });					   // firefox captive portal call home
	server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			   // windows call home

	// B Tier (uncommon)
	//  server.on("/chrome-variations/seed",[](AsyncWebServerRequest *request){request->send(200);}); //chrome captive portal call home
	//  server.on("/service/update2/json",[](AsyncWebServerRequest *request){request->send(200);}); //firefox?
	//  server.on("/chat",[](AsyncWebServerRequest *request){request->send(404);}); //No stop asking Whatsapp, there is no internet connection
	//  server.on("/startpage",[](AsyncWebServerRequest *request){request->redirect(localIPURL);});

	server.serveStatic("/", SPIFFS, "/").setCacheControl("max-age=120").setDefaultFile("index.html");  // serve any file on the device when requested

	server.onNotFound([](AsyncWebServerRequest *request) {
		request->redirect(localIPURL);
		ESP_LOGW("WebServer", "Page not found sent redirect to localIPURL");
		// DEBUG_SERIAL.print("onnotfound ");
		// DEBUG_SERIAL.print(request->host());       //This gives some insight into whatever was being requested on the serial monitor
		// DEBUG_SERIAL.print(" ");
		// DEBUG_SERIAL.print(request->url());
		// DEBUG_SERIAL.print(" sent redirect to " + localIPURL +"\n");
	});

	server.begin();

	ESP_LOGI("WebServer", "Startup complete");

	while (true) {
		dnsServer.processNextRequest();
		vTaskDelay(DNS_INTERVAL / portTICK_PERIOD_MS);
	}
}

void setup() {
	#if USE_SERIAL == true
		Serial.begin(115200);
		while (!Serial)
			;
		ESP_LOGI("Base ESP32 Project", "Compiled " __DATE__ " " __TIME__ " by CD_FER");
	#endif

	if (!SPIFFS.begin(true)) {	// Initialize SPIFFS (ESP32 SPI Flash Storage)
		ESP_LOGE("File System Error", "Can't mount SPIFFS");
	}

	// 			Function, Name (for debugging), Stack size, Params, Priority, Handle
	xTaskCreate(accessPoint, "accessPoint", 5000, NULL, 1, NULL);
	xTaskCreate(powerRails, "powerRails", 5000, NULL, 1, NULL);
	xTaskCreate(ARGBLEDs, "ARGBLEDs", 5000, NULL, 1, NULL);
	xTaskCreate(GPS, "GPS", 5000, NULL, 1, NULL);

	// // Setup Analog Input ==================================================
	// if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
	//     //printf("eFuse Vref: Supported\n");
	// } else {
	//     Serial.println("ADC Vref Factory Setting: NOT Found");
	//     digitalWrite(RED_LED_PIN, HIGH);
	//     delay(1000);
	//     ESP.restart();
	// }

	// //adc1_config_channel_atten(WATER_TEMP_PIN,ADC_ATTEN_DB_11);
	// adc1_config_channel_atten(PH_PIN,ADC_ATTEN_DB_11);
	// adc1_config_channel_atten(TDS_PIN,ADC_ATTEN_DB_11);
	// adc1_config_channel_atten(NITRATE_PIN,ADC_ATTEN_DB_11);
	// adc1_config_channel_atten(BATTERY_PIN,ADC_ATTEN_DB_11);

	// esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
	// adc1_config_width(ADC_WIDTH_BIT_12);
}

void loop() {
	vTaskDelay(1000 / portTICK_PERIOD_MS);	// Keep RTOS Happy when nothing is here
	switch (state) {
		case STARTUP:  // Find What State I should be in
			Serial.println("case STARTUP");
			ARGB1.fill(ARGB1.Color(255, 165, 0, 0));
			ARGB1.show();

			// for (uint8_t i = 0; i < ALPHA_SMOOTHING_DIVISOR; i++){
			//     readAllAdcChannels();
			//     dnsServer.processNextRequest();
			// }
			state = FIND_GPS;

			break;

		case FIND_GPS:
			Serial.println("case FIND_GPS");
			ARGB1.fill(ARGB1.Color(165, 255, 0, 0));
			ARGB1.show();

			// while(!gps.location.isValid()){
			//     if (uart.available() > 0){
			//         gps.encode(uart.read());    // get the byte data from the GPS
			//     }
			//     dnsServer.processNextRequest();
			// }

			// Serial.print(gps.location.lat(),6);//6dp
			// Serial.print(F(","));
			// Serial.println(gps.location.lng(),6);

			state = PRE_IDLE;
			break;

		case PRE_IDLE:
			Serial.println("case PRE_IDLE");
			ARGB1.fill(ARGB1.Color(0, 255, 0, 0));
			ARGB1.show();

			ARGB2.fill(ARGB1.Color(0, 0, 0, 255));
			ARGB2.show();

			state = IDLE;
			break;

		case IDLE:
			// dnsServer.processNextRequest();
			break;

		case PRE_RECORDING:
			break;

		case CHARGING:
			break;

		case RECORDING:
			break;

		default:
			ESP_LOGE("Hit default Case in state machine", "Restarting...");
			vTaskDelay(1000 / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
			ESP.restart();
			break;
	}
}

// void wifiSetup(){
//     digitalWrite(YELLOW_LED_PIN, HIGH);
//     Serial.println("wifiSetup()");
//     // Wifi Setup ===========================================================
//     WiFi.disconnect();
//     WiFi.mode(WIFI_OFF); // added to start with the wifi off, avoid crashing
//     WiFi.mode(WIFI_AP);
//     //WiFi.softAP(ssid, password, 4, 0, 4);
//     WiFi.softAP(ssid, NULL, 4, 0, 4);

//     // ANDROID 10 WIFI WORKAROUND============================================
//     delay(500); // seems like this delay is quite important or not...?
//     WiFi.disconnect(); //Stop wifi to change config parameters
//     esp_wifi_stop();
//     esp_wifi_deinit();
//     /*Disabling AMPDU RX is necessary for Android 10 support*/
//     wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
//     my_config.ampdu_rx_enable = 0;
//     esp_wifi_init(&my_config);
//     esp_wifi_start();
//     delay(500);
//     WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

//     // DNS Server Setup ===========================================================
//     dnsServer.setErrorReplyCode(DNSReplyCode::NoError);// if DNSServer is started with "*" (catchall) for domain name
//     dnsServer.start(DNS_PORT, "*", apIP);

//     // Webserver Setup ===========================================================
//     server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); //setup above function to run when requested from device
//     server.begin();
//     digitalWrite(YELLOW_LED_PIN, LOW);
// }

// void readAllAdcChannels(){
//     //readADC(WATER_TEMP_PIN, &waterTempValue);
//     readADC(TDS_PIN, &tdsValue);
//     readADC(PH_PIN, &phValue);
//     //readADC(PH_TEMP_PIN, &phTempValue);
// }

// void readADC(adc1_channel_t channel, uint16_t *value){
//     uint16_t input;
//     input = esp_adc_cal_raw_to_voltage(adc1_get_raw(channel), &adc1_chars);
//     if(*value & 0xFFFF){ // bitwise and operation = fast way to check if *value == 0xFFFF
//         *value = input;
//     } else {
//         *value = (input * ALPHA_SMOOTHING + (ALPHA_SMOOTHING_DIVISOR-ALPHA_SMOOTHING) * *value)
//                   / ALPHA_SMOOTHING_DIVISOR;
//     }
// }

// void appendLineToCSV(){

//     File CSV = SPIFFS.open(F(LogFilename), FILE_APPEND);

//     if(!CSV){
//         Serial.print(F("Error opening "));
//         Serial.println(F(LogFilename));
//         return;
//     }

//     CSV.print("\n");
//     Serial.print("\n");

//     //UTC_Date(YYYY-MM-DD),UTC_Time(HH:MM:SS),Latitude(Decimal),Longitude(Decimal),Altitude(Meters),Water Temperature(Deg C), TDS (PPM), pH (0-14)
//     if (!gps.date.isValid()){
//         CSV.print(F("****-**-**,"));
//         Serial.print(F("****-**-**,"));
//     }else{
//         char sz[32];
//         sprintf(sz, "%02d-%02d-%02d,", gps.date.year(), gps.date.month(), gps.date.day());
//         CSV.print(sz);
//         Serial.print(sz);
//     }

//     //-----Time from GPS-----
//     if (!gps.time.isValid()){
//         CSV.print(F("**:**:**,"));
//         Serial.print(F("**:**:**,"));
//     }else{
//         char sz[32];
//         sprintf(sz, "%02d:%02d:%02d,", gps.time.hour(), gps.time.minute(), gps.time.second());
//         CSV.print(sz);
//         Serial.print(sz);
//     }

//     //-----Lat and Long from GPS-----
//     if (!gps.location.isValid()){
//         CSV.print(F("***.******,***.******,"));
//         Serial.print(F("***.******,***.******,"));

//     }else{
//         CSV.print(gps.location.lat(),6);//6dp
//         CSV.print(F(","));
//         CSV.print(gps.location.lng(),6);
//         CSV.print(F(","));
//         Serial.print(gps.location.lat(),6);//6dp
//         Serial.print(F(","));
//         Serial.print(gps.location.lng(),6);
//         Serial.print(F(","));
//     }

//     //-----Altitude from GPS-----
//     if (!gps.altitude.isValid()){
//         CSV.print(F("***,"));
//         Serial.print(F("***,"));
//     }else{
//         CSV.print(gps.altitude.meters(),0);//0dp
//         CSV.print(F(","));
//         Serial.print(gps.altitude.meters(),0);//0dp
//         Serial.print(F(","));
//     }

//     //-----Water Temperature Conversion from Raw to *C -----
//     /*if (waterTempValue<=0){//142 is a disconnected sensor
//         CSV.print(F("***,"));
//         Serial.print(F("***,"));
//     }else{
//         //Cubic Regression Calibration y=a+bx+cx^2+dx^3
//         float waterTempOutput = waterTempa + waterTempb * (waterTempValue) + waterTempc * pow(waterTempValue,2) + waterTempd * pow(waterTempValue,3);
//         CSV.print(waterTempOutput);
//         CSV.print(",");
//         Serial.print(waterTempOutput);
//         Serial.print(",");

//     }*/
//     //------------------------------------------------------

//     //-----TDS Conversion from Raw to PPM -----
//     if (tdsValue<=142){//141 is a disconnected sensor
//         CSV.print(F("0,"));
//         Serial.print(F("0,"));
//     }else{
//         //Cubic Regression Calibration y=a+bx+cx^2+dx^3
//         float tdsOutput = tdsa + tdsb * (tdsValue) + tdsc * pow(tdsValue,2);
//         CSV.print(tdsOutput);
//         CSV.print(",");
//         Serial.print(tdsOutput);
//         Serial.print(",");
//     }
//     //------------------------------------------------------

//     //-----pH Conversion from Raw to 0-14 -----
//     /*if (phValue<=142){//142 is a disconnected sensor
//         CSV.print(F("***,"));
//         //Serial.print(F("***,"));
//     }else{
//         //Regression Calibration y=a+bx+cx^2
//         float phNoTempCal = pha + phb * (phValue) + phc * pow(phValue,2);
//         //Regression Calibration y=a+bx
//         float phOutput = phNoTempCal + phta + phtb * (waterTempValue);
//         CSV.print(phOutput);
//         CSV.print(",");
//         Serial.print(phOutput);
//         Serial.print(",");
//     }*/
//     //------------------------------------------------------

//     CSV.close();
// }
