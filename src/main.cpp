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
#include <NeoPixelBrightnessBus.h>	// instead of NeoPixelBus.h

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

// State Machine
enum deviceStates { STARTUP_DEVICE,
					FIND_GPS,
					PRE_IDLE,
					IDLE,
					PRE_RECORDING,
					RECORDING,
					CHARGING };

deviceStates state = STARTUP_DEVICE;

enum LEDStates { STARTUP_LEDS,
				 GREEN_SCAN,
				 FADE_TO_BLACK,
				 LED_IDLE,
				 SOLID_GREEN,
				 RAINBOW_SCAN,
				 LED_OFF };
LEDStates stripState = STARTUP_LEDS;

enum GPSStates { STARTUP_GPS,
				 FIND_LOCATION,
				 BACKGROUND_UPDATE,
				 POWERED_IDLE,
				 PRE_OFF,
				 GPS_OFF };
GPSStates gpsState = STARTUP_GPS;

// void appendLineToCSV();
// void readADC(adc1_channel_t, uint16_t *);
// void readAllAdcChannels();

void Sensors(void *parameter) {
	#define SENSOR_POWER 19

	#define BATTERY_PIN ADC1_CHANNEL_0	// SENVP, VBAT - 33k - BATTERY_PIN - 10k AOGND
	#define TEMP_OW_PIN 33				// GPIO where the DS18B20 Onewire Temp Pin is connected to

	#define TDS_PIN ADC1_CHANNEL_6
	#define PH_PIN ADC1_CHANNEL_7
	#define NITRATE_PIN ADC1_CHANNEL_4

	// Setup GPIO ==================================================
	pinMode(SENSOR_POWER, OUTPUT);
	digitalWrite(SENSOR_POWER, HIGH);



	vTaskDelete(NULL);
}

void adcSensors(void *parameter) {
	#define BATTERY_PIN ADC1_CHANNEL_0	// SENVP, VBAT - 33k - BATTERY_PIN - 10k AOGND
	#define TDS_PIN ADC1_CHANNEL_6
	#define PH_PIN ADC1_CHANNEL_7
	#define NITRATE_PIN ADC1_CHANNEL_4

	// Setup Analog Input ==================================================
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) != ESP_OK) {
		ESP_LOGE("ADC", "Vref Factory Setting: NOT Found");
	}


	adc1_config_channel_atten(PH_PIN,ADC_ATTEN_DB_11);
	adc1_config_channel_atten(TDS_PIN,ADC_ATTEN_DB_11);
	adc1_config_channel_atten(NITRATE_PIN,ADC_ATTEN_DB_11);
	adc1_config_channel_atten(BATTERY_PIN,ADC_ATTEN_DB_11);

	// esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
	// adc1_config_width(ADC_WIDTH_BIT_12);

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


	const u_int8_t scanInterval = 20;
	const u_int8_t updateInterval = 100;
	const u_int16_t idleInterval = 500;

	HardwareSerial uart(1);
	TinyGPSPlus gps;

	while (true) {
		switch (gpsState) {
			case STARTUP_GPS:
				pinMode(GPS_POWER, OUTPUT);
				digitalWrite(GPS_POWER, HIGH);
				uart.begin(9600, SERIAL_8N1, 16, 17);  // Setup GPS Serial Coms
				gpsState = FIND_LOCATION;
				break;

			case FIND_LOCATION:
				while(!gps.location.isValid()){
				    if (uart.available() > 0){
				        gps.encode(uart.read());    // get the byte data from the GPS
				    }
					vTaskDelay(scanInterval / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
				}

				Serial.print(gps.location.lat(), 6);  // 6dp
				Serial.print(F(","));
				Serial.println(gps.location.lng(), 6);
				gpsState = BACKGROUND_UPDATE;
				break;

			case BACKGROUND_UPDATE:
				if (uart.available() > 0) {
					gps.encode(uart.read());  // get the byte data from the GPS
				}
				vTaskDelay(updateInterval / portTICK_PERIOD_MS);  // vTaskDelay wants ticks, not milliseconds
				break;

			case POWERED_IDLE:
				vTaskDelay(idleInterval / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
				break;

			case PRE_OFF:
				digitalWrite(GPS_POWER, LOW);
				uart.end();
				gpsState = GPS_OFF;
				break;

			case GPS_OFF:
				vTaskDelay(idleInterval / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
				break;

			default:
				ESP_LOGE("GPS", "hit default -> gpsState = STARTUP_GPS");
				gpsState = STARTUP_GPS;
				break;
		}
	}
}

void ARGBLEDs(void *parameter) {
	#define LED_POWER 2
	#define ARGB1_PIN 13
	#define ARGB1_PIXELS 15
	#define colorSaturation 255	 // saturation of color constants
	RgbwColor green(0, colorSaturation, 0, 0);
	RgbwColor red(colorSaturation, 0, 0, 0);
	RgbwColor black(0, 0, 0, 0);
	const uint8_t FrameTime = 40;  // Milliseconds 40 = 25fps
	const uint8_t darkenBy = 32;
	int8_t moveDir = -1;  // current direction of led spot
	uint8_t pixel = 0;	  // current position of led spot
	RgbwColor spotColor;
	bool stripEnable = false;
	RgbwColor color;

	NeoPixelBrightnessBus<NeoGrbwFeature, Neo800KbpsMethod>strip(ARGB1_PIXELS, ARGB1_PIN);

	while (true) {
		switch (stripState) {
			case STARTUP_LEDS:	// Find What State I should be in
				ESP_LOGV("LED Strip", "stripState = STARTUP_LEDS");
				if (stripEnable == false) {
					// Setup ARGB strip ==================================================
					pinMode(LED_POWER, OUTPUT);
					strip.Begin();
					digitalWrite(LED_POWER, HIGH);
					strip.Show();
					stripEnable = true;
				}
				stripState = GREEN_SCAN;
				break;

			case GREEN_SCAN:
			for (uint16_t indexPixel = 0; indexPixel < ARGB1_PIXELS; indexPixel++) {
					color = strip.GetPixelColor(indexPixel);
					color.Darken(darkenBy);
					strip.SetPixelColor(indexPixel, color);
				}

				if (pixel == ARGB1_PIXELS || pixel == 0) {
					moveDir = -moveDir;
				}

				pixel += moveDir;
				strip.SetPixelColor(pixel, green);
				strip.Show();
				vTaskDelay(FrameTime / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds

				break;

			case FADE_TO_BLACK:
				ESP_LOGV("LED Strip", "stripState = FADE_TO_BLACK");
				for (size_t i = 0; i < 255; i++){
					for (uint16_t indexPixel = 0; indexPixel < ARGB1_PIXELS; indexPixel++) {
						color = strip.GetPixelColor(indexPixel);
						color.Darken(darkenBy);
						strip.SetPixelColor(indexPixel, color);
					}
					strip.Show();
					vTaskDelay(FrameTime / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
				}
				stripState = LED_IDLE;
				break;

			case SOLID_GREEN:
				ESP_LOGV("LED Strip", "stripState = SOLID_GREEN");
				for (uint16_t indexPixel = 0; indexPixel < ARGB1_PIXELS; indexPixel++) {
					strip.SetPixelColor(indexPixel, green);
					vTaskDelay(FrameTime / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
				}
				stripState = LED_IDLE;

				break;

			case RAINBOW_SCAN:
				for (uint16_t indexPixel = 0; indexPixel < ARGB1_PIXELS; indexPixel++) {
						color = strip.GetPixelColor(indexPixel);
						color.Darken(darkenBy);
						strip.SetPixelColor(indexPixel, color);
				}

				if (pixel == ARGB1_PIXELS || pixel == 0) {
						moveDir = -moveDir;
				}

				pixel += moveDir;
				strip.SetPixelColor(pixel, red);
				strip.Show();
				vTaskDelay(FrameTime / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
			break;

			case LED_IDLE:
			vTaskDelay(FrameTime * 10 / portTICK_PERIOD_MS);  // vTaskDelay wants ticks, not milliseconds
			break;

			case LED_OFF:
			ESP_LOGV("LED Strip", "stripState = LED_OFF");
			for (uint16_t indexPixel = 0; indexPixel < ARGB1_PIXELS; indexPixel++) {
						strip.SetPixelColor(indexPixel, black);
			}
			strip.Show();
			vTaskDelay(FrameTime / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
			digitalWrite(LED_POWER, LOW);
			stripEnable = false;
			stripState = LED_IDLE;
			break;

			default:
				ESP_LOGE("LED Strip", "hit default -> stripState = STARTUP_LEDS");
				stripState = STARTUP_LEDS;
				break;
		}
	}
}

void turbidity(void *parameter) {
	#define ARGB2_TURBIDITY_PIN 23
	#define ARGB2_TURBIDITY_PIXELS 2
	// Adafruit_NeoPixel ARGB2 = Adafruit_NeoPixel(ARGB2_TURBIDITY_PIXELS, ARGB2_TURBIDITY_PIN, NEO_GRBW + NEO_KHZ800);

	// ARGB2.begin();
	// ARGB2.setBrightness(255);
	// ARGB2.fill(ARGB2.Color(0, 0, 0, 0));
	// ARGB2.show();

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
		Serial.flush();
	#endif

	if (!SPIFFS.begin(true)) {	// Initialize SPIFFS (ESP32 SPI Flash Storage)
		ESP_LOGE("File System Error", "Can't mount SPIFFS");
	}

	// 			Function, Name (for debugging), Stack size, Params, Priority, Handle
	xTaskCreate(accessPoint, "accessPoint", 5000, NULL, 1, NULL);
	xTaskCreate(ARGBLEDs, "ARGBLEDs", 5000, NULL, 1, NULL);
	xTaskCreate(GPS, "GPS", 5000, NULL, 1, NULL);

	
}

void loop() {
	vTaskDelay(10 / portTICK_PERIOD_MS);	// Keep RTOS Happy when nothing is here
	switch (state) {
		case STARTUP_DEVICE:  // Find What State I should be in
			Serial.println("case STARTUP");
			state = FIND_GPS;
			break;

		case FIND_GPS:
			Serial.println("case FIND_GPS");
			stripState = GREEN_SCAN;
			while (gpsState == FIND_LOCATION) {
				vTaskDelay(100 / portTICK_PERIOD_MS);	// Keep RTOS Happy when nothing is here
			}



			state = PRE_IDLE;
			break;

		case PRE_IDLE:
			Serial.println("case PRE_IDLE");
			stripState = FADE_TO_BLACK;
			state = IDLE;
			break;

		case IDLE:
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
