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
#include <TinyGPS++.h>

// Analog Input
#include <driver/adc.h>
#include <esp_adc_cal.h>

// Adressable LEDs
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>

// Onewire Temp Sensor
#include <DallasTemperature.h>
#include <OneWire.h>

const char *ssid = "captive";  // FYI The SSID can't have a space in it.
const char *password = "12345678";
const IPAddress localIP(4, 3, 2, 1);					// the IP address the webserver, Samsung requires the IP to be in public space
const IPAddress gatewayIP(4, 3, 2, 1);					// IP address of the network
const String localIPURL = "http://4.3.2.1/index.html";	// URL to the webserver

char LogFilename[] = "/Water_Quality_Data.csv";

#define RECORDING_TIME 3000	 // time to record in milliseconds

// State Machine
enum deviceStates { STARTUP_DEVICE,
					FIND_GPS,
					PRE_IDLE,
					IDLE,
					PRE_RECORDING,
					RECORDING,
					POST_RECORDING,
					CHARGING };
deviceStates state = STARTUP_DEVICE;

enum LEDStates { STARTUP_LEDS,
				 FADE_TO_OFF,
				 LED_ANIMATION_UPDATE,
				 STARTUP_FADE_IN,
				 LED_OFF,
				 LED_IDLE };
LEDStates stripState = STARTUP_LEDS;

enum GPSStates { STARTUP_GPS,
				 FIND_LOCATION,
				 BACKGROUND_UPDATE,
				 POWERED_IDLE,
				 PRE_OFF,
				 GPS_OFF,
				 RESTART_GPS };
GPSStates gpsState = STARTUP_GPS;

// bool recording = false;
uint16_t tdsValue = 0xFFFF;
uint16_t phValue = 0xFFFF;
uint16_t nitrateValue = 0xFFFF;
u_int32_t waterTempValue = 0xFFFF;

char csvLine[256];


const uint16_t PixelCount = 15;	 // make sure to set this to the number of pixels in your strip
const uint8_t PixelPin = 13;	 // make sure to set this to the correct pin

NeoPixelBus<NeoGrbwFeature, NeoSk6812Method> strip(PixelCount, PixelPin);

NeoPixelAnimator animations(PixelCount, NEO_MILLISECONDS);
// NEO_MILLISECONDS        1    // ~65 seconds max duration, ms updates
// NEO_CENTISECONDS       10    // ~10.9 minutes max duration, 10ms updates
// NEO_DECISECONDS       100    // ~1.8 hours max duration, 100ms updates

// what is stored for state is specific to the need, in this case, the colors.
// basically what ever you need inside the animation update function
struct MyAnimationState {
	RgbwColor StartingColor;
	RgbwColor EndingColor;
};

MyAnimationState animationState[1];	 // we only need one as all the pixels are animated at once

void readADC(adc1_channel_t channel, uint16_t *value, uint8_t ALPHA_SMOOTHING, uint8_t ALPHA_SMOOTHING_DIVISOR, esp_adc_cal_characteristics_t *adc1_chars) {
	uint16_t input;
	input = esp_adc_cal_raw_to_voltage(adc1_get_raw(channel), adc1_chars);
	if (*value & 0xFFFF) {	// bitwise and operation = fast way to check if *value == 0xFFFF
		*value = input;
	} else {
		*value = (input * ALPHA_SMOOTHING + (ALPHA_SMOOTHING_DIVISOR - ALPHA_SMOOTHING) * *value) / ALPHA_SMOOTHING_DIVISOR;
	}
}

void adcSensors(void *parameter) {
#define BATTERY_PIN ADC1_CHANNEL_0	// SENVP, VBAT - 33k - BATTERY_PIN - 10k AOGND
#define TDS_PIN ADC1_CHANNEL_6
#define PH_PIN ADC1_CHANNEL_7
#define NITRATE_PIN ADC1_CHANNEL_4

#define ALPHA_SMOOTHING 1
#define ALPHA_SMOOTHING_DIVISOR 100

	esp_adc_cal_characteristics_t adc1_chars;

	// Setup Analog Input ==================================================
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) != ESP_OK) {
		ESP_LOGE("ADC", "Vref Factory Setting: NOT Found");
	}

	adc1_config_channel_atten(PH_PIN, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(TDS_PIN, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(NITRATE_PIN, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(BATTERY_PIN, ADC_ATTEN_DB_11);

	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
	adc1_config_width(ADC_WIDTH_BIT_12);

	while (true) {
		readADC(TDS_PIN, &tdsValue, ALPHA_SMOOTHING, ALPHA_SMOOTHING_DIVISOR, &adc1_chars);
		if (state == RECORDING) {
			readADC(PH_PIN, &phValue, ALPHA_SMOOTHING, ALPHA_SMOOTHING_DIVISOR, &adc1_chars);
			readADC(NITRATE_PIN, &nitrateValue, ALPHA_SMOOTHING, ALPHA_SMOOTHING_DIVISOR, &adc1_chars);
			vTaskDelay(10 / portTICK_PERIOD_MS);  // Keep RTOS Happy when there is nothing to do
		} else {
			vTaskDelay(100 / portTICK_PERIOD_MS);  // Keep RTOS Happy when there is nothing to do
		}
	}

	vTaskDelete(NULL);
}

void tempSensor(void *parameter) {
#define ONE_WIRE_BUS 33	 // GPIO where the DS18B20 Onewire Temp Pin is connected to
#define RESOLUTION 12	 // 12 Bit temperatures
#define WAIT_FOR_DATA 750 / (1 << (12 - RESOLUTION))

	vTaskDelay(200 / portTICK_PERIOD_MS);  // allow time for boot of sensor

	// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
	OneWire oneWire(ONE_WIRE_BUS);

	// Pass our oneWire reference to Dallas Temperature.
	DallasTemperature sensors(&oneWire);

	// arrays to hold device address
	DeviceAddress insideThermometer;

	sensors.begin();

	// Search for devices on the bus and assign based on an index
	if (!sensors.getAddress(insideThermometer, 0)) ESP_LOGE("Onewire Temp Sensor", "Unable to find address for device index 0");

	if (!sensors.isConnected(insideThermometer)) ESP_LOGE("Onewire Temp Sensor", "Sensor Not Connected");

	// set the resolution to XX bit (Each Dallas/Maxim device is capable of several different resolutions)
	sensors.setResolution(insideThermometer, RESOLUTION);

	// Async reading of Dallas Temperature Sensors
	sensors.setWaitForConversion(false);

	// set the resolution of the temperature readings
	if (sensors.getResolution(insideThermometer) != RESOLUTION) {
		ESP_LOGE("Onewire Temp Sensor", "Unable to set Resolution of readings");
	}

	while (true) {
		if (state == RECORDING) {
			sensors.requestTemperaturesByAddress(insideThermometer);
			// sensors.requestTemperatures();	// Send the command to get temperatures
			vTaskDelay(WAIT_FOR_DATA / portTICK_PERIOD_MS);

			u_int32_t input = sensors.getTemp(insideThermometer);

			// Check if reading was successful
			if (input != DEVICE_DISCONNECTED_C) {
				waterTempValue = input;
			} else {
				ESP_LOGE("Onewire Temp Sensor", "Could not read temperature data: DEVICE_DISCONNECTED_C");
			}

			// vTaskDelay(10 / portTICK_PERIOD_MS);  // Keep RTOS Happy when there is nothing to do
		} else {
			vTaskDelay(500 / portTICK_PERIOD_MS);  // Keep RTOS Happy when there is nothing to do
		}
	}
}

void sensors(void *parameter) {
#define SENSOR_POWER 19

	// Setup GPIO ==================================================
	pinMode(SENSOR_POWER, OUTPUT);
	digitalWrite(SENSOR_POWER, HIGH);

	xTaskCreate(adcSensors, "adcSensors", 5000, NULL, 1, NULL);
	xTaskCreate(tempSensor, "tempSensor", 5000, NULL, 1, NULL);

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
	#define GPS_POWER 18

	const u_int8_t scanInterval = 50;
	const u_int16_t updateInterval = 250;
	const u_int16_t idleInterval = 500;

	u_int32_t gpsCycles = 0;

	TinyGPSPlus gps;

	bool gpsErr = false;
	pinMode(GPS_POWER, OUTPUT);

	while (true) {
		switch (gpsState) {
			case STARTUP_GPS:
				ESP_LOGI("GPS", "STARTUP_GPS");

				digitalWrite(GPS_POWER, HIGH);

				gpsErr = false;
				gpsCycles = 0;

				Serial2.setRxBufferSize(1024);	// increase buffer from 256 -> 1024 so we don't end up with buffer overflows
				Serial2.begin(9600);

				gpsState = FIND_LOCATION;
				break;

			case FIND_LOCATION:
				ESP_LOGI("GPS", "FIND_LOCATION");
				while (!gps.location.isValid() && gpsErr == false) {
					while (Serial2.available() > 0) {  // any data waiting?
						gps.encode(Serial2.read());	   // get the data from the Serial Buffer
					}

					if (gpsCycles > 100 && gps.charsProcessed() < 10) {
						ESP_LOGE("GPS", "not getting any GPS data (no chars processed), check wiring");
						gpsErr = true;
					}

					if (gps.failedChecksum() > 0) {
						ESP_LOGW("GPS", "Failed Checksum, could be wiring or buffer overflow...");
						gpsErr = true;
					}

					gpsCycles++;
					vTaskDelay(scanInterval / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
				}

				if (gpsErr == false) {
					ESP_LOGI("GPS", "Location Found in ~%is", ((scanInterval * gpsCycles) / 1000));
					gpsState = PRE_OFF;
				} else {
					gpsState = RESTART_GPS;
				}

				break;

			case BACKGROUND_UPDATE:
				while (Serial2.available() > 0) {  // any data waiting?
					gps.encode(Serial2.read());	   // get the data from the Serial Buffer
				}

				vTaskDelay(updateInterval / portTICK_PERIOD_MS);  // vTaskDelay wants ticks, not milliseconds
				break;

			case PRE_OFF:
				digitalWrite(GPS_POWER, LOW);
				Serial2.end();
				gpsState = GPS_OFF;
				break;

			case GPS_OFF:
				vTaskDelay(idleInterval / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
				break;

			case RESTART_GPS:
				digitalWrite(GPS_POWER, LOW);
				Serial2.end();
				vTaskDelay(500 / portTICK_PERIOD_MS);  // vTaskDelay wants ticks, not milliseconds
				gpsState = STARTUP_GPS;
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
	const uint8_t FrameTime = 40;  // Milliseconds 40 = 25fps


	RgbwColor fadeInColour = HslColor(0.5f, 1.0f, 0.4f); //this is a light blue

	LEDStates nextState = STARTUP_FADE_IN;
	stripState = STARTUP_LEDS;

	// //Random Seed Generation
	// uint32_t seed;
	// // random works best with a seed that can use 31 bits
	// // analogRead on a unconnected pin tends toward less than four bits
	// seed = analogRead(39);
	// vTaskDelay(10 / portTICK_PERIOD_MS);
	// for (int shifts = 3; shifts < 31; shifts += 3) {
	// 	seed ^= analogRead(39) << shifts;
	// 	delay(1);
	// }
	// // Serial.println(seed);
	// randomSeed(seed);

	void BlendAnimUpdate(const AnimationParam &param);


	while (true) {
		switch (stripState) {
			case STARTUP_LEDS:
				ESP_LOGI("LED Strip", "STARTUP_LEDS");
				pinMode(LED_POWER, OUTPUT);
				strip.Begin();
				digitalWrite(LED_POWER, HIGH);

				for (int i = 0; i < 10; ++i) {
					strip.Show();
					vTaskDelay(10 / portTICK_PERIOD_MS);
				}

				stripState = nextState;
				break;

			case STARTUP_FADE_IN:
				animationState[0].StartingColor = strip.GetPixelColor(0);
				animationState[0].EndingColor = fadeInColour;
				animations.StartAnimation(0, 500, BlendAnimUpdate);

				stripState = LED_ANIMATION_UPDATE;
				nextState = FADE_TO_OFF;
				break;

			case FADE_TO_OFF:
				animationState[0].StartingColor = strip.GetPixelColor(0);
				animationState[0].EndingColor = RgbwColor(0);  // black
				animations.StartAnimation(0, 2000, BlendAnimUpdate);
				stripState = LED_ANIMATION_UPDATE;
				nextState = LED_OFF;
				break;

			case LED_ANIMATION_UPDATE:
				if (animations.IsAnimating()) {
					animations.UpdateAnimations();
					strip.Show();
					vTaskDelay(FrameTime / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
				} else {
					stripState = nextState;
				}
				break;

			case LED_OFF:
				strip.ClearTo(0); //set to black
				strip.Show();
				vTaskDelay(FrameTime / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
				digitalWrite(LED_POWER, LOW);
				stripState = LED_IDLE;
				break;

			case LED_IDLE:
				vTaskDelay(500 / portTICK_PERIOD_MS);	 // vTaskDelay wants ticks, not milliseconds
				break;
			

			default:
				ESP_LOGE("LED Strip", "hit default -> stripState = STARTUP_LEDS");
				stripState = STARTUP_LEDS;
			break;
			
		}
	}
}

// simple blend function
void BlendAnimUpdate(const AnimationParam &param) {
	// this gets called for each animation on every time step
	// progress will start at 0.0 and end at 1.0
	// we use the blend function on the RgbColor to mix
	// color based on the progress given to us in the animation
	RgbwColor updatedColor = RgbwColor::LinearBlend(
		animationState[param.index].StartingColor,
		animationState[param.index].EndingColor,
		param.progress);

	// apply the color to the strip
	for (uint16_t pixel = 0; pixel < PixelCount; pixel++) {
		strip.SetPixelColor(pixel, updatedColor);
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

	// ESP_LOGI("WebServer", "Startup complete");

	while (true) {
		dnsServer.processNextRequest();
		vTaskDelay(DNS_INTERVAL / portTICK_PERIOD_MS);
	}
}

/*void appendLineToCSV(void *parameter) {
	// UTC_Date(YYYY-MM-DD),UTC_Time(HH:MM:SS),Latitude(Decimal),Longitude(Decimal),Altitude(Meters),Water Temperature(Deg C), TDS (PPM), pH (0-14), Nitrate (PPM TDS)

	//-----Date from GPS-----------------------------------
	char datef[32];
	if (!gps.date.isValid()) {
		strcpy(datef, "****-**-**");
		ESP_LOGW("GPSDate", "not valid");
	} else {
		sprintf(datef, "%02d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
	}
	//------------------------------------------------------

	//-----Time from GPS-----
	char timef[32];
	if (!gps.time.isValid()) {
		strcpy(timef, "**:**:**");
		ESP_LOGW("GPSTime", "not valid");
	} else {
		sprintf(timef, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
	}
	//------------------------------------------------------

	//-----Lat and Long from GPS-----
	char latlngf[32];
	if (!gps.location.isValid()) {
		strcpy(latlngf, "***.******,***.******");
		ESP_LOGW("GPSLatLong", "not valid");
	} else {
		sprintf(latlngf, "%f,%f", gps.location.lat(), gps.location.lng());
		//dtostrf(gps.altitude.meters(), -7, 6, altf);  // 7 characters with 6 decimal place and left aligned (negative width)
	}
	//------------------------------------------------------

	//-----Altitude from GPS--------------------------------
	char altf[8];
	if (!gps.altitude.isValid()) {
		strcpy(altf, "***");
		ESP_LOGW("GPSAltitude", "not valid");
	} else {
		//sprintf(altf, "%f", gps.altitude.meters());
		dtostrf(gps.altitude.meters(), -7, 0, altf);  // 7 characters with 0 decimal place and left aligned (negative width)
	}
	//------------------------------------------------------

	//-----Water Temperature -------------------------------
Serial.println(sensors.rawToCelsius(waterTempValue),6);
	float waterTempValue;
	char tempf[8];
	if (waterTempValue <= 0 || waterTempValue > 50) {  // 141 is a disconnected sensor
		strcpy(tempf, "**");
		ESP_LOGW("waterTempSensor", "out of bounds");
	} else {
		//sprintf(tempf, "%f", waterTempValue);
		dtostrf(waterTempValue, -7, 2, tempf);	// 7 characters with 2 decimal place and left aligned (negative width)
	}
	//------------------------------------------------------

	//-----TDS Conversion from Raw to PPM -----
	// Regression Calibration of pH y=a+bx+cx^2
	float tdsa = -54.0;
	float tdsb = 0.368;
	float tdsc = 0.0000666;

	char tdsf[8] = "";

	if (tdsValue <= 142) {	// 141 is a disconnected sensor
		strcpy(tdsf, "***");
		ESP_LOGW("tdsSensor", "tds <= 142, out of bounds");
	} else {
		// Cubic Regression Calibration y=a+bx+cx^2+dx^3
		float tdsOutput = tdsa + tdsb * (tdsValue) + tdsc * pow(tdsValue, 2);
		dtostrf(tdsOutput, -7, 1, tdsf);	 // 7 characters with 1 decimal place and left aligned (negative width)
	}
	//------------------------------------------------------

	//-----pH Conversion from Raw to 0-14 ------------------
	// Regression Calibration of pH y=a+bx+cx^2
	float pha = 32.8;
	float phb = -0.0151;
	float phc = 0.00000142;
	// Regression Calibration of pH vs Temp y=a+bx
	float phta = -0.713;
	float phtb = 0.000293;

	char phf[8];

	// Regression Calibration y=a+bx+cx^2
	float phNoTempCal = pha + phb * (phValue) + phc * pow(phValue, 2);
	// Regression Calibration y=a+bx
	float phOutput = phNoTempCal + phta + phtb * (waterTempValue);

	if (phOutput > 13 || phOutput < 1) {
		strcpy(phf, "***");
		ESP_LOGW("phSensor", "ph, out of bounds");
	} else {
		//sprintf(phf, "%f", phOutput);
		dtostrf(phOutput, -7, 2, phf);	// 7 characters with 2 decimal place and left aligned (negative width)
	}
	//------------------------------------------------------

	// Cubic Regression Calibration y=a+bx+cx^2+dx^3
	float nitratea = 0;
	float nitrateb = 0;
	float nitratec = 0;
	float nitrated = 0;

	char csvLine[256];
	// UTC_Date(YYYY-MM-DD),UTC_Time(HH:MM:SS),Latitude(Decimal),Longitude(Decimal),Altitude(Meters),Water Temperature(Deg C), TDS (PPM TDS442), pH (0-14)
	sprintf(csvLine, "%s,%s,%s,%s,%s,%s,%s\r\n", datef, timef, latlngf, altf, tempf, tdsf, phf);
	Serial.println(csvLine);

	File CSV = SPIFFS.open(F(LogFilename), FILE_APPEND);
	if (!CSV) {
		ESP_LOGE("CSV File", "Error opening");
	} else {
		if (CSV.print(csvLine)) {
			ESP_LOGI("CSV File", "File append success");
			CSV.close();
		} else {
			ESP_LOGE("CSV File", "File append failed");
		}
	}
	// char sz[20] = "";
	// float val = -16.893;
	//sprintf(sz, "%f", dtostrf(val, -7, 1, buf));
	// dtostrf(val, -7, 1, sz);  // 7 characters with 1 decimal place and left aligned (negative width)
	// Serial.println(sz);
	vTaskDelete(NULL);
}*/

void logFreeHeap(void *parameter) {
	uint32_t freeHeap;

	while (true) {
		freeHeap = ESP.getFreeHeap();
		if (freeHeap < 100000) {
			ESP_LOGW("RTOS", "free bytes in the (data memory) heap is %i bytes", (ESP.getFreeHeap()));
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);  // vTaskDelay wants ticks, not milliseconds
	}
}


void setup() {
	Serial.begin(115200);
	while (!Serial)
		;
	ESP_LOGI("OSWQS", "Compiled " __DATE__ " " __TIME__ " by CD_FER");
	Serial.flush();
}

void loop() {
	switch (state) {
		case STARTUP_DEVICE:
			// ESP_LOGI("deviceState", "STARTUP_DEVICE");
			if (!SPIFFS.begin(true)) {	// Initialize SPIFFS (ESP32 SPI Flash Storage)
				ESP_LOGE("File System Error", "Can't mount SPIFFS");
			}

			// 			Function, Name (for debugging), Stack size, Params, Priority, Handle
			xTaskCreate(logFreeHeap, 	"logFreeHeap", 	5000, NULL, 1, NULL);
			xTaskCreate(accessPoint, 	"accessPoint", 	5000, NULL, 1, NULL);
			xTaskCreate(GPS, 			"GPS", 			5000, NULL, 1, NULL);
			xTaskCreate(debugLEDs, 		"debugLEDs", 	1000, NULL, 1, NULL);
			xTaskCreate(ARGBLEDs, 		"ARGBLEDs", 	5000, NULL, 1, NULL);
			xTaskCreate(sensors, 		"sensors", 		5000, NULL, 1, NULL);
			state = IDLE;
			break;

		case IDLE:
			// ESP_LOGI("deviceState", "IDLE");
			

			// while (tdsValue < 150 || tdsValue == 0xFFFF) {
			// 	vTaskDelay(100 / portTICK_PERIOD_MS);
				
			// }
			//state = PRE_RECORDING;
			break;

		case PRE_RECORDING:
			ESP_LOGI("deviceState", "PRE_RECORDING");
			xTaskCreate(sensors, "sensors", 5000, NULL, 1, NULL);
			// recording = true;
			state = RECORDING;
			break;

		case RECORDING:
			ESP_LOGI("deviceState", "RECORDING");
			vTaskDelay(RECORDING_TIME / portTICK_PERIOD_MS);
			state = POST_RECORDING;
			break;

		case POST_RECORDING:
			ESP_LOGI("deviceState", "POST_RECORDING");
			// recording = false;
			// xTaskCreate(appendLineToCSV, "appendLineToCSV", 5000, NULL, 1, NULL);
			state = IDLE;
			break;

		case CHARGING:
			break;

		default:
			ESP_LOGE("Hit default Case in state machine", "Restarting...");
			vTaskDelay(1000 / portTICK_PERIOD_MS);	// vTaskDelay wants ticks, not milliseconds
			ESP.restart();
			break;
	}
	vTaskDelay(1);	// Keep RTOS Happy with a 1 tick delay when there is nothing to do
}