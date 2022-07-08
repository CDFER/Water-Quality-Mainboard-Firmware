/*
 * Connect the following pins:
 *
 * ESP32 | SD CARD
 *    D5       CS
 *    VIN      5V
 *    D18      SCK
 *    D23      MOSI
 *    D19      MISO

 * ESP32 | SCREEN
 *    D22      SCK
 *    D21      SDA
 *    3v3      VDD
 */

#include <Arduino.h>

// Libs & settings for OLED/screen
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Libs for SD Card & logging
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Lib for GPS Module
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Global Variables
SSD1306AsciiWire oled;

char LogFilename[] = "/Water_Quality.csv";

SoftwareSerial ss(13, 12);
TinyGPSPlus gps;

char * getLastDateTime();
void logDateTime();
void logGPS();
void logTxt(const char * text);
void logLineToFile();
void logTxtln(char * text);
void logUInt(uint8_t num);
void logUIntln(uint8_t num);
void logFlt(float num);
void displayFlt(float num);
char* ftoa(double d, char *buffer, int precision);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);

//------------------------------------------------------------------------------
void setup() {
    // OLED Setup stuff
    Wire.begin();
    Wire.setClock(400000L);
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(System5x7);
    
    // Set auto scrolling at end of window.
    oled.setScrollMode(SCROLL_MODE_AUTO);
  
  
    // SD Card & logging Setup stuff
    if(!SD.begin(5)){
        oled.println("Card Mount Failed");
        return;
    }

    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        oled.println("No SD card attached");
        return;
    }

    oled.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        oled.println("MMC");
    } else if(cardType == CARD_SD){
        oled.println("SDSC");
    } else if(cardType == CARD_SDHC){
        oled.println("SDHC");
    } else {
        oled.println("UNKNOWN");
    }

    deleteFile(SD, LogFilename);
    writeFile(SD, LogFilename, "UTC_Date,UTC_Time,Latitude(Decimal),Longitude(Decimal)\n");

    oled.print("Total Space: ~");
    oled.print(SD.totalBytes() / (1024 * 1024 * 1024));
    oled.println("GB");

    oled.print("Used Space:  ~");
    oled.print(SD.usedBytes() / (1024 * 1024));
    oled.println("MB");
  

   // GPS Module Setup stuff
    ss.begin(9600);
  
}

//------------------------------------------------------------------------------
void loop() {

  while (ss.available() > 0){
    // get the byte data from the GPS
    gps.encode(ss.read());
  }

  if (gps.location.isUpdated()){
    // Log data to CSV file on SD Card
    logDateTime();
    logGPS();
    // ...
    logLineToFile();

    // Debugging: Display data to oled screen
    oled.clear(); 
    oled.println(getLastDateTime());
    oled.print("Lat: "); displayFlt(gps.location.lat()); 
    oled.print("Lng: "); displayFlt(gps.location.lng()); //*/
  }
}

//------------------------------------------------------------------------------
/* Logging stuff */
char logLine[200] = "";
char lastDateTime[20] = "";

char * getLastDateTime() { return lastDateTime; }

void logDateTime(){
  char buf[6];
  strcpy(lastDateTime, "");
  strcat(lastDateTime, itoa(gps.date.year(),buf,10)); strcat(lastDateTime, "-"); 
  strcat(lastDateTime, itoa(gps.date.month(),buf,10)); strcat(lastDateTime, "-"); 
  strcat(lastDateTime, itoa(gps.date.day(),buf,10)); strcat(lastDateTime, ","); 
  strcat(lastDateTime, itoa(gps.time.hour(),buf,10)); strcat(lastDateTime, ":"); 
  strcat(lastDateTime, itoa(gps.time.minute(),buf,10)); strcat(lastDateTime, ":"); 
  strcat(lastDateTime, itoa(gps.time.second(),buf,10)); 
  strcat(logLine, lastDateTime);
  strcat(logLine, ","); 
}

void logGPS(){
  logFlt(gps.location.lat()); logTxt(",");
  logFlt(gps.location.lng()); logTxt(",");
}

void logTxt(const char * text){
  strcat(logLine, text);
}

void logLineToFile(){
  File file = SD.open(LogFilename, FILE_APPEND);
  if(!file){
      oled.println("Failed to open file for appending");
      return;
  }
  file.println(logLine);
  file.close();

  strcpy(logLine,""); // reset the logLine buffer
}

void logTxtln(char * text){
  logTxt(text);
  logTxt("\n");
  logLineToFile();
}

void logUInt(uint8_t num){
  char buf[6];
  logTxt(itoa(num,buf,10));
}

void logUIntln(uint8_t num){
  char buf[6];
  logTxtln(itoa(num,buf,10));
}

void logFlt(float num){
  char buf[20];
  logTxt(ftoa(num,buf,6));
}

void displayFlt(float num){
  char buf[20];
  oled.println(ftoa(num,buf,6));
}

char* ftoa(double d, char *buffer, int precision) {

	long wholePart = (long) d;

	// Deposit the whole part of the number.

	itoa(wholePart,buffer,10);

	// Now work on the faction if we need one.

	if (precision > 0) {

		// We do, so locate the end of the string and insert
		// a decimal point.

		char *endOfString = buffer;
		while (*endOfString != '\0') endOfString++;
		*endOfString++ = '.';

		// Now work on the fraction, be sure to turn any negative
		// values positive.

		if (d < 0) {
			d *= -1;
			wholePart *= -1;
		}
		
		double fraction = d - wholePart;
		while (precision > 0) {

			// Multipleby ten and pull out the digit.

			fraction *= 10;
			wholePart = (long) fraction;
			*endOfString++ = '0' + wholePart;

			// Update the fraction and move on to the
			// next digit.

			fraction -= wholePart;
			precision--;
		}

		// Terminate the string.

		*endOfString = '\0';
	}

    return buffer;
}

/** 
 * SD card stuff
 */
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    oled.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        oled.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        oled.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            oled.print("  DIR : ");
            oled.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            oled.print("  FILE: ");
            oled.print(file.name());
            oled.print("  SIZE: ");
            oled.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    oled.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        oled.println("Dir created");
    } else {
        oled.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    oled.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        oled.println("Dir removed");
    } else {
        oled.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    oled.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        oled.println("Failed to open file for reading");
        return;
    }

    oled.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    oled.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        oled.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        oled.println("File written");
    } else {
        oled.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    oled.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        oled.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        oled.println("Message appended");
    } else {
        oled.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    oled.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        oled.println("File renamed");
    } else {
        oled.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    oled.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        oled.println("File deleted");
    } else {
        oled.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        oled.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        oled.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        oled.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    oled.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}