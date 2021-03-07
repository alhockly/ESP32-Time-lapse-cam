/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"

// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

int pictureNumber = 0;
bool SDMounted = false;

const char* ssid = "";  //WiFi SSID
const char* password = "";    //WiFi Password

AsyncWebServer server(80);

long numImages;
String latestFile;
const int FILE_LIST_SIZE = 255;
String files[FILE_LIST_SIZE];
long MAX_NUMBER_OF_FILES = 300; //Number of files to create before starting to delete old ones. This is a balance between speed of the /list endpoint
                                // and how far back the timelapse images on the SD go 


const unsigned long THIRTY_MINS = 30*60*1000UL;
const unsigned long TEN_MINS = 10*60*1000UL;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;
char lastcapture[20]; 

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <h2>ESP Image Web Server</h2>
  <img src="/latest">
  </body>  
</html>)rawliteral";

String listDir(fs::FS &fs, const char * dirname, uint8_t max_files){
    //sets string array to contain file list, updates number of images count, returns full list
    //Serial.printf("Listing directory: %s\n", dirname);
    String fileList = String("");
   
    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return "";
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return "";
    }

    File file = root.openNextFile();
    long fileCount = 0;
    while(file){
        esp_task_wdt_reset();
        
        if(max_files > 0 && fileCount > max_files){
          break;
        }
        if(file.isDirectory()){
            //Serial.print("  DIR : ");
            //Serial.println(file.name());
            
        } else {
            //Serial.print("  FILE: ");
            //Serial.print(file.name());
            fileList += file.name() + String("\n");
            if(fileCount<FILE_LIST_SIZE){
              files[fileCount] = file.name();
            }
            fileCount += 1;
            //Serial.print("  SIZE: ");
            //Serial.println(file.size());
        }

        
        file = root.openNextFile();
    }
    numImages = fileCount;
    //Serial.println(fileList);
    // for(int i=0;i<FILE_LIST_SIZE;i++){
    //   Serial.println(files[i]);
    // }
    return fileList;
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void refreshNetworkTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  strftime(lastcapture, 20, "%F_%H-%M-%S", &timeinfo);
  //Serial.println(lastcapture);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  //Serial.println();
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  delay(200);
  if(psramFound()){
    Serial.println("\npsram enabled");
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(2000);
    ESP.restart();
  }

  sensor_t * s = esp_camera_sensor_get();
  s -> set_lenc(s, 1);
  
  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    delay(2000);
    ESP.restart();
  } else {
    Serial.println("SD card mounted");
    SDMounted = true;
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
  Serial.println(("Listing files"));
  listDir(SD_MMC, "/", 0);
  
  Serial.println("Connecting to Wifi");
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(2000);
    WiFi.begin(ssid, password);
  }

  Serial.println("");
  Serial.print("Connected to: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
 
  
  

server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
  
  Serial.print("Recieved /download request from ");
  Serial.println(request->client()->remoteIP());

  if(SDMounted){

    int paramsNr = request->params();
    for(int i=0;i<paramsNr;i++){
 
      AsyncWebParameter* p = request->getParam(i);
      String paramName = p->name();
      String paramValue = p->value();
      if(paramName=="file"){
        File file = SD_MMC.open(paramValue, FILE_READ);
        if(!file){
          Serial.println(" Failed to open file for reading");
          request->send(404, "text/plain", "File not found");
          return;
        }else{
          request->send(file, paramValue, "text/xhr", true);
        }
      }
    }
    
  } else {
    Serial.println("SD not mounted");
    request->send(404, "text/plain", "SD card not mounted");
  }

 });
 

server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request){
  Serial.print("/list from ");
  Serial.println(request->client()->remoteIP());
  if(SDMounted){
    
    String filesList = listDir(SD_MMC, "/", 0);
    File file = SD_MMC.open(latestFile, FILE_READ);
    if(!file){
      Serial.println(" Failed to open file for reading");
      return;
    }
    request->send(200, "text/plain", filesList);
    Serial.print(numImages);
    Serial.println(" images found");
    
  } else {
    Serial.println("SD not mounted");
    request->send(200, "text/plain", "SD not mounted");
  }

 });

server.on("/latest", HTTP_GET, [](AsyncWebServerRequest *request){
  
  if(SDMounted){
    File file = SD_MMC.open(latestFile, FILE_READ);
    if(!file){
      Serial.println(" Failed to open file for reading");
      request->send(404, "text/plain", "File not found");
      return;
    }else{
      request->send(file, latestFile, "text/xhr", true);
    }
   
  } else {
    Serial.println("SD not mounted");
    request->send(200, "text/plain", "SD not mounted");
  }
 });


 server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  
  if(SDMounted){
    request->send_P(200, "text/html", index_html);
  } else {
    Serial.println("SD not mounted");
  }

 });

  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request){
    refreshNetworkTime();
    request->send_P(200, "text/html", lastcapture);
  
 });


  server.begin();
}






void takePicAndSave(){
  camera_fb_t * fb = NULL;
  refreshNetworkTime();

  listDir(SD_MMC, "/", 0);
  if(numImages > MAX_NUMBER_OF_FILES){
    Serial.print(numImages);
    Serial.println(" images found. Deleting oldest file");
    deleteFile(SD_MMC, files[0].c_str());
  }
   
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Path where new picture will be saved in SD Card
  String path = "/" + String(lastcapture) +".jpg";
  

  //Set fs to use SD card
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    latestFile = path;
    
  }
  file.close();
  esp_camera_fb_return(fb); 

}

void loop() {
  if(SDMounted){
    takePicAndSave();
    delay(THIRTY_MINS);
  }
  
}