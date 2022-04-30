//Picture capturing function taken from
//Rui Santos --  https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card

/*********
In order to have operation of GPIO 4 (to control dehumidifier) the SD card is only mounted when needed and then unmounted after all file operations have been completed
Similiarly the sensors are only mounted when needed.
This has the advantage of making these parts hotswapable. Camera is not hotswappable lol

  Tips for flashing
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
   - You may need to supply power to ESP board seperately 
   - try to keep the same serial connection throughout flashing/ restarting


*********/
//lots of libraries
#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"      //to feed the task watchdog
#include <vector>
#include <algorithm>
#include "WifiCredentials.h"  
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <SPI.h>
#include "SparkFunBME280.h" //edit this library to change the i2c address if bme is not found
#include <SparkFun_SGP30_Arduino_Library.h>
#include <PubSubClient.h>



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

//global objects
WiFiMulti wifiMulti;
AsyncWebServer server(80);
std::vector<String> files;

//global vals
//file
long numImages;
String latestFile = "";   //these are different lol
char lastcapture[20];     
long MAX_NUMBER_OF_FILES = 300; //Number of files to create before starting to delete old ones. This is a balance between speed of the /list endpoint
                                // and how far back the timelapse images on the SD go 
//hardware
bool cameraMounted = false;
bool SDmounted = false;
bool bmeMounted = false;
bool sgpMounted = false;
bool dehumidiferState = false;
int dehumidifierControlPin = 4;

BME280 bme280;
SGP30 sgp30;

TwoWire tw = TwoWire(0);

//environ vals
float temp =0;
float humidity =0;
float co2 =0;
float tvoc =0;
int upperHumidityBound = 70;
int lowerHumidityBound = 50;
int sensorInterval = 1000;
bool pauseSensorReading = false;    //This is used as a flag to indicate sd mount needed
int sensorPauseTime = 20000;
//time
const unsigned long THIRTY_MINS = 30*60*1000UL;
const unsigned long TEN_MINS = 10*60*1000UL;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

// Add your MQTT Broker IP address, example:
const char* mqtt_server = MQTTURL;
WiFiClient espClient;
PubSubClient mqttclient(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


const char start_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Grow box</title>
  <script src='https://code.jquery.com/jquery-2.2.4.js'></script>
</head>
<body>
  <p>Dehumidifier settings</p>
  <input type='number' id='lower' placeholder='lower humidity bound'></input><br><button id='setLower'>Set lower</button><br>
  <br><input type='number' id='higher' placeholder='higher humidity bound'></input><br><button id ='setHigher'>Set higher</button>
  <p id="currentvals"></p>
  <img src="/latest" width="400px">
  <script>
              function setLowerBound(){
                var oReq = new XMLHttpRequest();
                oReq.open("GET", "/lowBound?int="+document.getElementById("lower").value);
                oReq.send();
              }  

              function setHigherBound(){
                var oReq = new XMLHttpRequest();
                oReq.open("GET", "/highBound?int="+document.getElementById("higher").value);
                oReq.send();
              } 
              document.getElementById("setLower").onclick = function() {setLowerBound()};
              document.getElementById("setHigher").onclick = function() {setHigherBound()};


              function getEnviron(){
                      var oReq = new XMLHttpRequest();
                      oReq.addEventListener("load", environDisplay);
                      oReq.open("GET", "/environ");
                      oReq.send();
               }
              function environDisplay(){
                console.log(this.responseText)
                 document.getElementById('currentvals').innerHTML = this.responseText;
              }

              function sleep(milliseconds) {
                var start = new Date().getTime();
                for (var i = 0; i < 1e7; i++) {
                  if ((new Date().getTime() - start) > milliseconds){
                    break;
                  }
                }
              }

              sleep(2000);
              setInterval(getEnviron, 5000);
              getEnviron();
    </script>
  </body>  
</html>)rawliteral";

//Function for debug
void scanForWifi(){
  Serial.print("wifi Scan start ... ");
  int n = WiFi.scanNetworks();
  Serial.print(n);
  Serial.println(" network(s) found");
  for (int i = 0; i < n; i++)
  {
    Serial.println(WiFi.SSID(i));
    Serial.println(WiFi.RSSI(i));
  }
  Serial.println();
}

void listDir(fs::FS &fs, const char * dirname, uint8_t max_files){
  //sets vector array to contain file list, updates number of images count
  //Serial.printf("Listing directory: %s\n", dirname);
  pauseSensorReading = true;
  files.clear();
  File root = fs.open(dirname);
  if(!root){
      Serial.println("Failed to open directory");
      return;
  }
  if(!root.isDirectory()){
      Serial.println("Not a directory");
      return;
  }

  File file = root.openNextFile();
  long fileCount = 0;
  while(file){
    esp_task_wdt_reset();
    
    if(max_files > 0 && fileCount > max_files){
      break;
    }
    if(!file.isDirectory()){
        
        files.push_back(file.name());
        fileCount += 1;
        //Serial.print("  SIZE: ");
        //Serial.println(file.size());
    }
    
    file = root.openNextFile();
  }
  std::sort(files.begin(), files.end());
  // for (int i = 0; i < files.size(); i++){
  //     Serial.println(files[i]);
  // }
  esp_task_wdt_reset();

  numImages = fileCount;
  
  return;
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

bool refreshNetworkTime(bool setLastCapture){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return false;
  }
  if(setLastCapture){
    strftime(lastcapture, 20, "%F_%H-%M-%S", &timeinfo);
  }
  return true;
}

void i2cScan(){
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


void initSensors(){
  //Wire.begin(13, 12);               //SDA orange, SCL purple      // Default is SDA 14, SCL 15
  TwoWire tw = TwoWire(0);
  tw.begin(13,12);
  BME280 bme280;
  SGP30 sgp30;

  if(bme280.beginI2C(tw)){
    bmeMounted = true;
    Serial.println("bme280 found");
  } else {
    Serial.println("bme280 not found"); 
    //i2cScan();
  }
  if(sgp30.begin(tw)){
    sgpMounted = true;
    sgp30.initAirQuality();
    Serial.println("sgp30 found");
  } else {
    Serial.println("sgp30 not found");
    //i2cScan();
  }
}

bool mountSDCard(){
  if(SDmounted){
     Serial.println("SD Card already mounted :)");
     return true;
  }
  if(!SD_MMC.begin("/sdcard", true)){       //1 bit mode to free up pins 12 and 13
    Serial.println("SD Card Mount Failed");
    delay(2000);
    SDmounted = false;
    return false;
  } else {
    esp_task_wdt_reset();
    Serial.println("SD card mounted");
    SDmounted = true;
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    return true;
  }
}

void unmountSDCard(){
  SD_MMC.end();
  SDmounted = false;
  Serial.println("SD card unmounted");
  pinMode(dehumidifierControlPin, OUTPUT);  //disable flash
  digitalWrite(dehumidifierControlPin, LOW); 
}


void getSensorReadings(){
  //First 15 readings from SGP30 will be
  //CO2: 400 ppm  TVOC: 0 ppb as it warms up

  if(!bmeMounted){
    if(bme280.beginI2C(tw)){
      bmeMounted = true;
    } 
  }
  if(!sgpMounted){
    if(sgp30.begin(tw)){
      sgpMounted = true;
      sgp30.initAirQuality();
    }
  }


  if(!bmeMounted && !sgpMounted){return;}
  Serial.println("reading sensors");
  if(bmeMounted){
    humidity = bme280.readFloatHumidity();
    temp = bme280.readTempC();
  }

  if(sgpMounted){
    sgp30.measureAirQuality();
    co2 = sgp30.CO2;
    tvoc = sgp30.TVOC;
  }

  char s[50];
  snprintf_P(s, sizeof(s), PSTR("{'temp':%f, 'humidity':%f, 'co2':%f, 'tvoc':%f}"), temp, humidity, co2, tvoc);
  
  mqttclient.publish("box/environ", s);
}


void getSensorReadingsWeb(AsyncWebServerRequest *request){
  char s[135];
  snprintf_P(s, sizeof(s), PSTR("{'temp':%f, 'humidity':%f, 'co2':%f, 'tvoc':%f, 'lowerBound':%i, 'upperBound':%i, dehumidifierState:%d}"), temp, humidity, co2, tvoc, lowerHumidityBound, upperHumidityBound, dehumidiferState);
  request->send_P(200, "application/json", s);
}

void pressDehumidifierButton(){
  while(SDmounted){
    //Serial.println("waiting for pin 4 to be free :)");
    delay(500); //wait for SD card to not be mounted so we can use pin 4
    esp_task_wdt_reset();
  }
  pinMode(dehumidifierControlPin, OUTPUT);
  Serial.println("Switching dehumdifier");
  digitalWrite(dehumidifierControlPin, HIGH);  
  delay(500); 
  digitalWrite(dehumidifierControlPin, LOW); 
  dehumidiferState = !dehumidiferState;
}

void pressDehumidifierButtonWeb(AsyncWebServerRequest *request){
  pressDehumidifierButton();
  request->send(200, "text/plain", "Dehumidifier switched");
}

void downloadWeb(AsyncWebServerRequest *request){
  esp_task_wdt_reset(); //feed watchdog here for when downloading loads of files via script
  pauseSensorReading = true;
  Serial.print("Received /download request from ");
  Serial.println(request->client()->remoteIP());

  if(mountSDCard()){
    int paramsNr = request->params();
    for(int i=0;i<paramsNr;i++){

      AsyncWebParameter* p = request->getParam(i);
      String paramName = p->name();
      String paramValue = p->value();
      if(paramName=="file"){
        File file = SD_MMC.open(paramValue, FILE_READ);
        if(!file){
          Serial.println("Failed to open file for reading");
          request->send(404, "text/plain", "File not found. Make sure filename isn't enclosed in quotes.");
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
}

void setUpperbound(AsyncWebServerRequest *request){
    int paramsNr = request->params();
    for(int i=0;i<paramsNr;i++){
      AsyncWebParameter* p = request->getParam(i);
      String paramName = p->name();
      String paramValue = p->value();
      if(paramName=="int" && paramValue.toInt()!=0){
        Serial.println("setting upper bound to");
        Serial.println(paramValue);
        upperHumidityBound = paramValue.toInt();
        request->send(200, "text/plain", "upperBound set");
      } else {
        Serial.println("cast failed or int param not found");
        request->send(400, "cast failed or int param not found");
      }
    }
}

void setLowerbound(AsyncWebServerRequest *request){
    int paramsNr = request->params();
    for(int i=0;i<paramsNr;i++){
      AsyncWebParameter* p = request->getParam(i);
      String paramName = p->name();
      String paramValue = p->value();
      if(paramName=="int" && paramValue.toInt()!=0){
        Serial.println("setting lower bound to");
        Serial.println(paramValue);
        lowerHumidityBound = paramValue.toInt();
        Serial.println(lowerHumidityBound);
        request->send(200, "text/plain", "lowerBound set");
      } else {
        Serial.println("cast failed or int param not found");
        request->send(400, "cast failed or int param not found");
      }
    }
}

void mqttMessageReceived(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
}


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial console to open!
  //turn off flash
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  delay(200);

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
    Serial.println("\nno psram found :( no HD");
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
  } else {
    cameraMounted = true;
    Serial.println("Camera detected");
    sensor_t * s = esp_camera_sensor_get();
    s -> set_lenc(s, 1);    //turn lens correction on (good for fisheye lens)
  }

  mountSDCard();  //Check if SDCard is Attached
  unmountSDCard();
  //sensors
  tw.begin(13,12);
 
  wifiMulti.addAP(SSIDNAME, PASS);    //Add additional networks here
  wifiMulti.run();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(2000);
  }

  Serial.println("");
  Serial.print("Connected to: ");
  Serial.println(SSIDNAME);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqttclient.setServer(mqtt_server, 1883);
  mqttclient.setCallback(mqttMessageReceived);


  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
 
  //http endpoints
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", start_html);
  });

  server.on("/download", downloadWeb);    //Download specified file e.g. /download?file=filename.jpg
  server.on("/highBound", setUpperbound);
  server.on("/lowBound", setLowerbound);  
  server.on("/environ", getSensorReadingsWeb);
  server.on("/press", pressDehumidifierButtonWeb);

  server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request){
    pauseSensorReading = true;
    //delay(2000);
    Serial.print("/list from ");
    Serial.println(request->client()->remoteIP());
    if(mountSDCard()){
      esp_task_wdt_reset();
      listDir(SD_MMC, "/", 0);
      String filesList;
      for (int i = 0; i < files.size(); i++){
          esp_task_wdt_reset();
          filesList += String(files[i]) + String("\n");
      }

      if(numImages == 0){
        request->send(404, "text/plain", "No images found :( but SD is mounted");
        return;
      }
      request->send(200, "text/plain", filesList);
      Serial.print(numImages);
      Serial.println(" images found");
    } else {
      Serial.println("could not mount or find SD card");
      request->send(200, "text/plain", "SD not mounted");
    }
  });

  server.on("/latest", HTTP_GET, [](AsyncWebServerRequest *request){
    pauseSensorReading = true;
    if(!latestFile.equals("")){
      if(mountSDCard()){
        File file = SD_MMC.open(latestFile, FILE_READ);
        if(!file){
          Serial.println(" Failed to open file for reading");
          request->send(404, "text/plain", "File not found");
          return;
        }else{
          request->send(file, latestFile, "text/xhr", true);
        }
        
      } else {
        Serial.println("could not mount or find SD card");
        request->send(200, "text/plain", "SD not mounted");
      }
    } else {
      Serial.println("latest file is unknown");
      request->send(200, "text/plain", "latest file is unknown");
    }
  });


  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request){
    pauseSensorReading =true;
    refreshNetworkTime(false);
    request->send_P(200, "text/html", lastcapture);
  });

  server.begin();

  //turn off flash again
  digitalWrite(4, LOW);
}


void takePicAndSave(){
  Serial.println("Preparing for photos");
  
  if(!cameraMounted){
    Serial.println("Camera not mounted :(");
    return;
  }
  if(!mountSDCard()){
    return;
  }
 
  
  camera_fb_t * fb = NULL;
  refreshNetworkTime(true);

  Serial.println("Checking file space");
  listDir(SD_MMC, "/", 0);
  if(numImages >= MAX_NUMBER_OF_FILES){
    Serial.print(numImages);
    Serial.println(" images found. Deleting oldest file");
    deleteFile(SD_MMC, files[0].c_str());
  }
   
  Serial.println("Taking a pic! >.<");
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed :(");
    return;
  }

  // Path where new picture will be saved in SD Card
  String path = "/" + String(lastcapture) +".jpg";
  
  //Set fs to use SD card
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    latestFile = path;
    
  }
  file.close();
  esp_camera_fb_return(fb); 
  pinMode(dehumidifierControlPin, OUTPUT);  //disable flash
  digitalWrite(dehumidifierControlPin, LOW); 
}



void operateDehumidifier(){   //if humidity too high turn off the dehumifier and vice-versa
  if(bmeMounted && humidity!=(float)0){
    Serial.println("checking dehumdifier");
    if(dehumidiferState){ //dehumidifier on
        if(humidity <= (float)lowerHumidityBound){
          pressDehumidifierButton();
        } else{
          Serial.println("dehumdifier in the right state");
        }
    } else{ //dehumidifer off
      if(humidity >= (float)upperHumidityBound){
        pressDehumidifierButton();
      } else{
        Serial.println("dehumdifier in the right state");
      }
    }
  }
}

void mqttreconnect() {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      mqttclient.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!mqttclient.connected()) {    //setup mqtt
    mqttreconnect();
  }
  mqttclient.loop();

  delay(1000);
  takePicAndSave(); //This method is called first so that latestFile can be set properly. It does delay startup a bit so not ideal
  delay(1000); //delay to properly save

  for (unsigned long i = 0; i < THIRTY_MINS/sensorInterval; i++) {    //exit this loop every half an hour to take pics ~
    if(!pauseSensorReading){  
      getSensorReadings();


      if(i%5){    //every other loop iteration
        operateDehumidifier();
      }

    } else {
      pauseSensorReading = false;
      delay(sensorPauseTime);
      // check if pauseSensorReading has changed to true again in the delay period
      if(!pauseSensorReading){
        unmountSDCard(); 
      } 
    }

    delay(sensorInterval);
  }
}