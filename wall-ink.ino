#include <driver/adc.h>
#include <GxEPD2_BW.h>
#include <Arduino.h>
#include <Preferences.h>
#include <Hash.h>
#include "debug_mode.h"
#include "credentials.h"
//#include "testImage.h"
#include <pgmspace.h>
#define FIRMWARE_VERSION "4.02a"
#define BASE_URL "http://192.168.1.22/get_image.php"
#define DEVICE_TYPE 1
#define ADMIN_MODE_ENABLED 1
#define MAX_SLEEP 20
#define PD 25
#define MIN_SLEEP 10
#define ONE_DAY 86400
#define ONE_HOUR 3600
#define INITIAL_DRIFT_SECONDS 3
#define DEBUG_ON_BY_DEFAULT true
#define WIFI_PROFILE_1_ACTIVE_BY_DEFAULT true
#define ENABLE_GxEPD2_GFX 0
#define X_RES 1304
#define Y_RES 984
#define ROTATION 0

GxEPD2_BW < GxEPD2_1248, GxEPD2_1248::HEIGHT / 4 >
display(GxEPD2_1248(/*sck=*/ 13, /*miso=*/ 34, /*mosi=*/ 14, /*cs_m1=*/ 23, /*cs_s1=*/ 22, /*cs_m2=*/ 16, /*cs_s2=*/ 19, /*dc1=*/ 27, /*dc2=*/ 17, /*rst1=*/ 33, /*rst2=*/ 5, /*busy_m1=*/ 32, /*busy_s1=*/ 26, /*busy_m2=*/ 18, /*busy_s2=*/ 4));

#include <esp_wifi.h>
#include "admin_mode.h"
#include <Fonts/FreeMonoBold9pt7b.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <EEPROM.h>
#include <WebServer.h>

WiFiMulti WiFiMulti;

WebServer server(80);
Preferences preferences;

// CRC function used to ensure data validity
uint32_t calculateCRC32(const uint8_t *data, size_t length);

//number of failed attempts to connect to wifi
uint16_t attempts = 0;

// Structure which will be stored in RTC memory.
// First field is CRC32, which is calculated based on the
// rest of structure contents.
// Any fields can go after CRC32.
// Must use a multiple of 4 bytes of RAM.
struct {
  uint32_t crc32;
  uint32_t currentTime;
  uint32_t nextTime;
  uint32_t elapsedTime;
  int32_t driftSeconds;
  uint32_t consecutiveCrashes;
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t bssid[6]; // 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
  char ssid[20];
  char password[20];
  char imageHash[20];
  uint32_t errorCode;
} rtcData;

struct {
  bool wifiProfile1Active;
  char baseURL[100];
  char ssid0[20];
  char ssid1[20];
  char password0[20];
  char password1[20];
  char imageKey[20];
  bool debug;
  uint8_t hash[20];
} eeprom;

String url = "";
bool res;
void saveRTC() {
  preferences.begin("storage", false);
  preferences.putUInt("crc32", rtcData.crc32);
  preferences.putUInt("currentTime", rtcData.currentTime);
  preferences.putUInt("nextTime", rtcData.nextTime);
  preferences.putUInt("elapsedTime", rtcData.elapsedTime);
  preferences.putUInt("driftSeconds", rtcData.driftSeconds);
  preferences.putUInt("consCrashes", rtcData.consecutiveCrashes);
  preferences.putUInt("channel", rtcData.channel);
  preferences.putBytes("bssid", rtcData.bssid, sizeof(rtcData.bssid));
  preferences.putBytes("ssid", rtcData.ssid, sizeof(rtcData.ssid));
  preferences.putBytes("password", rtcData.password, sizeof(rtcData.password));
  preferences.putBytes("imageHash", rtcData.imageHash, sizeof(rtcData.imageHash));
  preferences.putUInt("padding", rtcData.padding);
  preferences.putUInt("errorCode", rtcData.errorCode);
  preferences.end();
}
void restoreRTC() {
  preferences.begin("storage", false);
  rtcData.crc32 = preferences.getUInt("crc32");
  rtcData.currentTime = preferences.getUInt("currentTime");
  rtcData.nextTime = preferences.getUInt("nextTime");
  rtcData.elapsedTime = preferences.getUInt("elapsedTime");
  rtcData.driftSeconds = preferences.getUInt("driftSeconds");
  rtcData.consecutiveCrashes = preferences.getUInt("consCrashes");
  rtcData.channel = preferences.getUInt("channel");
  size_t bssidLen = preferences.getBytesLength("bssid");
  char BSSIDbuffer[bssidLen];
  preferences.getBytes("bssid", BSSIDbuffer, bssidLen);
  memcpy(rtcData.bssid, BSSIDbuffer, bssidLen);
  rtcData.padding = preferences.getUInt("padding");
  size_t ssidLen = preferences.getBytesLength("ssid");
  char SSIDbuffer[ssidLen];
  preferences.getBytes("ssid", SSIDbuffer, ssidLen);
  memcpy(rtcData.ssid, SSIDbuffer, ssidLen);
  size_t passwordLen = preferences.getBytesLength("password");
  char passwordbuffer[passwordLen];
  preferences.getBytes("password", passwordbuffer, passwordLen);
  memcpy(rtcData.password, passwordbuffer, passwordLen);
  size_t imageHashLen = preferences.getBytesLength("imageHash");
  char imageHashbuffer[imageHashLen];
  preferences.getBytes("password", imageHashbuffer, imageHashLen);
  memcpy(rtcData.imageHash, imageHashbuffer, imageHashLen);
  rtcData.errorCode = preferences.getUInt("errorCode");
  preferences.end();
}
bool readEeprom() {
  EEPROM.begin(sizeof(eeprom));
  EEPROM.get(0, eeprom);
  EEPROM.end();
  uint8_t hash[20];
  sha1((uint8_t*) &eeprom, sizeof(eeprom) - 20, hash);
  bool dataCorrupted = false;
  for (int i = 0; i < 20; i++) {
    if (eeprom.hash[i] != hash[i]) {
      dataCorrupted = true;
    }
  }
  return !dataCorrupted;
}
float battery_read() { //read battery voltage per %
  long sum = 0;                  // sum of samples taken
  float voltage = 0.0;           // calculated voltage
  float output = 0.0;            //output value
  const float battery_max = 3.6; //maximum voltage of battery
  const float battery_min = 3.3; //minimum voltage of battery before shutdown

  float R1 = 100000.0; // resistance of R1 (100K)
  float R2 = 10000.0;  // resistance of R2 (10K)

  for (int i = 0; i < 500; i++)
  {
    sum += adc1_get_voltage(ADC1_CHANNEL_0);
    delayMicroseconds(1000);
  }
  // calculate the voltage
  voltage = (sum / (float)500);
  voltage = (voltage * 1.1) / 4096.0; //for internal 1.1v reference
  // use if added divider circuit
  // voltage = voltage / (R2/(R1+R2));
  //round value by two precision
  voltage = roundf(voltage * 100) / 100;
  Serial.print("voltage: ");
  Serial.println(voltage, 2);
  output = ((voltage - battery_min) / (battery_max - battery_min)) * 100;
  if (output < 100)
    //return output;
    return 3000.0f;
  else
    return 3000.0f;
}
void writeEeprom() {
  sha1((uint8_t*) &eeprom, sizeof(eeprom) - 20, eeprom.hash);
  EEPROM.begin(sizeof(eeprom));
  EEPROM.put(0, eeprom);
  EEPROM.end();
}
String getMAC() {
  String mac = WiFi.macAddress();
  while (mac.indexOf(':') != -1) {
    mac.remove(mac.indexOf(':'), 1);
  }
  return mac;
}
void setURL() {
  url = eeprom.baseURL;
  url += "?mac_address=";
  url += getMAC();
  url += "&firmware=";
  url += FIRMWARE_VERSION;
  if (eeprom.debug) {
    url += "_debug";
  }
  url += "&error=";
  url += rtcData.errorCode;
  url += "&width=";
  url += X_RES;
  url += "&height=";
  url += Y_RES;
  url += "&voltage=";
  if (eeprom.debug) {
    Serial.println("Measuring Voltage...");
    Serial.print("Time in milliseconds: ");
    Serial.println(ESP.getCycleCount() / 80000);
  }
  float volts = 0.00f;
  volts = battery_read();
  url += String(volts / 1024.00f);
  if (eeprom.debug) {
    Serial.print("Voltage: ");
    Serial.println((volts / 1024.00f) + 2);
    Serial.print("Time in milliseconds: ");
    Serial.println(ESP.getCycleCount() / 80000);
  }
}
void crash(String reason) {

  ++rtcData.consecutiveCrashes;

  //set sleep time here
  uint32_t sleepTime;

  switch (rtcData.consecutiveCrashes) {
    case 1:
      sleepTime = 60 * 1;
      break;
    case 2:
      sleepTime = 60 * 5;
      break;
    case 3:
      sleepTime = 60 * 15;
      break;
    case 4:
      sleepTime = 60 * 30;
      break;
    case 5:
      sleepTime = 60 * 30;
      break;
    case 6:
      sleepTime = 60 * 30;
      break;
    case 7:
      sleepTime = 60 * 30;
      break;
    case 8:
      sleepTime = ONE_HOUR * 2;
      break;
    case 9:
      sleepTime = ONE_HOUR * 2;
      break;
    case 10:
      sleepTime = ONE_HOUR * 2;
      break;
    default:
      sleepTime = ONE_HOUR * 4;
  }

  if (rtcData.consecutiveCrashes > 3 || eeprom.debug) {
    dumpToScreen(reason, sleepTime);
  }

  rtcData.nextTime += sleepTime;

  if (eeprom.debug) {
    Serial.println(reason);

    Serial.print("SSID:");
    Serial.println(WiFi.SSID());

    Serial.print("Free heap:");
    Serial.println(system_get_free_heap_size());

    Serial.print("Router MAC Address:");
    Serial.println(WiFi.BSSIDstr());

    Serial.print("IP address:");
    Serial.println(WiFi.localIP());

    Serial.print("Gateway IP:");
    Serial.println(WiFi.gatewayIP());

    Serial.print("RSSI:");
    Serial.println(WiFi.RSSI());

    Serial.print("MAC Address:");
    //Serial.println(WiFi.macAddress());
    Serial.println(getMAC());
    Serial.print("URL:");
    setURL();
    Serial.println(url);

    Serial.print("Last successful attempt at:");
    Serial.println(rtcData.currentTime);

    Serial.print("Last unsuccessful attempt at:");
    Serial.println(rtcData.currentTime + rtcData.elapsedTime);

    //Serial.print("Next planned wakeup time:");
    //Serial.println(rtcData.nextTime);

    Serial.print("Next attempt in:");
    Serial.print(sleepTime);
    Serial.println(" seconds");
  }

  if (sleepTime > MAX_SLEEP) {
    sleepTime = MAX_SLEEP;
  }

  rtcData.elapsedTime += sleepTime;

  // Update CRC32 of data
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);

  // Write struct to RTC memory; this if statement has important side-effects; do not remove it!
  //if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
  //Serial.println("Write: ");
  //printMemory();
  //Serial.println();
  //}
  saveRTC();
  delay(200);
  ESP.deepSleep(sleepTime * 1000000);

}
void sleep() {
  uint32_t sleepTime = rtcData.nextTime - rtcData.currentTime - rtcData.elapsedTime;
  if (sleepTime > MAX_SLEEP) {
    sleepTime = MAX_SLEEP;
  }
  rtcData.elapsedTime += sleepTime;

  // Update CRC32 of data
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
  // Write struct to RTC memory
  //if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
  //}
  saveRTC();

  if (eeprom.debug) {
    Serial.print("currentTime: ");
    Serial.println(rtcData.currentTime);
    Serial.print("nextTime: ");
    Serial.println(rtcData.nextTime);
    Serial.print("elapsedTime: ");
    Serial.println(rtcData.elapsedTime);
    Serial.print("consecutiveCrashes: ");
    Serial.println(rtcData.consecutiveCrashes);
  }

  if (sleepTime + rtcData.driftSeconds < MIN_SLEEP) {
    if (eeprom.debug) {
      Serial.print("Sleeping for the minimum of");
      Serial.print(MIN_SLEEP);
      Serial.println(" seconds");
      Serial.print("Drift seconds: ");
      Serial.println(rtcData.driftSeconds);
    }
    delay(200);
    ESP.deepSleep(MIN_SLEEP * 1000000);
  }
  if (eeprom.debug) {
    Serial.print("sleepTime: ");
    Serial.print(sleepTime);
    Serial.println(" seconds");
    Serial.print("Drift seconds added to the above figure: ");
    Serial.println(rtcData.driftSeconds);
    Serial.print("Time in milliseconds: ");
    Serial.println(ESP.getCycleCount() / 80000);
    Serial.println("");
  }
  delay(200);
  ESP.deepSleep((sleepTime + rtcData.driftSeconds) * 1000000);
}
void dumpToScreen(String reason, uint32_t sleepTime) {
  display.firstPage();
  do
  {
    display.setRotation(ROTATION);
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(0, 0);
    display.println();

    display.println(reason);

    display.print("SSID:");
    display.println(WiFi.SSID());

    display.print("Router MAC Address:");
    display.println(WiFi.BSSIDstr());

    display.print("IP address:");
    display.println(WiFi.localIP());

    display.print("Gateway IP:");
    display.println(WiFi.gatewayIP());

    display.print("RSSI:");
    display.println(WiFi.RSSI());

    display.print("MAC Address:");
    //display.println(WiFi.macAddress());
    display.println(getMAC());

    display.print("URL:");
    setURL();
    display.println(url);

    display.print("Last successful attempt at:");
    display.println(rtcData.currentTime);

    //display.print("Original planned wakeup time:");
    //display.println(rtcData.nextTime);

    display.print("Last unsuccessful attempt at:");
    display.println(rtcData.currentTime + rtcData.elapsedTime);

    display.print("Next attempt in:");
    display.print(sleepTime);
    display.println(" seconds");
  }
  while (display.nextPage());
  //display.update();
  for (int i = 0; i < 20; i++)
    rtcData.imageHash[i] = -1;
}
void handleRoot() {
  if (eeprom.debug) {
    Serial.println("Received request at /");
  }
  String webPage = "<!DOCTYPE HTML> <head> <script type='text/javascript'> function myFunction() { var copyText = document.getElementById('macaddress'); console.log(copyText); copyText.select(); document.execCommand('copy'); } </script> <style type='text/css'> body { margin: auto; background: #eaeaea; font-family: 'Open Sans', sans-serif; } .info p { text-align:center; color: #999; text-transform:none; font-weight:600; font-size:15px; margin-top:2px } .info i { color:#F6AA93; } h2 { width:100px; margin: 0px auto; } form h1 { font-size: 18px; background: #F6AA93 none repeat scroll 0% 0%; color: rgb(255, 255, 255); padding: 22px 25px; border-radius: 5px 5px 0px 0px; margin: auto; text-shadow: none; text-align:left } form { border-radius: 5px; max-width:700px; width:100%; margin: 10px auto; background-color: #FFFFFF; overflow: hidden; } p span { color: #000; } p { margin: 0px; font-weight: 500; line-height: 2; color:#333; } h1 { text-align:center; color: #666; text-shadow: 1px 1px 0px #FFF; margin:50px 0px 0px 0px } input { border-radius: 0px 5px 5px 0px; border: 1px solid #eee; margin-bottom: 15px; width: 75%; height: 40px; padding: 0px 15px; } input#macaddress { width: 120px; height: 25px; } a { text-decoration:inherit } legend { font-weight: bold; } .form-group { overflow: hidden; clear: both; } i { color:#555; } .contentform { padding: 40px 30px; } .button-contact{ background-color: #81BDA4; color: #FFF; text-align: center; width: 100%; border:0; padding: 17px 25px; border-radius: 0px 0px 5px 5px; cursor: pointer; margin-top: 40px; font-size: 18px; } .leftcontact { width:49.5%; float:left; border-right: 1px dotted #CCC; box-sizing: border-box; padding: 0px 15px 0px 15px; } .rightcontact { width:49.5%; float:right; box-sizing: border-box; padding: 0px 0px 0px 15px; } #sendmessage { border:1px solid #fff; display:none; text-align:center; margin:10px 0; font-weight:600; margin-bottom:30px; background-color: #EBF6E0; color: #5F9025; border: 1px solid #B3DC82; padding: 13px 40px 13px 18px; border-radius: 3px; box-shadow: 0px 1px 1px 0px rgba(0, 0, 0, 0.03); } #sendmessage.show,.show { display:block; } .pure-form input { padding:.5em .6em; display:inline-block; border:1px solid #ccc; box-shadow:inset 0 1px 3px #ddd; border-radius:4px; vertical-align:middle; box-sizing:border-box } .pure-form .pure-checkbox,.pure-form .pure-radio{ margin:.5em 0; display:block } .pure-form label{ margin:.5em 0 .2em } .pure-form fieldset{ margin:0; padding:.35em 0 .75em; border:0 } .pure-form legend{ display:block; width:100%; padding:.3em 0; margin-bottom:.3em; color:#333; border-bottom:1px solid #e5e5e5 } .pure-form .pure-group fieldset{ margin-bottom:10px } .pure-form .pure-group input{ display:block; padding:10px; margin:0 0 -1px; border-radius:0; position:relative; top:-1px } .pure-form .pure-group button{ margin:.35em 0 }";
  webPage += "</style></head><body>";
  webPage += "<h1>Web Configuration</h1>";
  webPage += "<div class='info'>";
  webPage += "<p>MAC Address <input id='macaddress' value='";
  webPage += getMAC();
  webPage += "'></span><i class='fa fa-heart'></i><br/>Firmware Version";
  webPage += FIRMWARE_VERSION;
  webPage += "<br/><button onclick='myFunction()'>Copy MAC Address to clipboard</button></p></div>";
  webPage += "<form method='get' action='submit' class='pure-form'><h1>Please fill out the form to configure your wall-ink device</h1><div class='leftcontact'><fieldset class='form-group'><legend>Wifi Connection</legend><p><label for='ssid0'>Wireless SSID0: </label><br/><input id='ssid0' name='ssid0' value='";
  webPage += eeprom.ssid0;
  webPage += "' length=20></p><p><label for='password0'>Password0: </label><br/><input id='password0' name='password0' value='";
  webPage += eeprom.password0;
  webPage +=   "' length=20></p></fieldset><fieldset class='form-group'><legend>Backup Wifi Connection 1</legend><p><label for='ssid1'>Wireless SSID1: </label><br/><input id='ssid1' name='ssid1' value='";
  webPage += eeprom.ssid1;
  webPage += "' length=20></p><p><label for=password1>Password1: </label><br/><input id='password1' name='password1' value='";
  webPage += eeprom.password1;
  webPage += "' length=20></p></fieldset></div><div class='rightcontact'><fieldset class='form-group'><legend>Wall Ink Server Settings</legend><p><label for='baseurl'>Base URL: </label><br/><input id='baseurl' name='baseurl' value='";
  webPage += eeprom.baseURL;
  webPage += "' length=100></p><p><label for='imagekey'>Image Key: </label><br/><input id='imagekey' name='imagekey' value='";
  webPage +=  eeprom.imageKey;
  webPage += "' length=100>'</p></fieldset><fieldset class='form-group'><legend>Debug Mode</legend><p><label for='on'>On</label><br/><input name='debug' id='debug' value='1' type='radio'></p><p><label for='off'>Off</label><br/><input name='debug' id='debug' value='0' checked='true' type='radio'></p></fieldset></div><button type='submit' class='button-contact'>Send</button></form></body></html>";
  server.send(200, "text/html", webPage);
}
void handleSubmit() {
  String webPage = "<!DOCTYPE HTML> <head><style type='text/css'>body { margin: auto; background: #eaeaea; font-family: 'Open Sans', sans-serif; } h1 { text-align:center; color: #666; text-shadow: 1px 1px 0px #FFF; margin:50px 0px 0px 0px } a { text-decoration:inherit } h2 {width:100px; margin: 0px auto;}";
  webPage += "</style> </head><body>";
  webPage += "<h1>Settings Saved</h1>";
  webPage += "<h2><a href='/'>Back</a></h2>";
  webPage += "</body></html>";
  server.send(200, "text/html", webPage);
  strcpy(eeprom.ssid0, server.arg("ssid0").c_str());
  strcpy(eeprom.password0, server.arg("password0").c_str());
  strcpy(eeprom.ssid1, server.arg("ssid1").c_str());
  strcpy(eeprom.password1, server.arg("password1").c_str());
  strcpy(eeprom.baseURL, server.arg("baseurl").c_str());
  strcpy(eeprom.imageKey, server.arg("imagekey").c_str());
  eeprom.debug = server.arg("debug").equals("1");
  writeEeprom();
  if (eeprom.debug == 1) {
    Serial.println("Received request at /submit");
    Serial.println("SSID0: ");
    Serial.println(server.arg("ssid0"));
    Serial.println("Password0: ");
    Serial.println(server.arg("password0"));
    Serial.println("SSID1: ");
    Serial.println(server.arg("ssid1"));
    Serial.println("password1: ");
    Serial.println(server.arg("password1"));
    Serial.println("baseurl: ");
    Serial.println(server.arg("baseurl"));
    Serial.println("imagekey: ");
    Serial.println(server.arg("imagekey"));
    Serial.println("Debug Mode: ");
    Serial.println(server.arg("debug"));
    Serial.println("New values have been written to Eeprom");
  }
}
void activateAdminMode() {

  // Write bad crc32 so that a new image is refreshed on reboot
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData), sizeof(rtcData));
  // Write struct with bad crc32 to RTC memory
  //if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
  //}
  saveRTC();

  if (eeprom.debug) {
    Serial.println();
    Serial.print("Configuring access point...\n");
  }

  while (display.nextPage());
  //display.update();
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  unsigned char* buf = new unsigned char[20];
  String ssid_t = "BYU_DD_" + getMAC();
  ssid_t.getBytes(buf, 20, 0);
  const char *ssid = (const char*)buf;
  const char *adm_pass = (const char*)ADMIN_PASSWORD;
  WiFi.softAP(ssid, adm_pass);
  delay(2000);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.on("/submit", handleSubmit);
  server.begin();
  Serial.println("Webserver started, RTC memory invalidated");
  while (true) {
    server.handleClient();
    delay(100);
  }
}
void eepromSetDefault() {
  eeprom.debug = DEBUG_ON_BY_DEFAULT;
  eeprom.wifiProfile1Active = WIFI_PROFILE_1_ACTIVE_BY_DEFAULT;
  strcpy(eeprom.baseURL, BASE_URL);
  strcpy(eeprom.ssid0, SSID0);
  strcpy(eeprom.ssid1, SSID1);
  strcpy(eeprom.password0, PASSWORD0);
  strcpy(eeprom.password1, PASSWORD1);
  strcpy(eeprom.imageKey, DEFAULT_IMAGE_KEY);
  writeEeprom();
}
void readRTC() {
  if (true) {
    restoreRTC();
    if (eeprom.debug) {
      //Serial.println("Read: ");
      //printMemory();
      //Serial.println();
    }
    uint32_t crcOfData = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
    if (eeprom.debug) {
      Serial.print("CRC32 of data: ");
      Serial.println(crcOfData, HEX);
      Serial.print("CRC32 read from RTC: ");
      Serial.println(rtcData.crc32, HEX);
    }
    if (crcOfData != rtcData.crc32) {
      if (eeprom.debug) {
        Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Data is probably invalid!");
        Serial.println("Connecting to wifi with default settings");
        Serial.print("Time in milliseconds: ");
        Serial.println(ESP.getCycleCount() / 80000);
      }
      WiFi.mode(WIFI_STA);
      WiFiMulti.addAP(eeprom.ssid0, eeprom.password0);
      WiFiMulti.addAP(eeprom.ssid1, eeprom.password1);
      while (WiFiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
        if (WiFi.status() == WL_CONNECT_FAILED) {
          rtcData.errorCode = 4;
          crash("WiFi connection failed");
        } else if (WiFi.status() == WL_NO_SSID_AVAIL) {
          rtcData.errorCode = 5;
          crash("SSID not available");
        } else if (WiFi.status() == WL_CONNECTION_LOST) {
          rtcData.errorCode = 6;
          crash("WiFi connection lost");
        }
        attempts++;
        if (attempts == 15) {
          //WiFi.disconnect();
          delay(1);
          //WiFiMulti.addAP(WIFI_SSID0, WIFI_PASSWORD0);
          //WiFiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
          //WiFiMulti.run();
        } else if (attempts > 40) {
          rtcData.errorCode = 7;
          crash("Error connecting to WiFi");
        }
        delay(500);
        if (eeprom.debug) {
          String a = "Attempt " + String(attempts);
          Serial.println(a);
        }
      }
      rtcData.currentTime = 0;
      rtcData.nextTime = 0;
      rtcData.elapsedTime = 0;
      rtcData.errorCode = 0;
      for (int i = 0; i < 20; i++)
        rtcData.imageHash[i] = 0;
      rtcData.driftSeconds = INITIAL_DRIFT_SECONDS;
      rtcData.consecutiveCrashes = 0;
    } else {
      //if we aren't there yet, sleep
      if (rtcData.elapsedTime + rtcData.currentTime < rtcData.nextTime) {
        sleep();
      }
      randomSeed(ESP.getCycleCount());
      if (random(30) == 1 || rtcData.consecutiveCrashes != 0) {
        WiFiMulti.addAP(eeprom.ssid0, eeprom.password0);
        WiFiMulti.addAP(eeprom.ssid1, eeprom.password1);
        if (eeprom.debug) {
          Serial.println("Connecting to wifi with default settings");
          Serial.print("Time in milliseconds: ");
          Serial.println(ESP.getCycleCount() / 80000);
        }
        WiFi.mode(WIFI_STA);
        while (WiFiMulti.run() != WL_CONNECTED) {
          Serial.print(".");
          delay(500);
          if (WiFi.status() == WL_CONNECT_FAILED) {
            rtcData.errorCode = 4;
            crash("WiFi connection failed");
          } else if (WiFi.status() == WL_NO_SSID_AVAIL) {
            rtcData.errorCode = 5;
            crash("SSID not available");
          } else if (WiFi.status() == WL_CONNECTION_LOST) {
            rtcData.errorCode = 6;
            crash("WiFi connection lost");
          }
          attempts++;
          if (attempts == 15) {
            //WiFi.disconnect();
            delay(1);
            //WiFiMulti.addAP(WIFI_SSID0, WIFI_PASSWORD0);
            //WiFiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
            //WiFiMulti.run();
          } else if (attempts > 40) {
            rtcData.errorCode = 7;
            crash("Error connecting to WiFi");
          }
          delay(500);
          if (eeprom.debug) {
            String a = "Attempt " + String(attempts);
            Serial.println(a);
          }
        }
      } else {
        WiFi.persistent(false);
        WiFi.begin(rtcData.ssid, rtcData.password, rtcData.channel, rtcData.bssid);
        if (eeprom.debug) {
          Serial.println("Connecting to wifi with stored settings");
          Serial.print("Time in milliseconds: ");
          Serial.println(ESP.getCycleCount() / 80000);
        }
      }
    }
  }
}
void setup() {
  //if EEPROM data is bad, regenerate it

  if (!readEeprom()) {
    eepromSetDefault();
  }

  yield();
  //flush serial
  if (eeprom.debug) {
    Serial.begin(115200);

    Serial.print("Time in milliseconds: ");
    Serial.println(ESP.getCycleCount() / 80000);
    // Serial.setDebugOutput(true);
    for (uint8_t t = 4; t > 0; t--) {
      Serial.printf("[SETUP] WAIT %d...\n", t);
      Serial.flush();
      delay(100);
    }
  }
  display.init(115200);
  yield();

#if ADMIN_MODE_ENABLED == 1
  //activate AP mode if set to do so
  pinMode(PD, INPUT_PULLUP);
  if (digitalRead(PD) == LOW) {
    activateAdminMode();
  } else {
    WiFi.mode(WIFI_STA);
  }
#else
  WiFi.mode(WIFI_STA);
#endif

  //Don't write wifi info to flash
  WiFi.persistent(false);

  // Read struct from RTC memory
  readRTC();

}

void loop() {
  // wait for WiFi connection
  unsigned char test_image_1[] = {0};
  if ((WiFi.status() == WL_CONNECTED)) {
    if (eeprom.debug) {
      Serial.println("WiFi Connection established");
      Serial.print("IP address:");
      Serial.println(WiFi.localIP());
      Serial.print("Gateway IP:");
      Serial.println(WiFi.gatewayIP());
      Serial.print("Subnet Mask:");
      Serial.println(WiFi.subnetMask());
      Serial.print("Time in milliseconds: ");
      Serial.println(ESP.getCycleCount() / 80000);
    }

    if (WiFi.RSSI() < -90) {
      rtcData.errorCode = 1;
      crash("Connection Weak");
    }

    //Save wifi connection info
    strcpy(rtcData.ssid, WiFi.SSID().c_str());
    strcpy(rtcData.password, WiFi.psk().c_str());
    rtcData.channel = WiFi.channel();
    memcpy(rtcData.bssid, WiFi.BSSID(), 6);

    // configure server and url
    setURL();
    Serial.println("URL: ");
    Serial.println(url);

    if (eeprom.debug) {
      Serial.print("[HTTP] begin...\n");
    }

    HTTPClient http;
    http.begin(url);

    if (eeprom.debug) {
      Serial.print("[HTTP] GET...\n");
    }
    // start connection and send HTTP header
    int httpCode = http.GET();
    if (httpCode == 404) {
      rtcData.errorCode = 404;
      crash("Error:404");
    } else if (httpCode == 200) {
      if (eeprom.debug) {
        Serial.print("Time in milliseconds: ");
        Serial.println(ESP.getCycleCount() / 80000);
      }
      uint32_t cursor = 0;
      uint8_t eightPixels;
      boolean initialized = false;
      // get length of document (is -1 when Server sends no Content-Length header)
      int len = http.getSize();
      if (eeprom.debug) {
        Serial.print("File Size: ");
        Serial.println(len);
      }

      //this will happen if the mac address isn't found in the database
      if (len < 50) {
        rtcData.errorCode = 2;
        crash("File too small or not found");
        if (eeprom.debug) {
          system_get_free_heap_size();
        }
      }

      // create buffer for read
      uint8_t buff[128] = { 0 };

      // get tcp stream
      WiFiClient * stream = http.getStreamPtr();

      //sha1( sha1(raw_image) . sha1(chunk2 . chunk3) . sha1(mac_address) . sha1(image_key) )
      //this will be filled out at various points below.
      //the server does the same thing, and the client compares the results.
      uint8_t* hash = (uint8_t*) malloc(80);
      uint8_t* serverEverythingHash = (uint8_t*) malloc(20);
      int byte_num = 0;
      // read all data from server

      while (http.connected() && (len > 0 || len == -1)) {
        // get available data size
        size_t size = stream->available();
        Serial.print("Size: "); Serial.println(size);
        Serial.println("");
        if (size) {
          // read up to 128 byte
          int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
          for (int offset = 0; offset < c; offset++) {
            if (!initialized) {           //Grab the stuff off the front of the file
              initialized = true;
              offset += 68;
            }
            eightPixels = buff[offset];
            if (cursor < X_RES * Y_RES) {
              for (uint8_t i = 0; i < 8; i++) {
                if ((eightPixels >> (7 - i)) & 0x01)
                  display.drawPixel((cursor + i) % X_RES, (cursor + i) / X_RES, GxEPD_BLACK);
              }
              cursor += 8;
            }
          }
          if (len > 0) {
            len -= c;
          }
        }
        delay(1);
      }
      WiFi.disconnect();
      delay(10);
      //WiFi.forceSleepBegin();
      delay(10);
      if (eeprom.debug) {
        Serial.println("Calculating SHA1 hash");
        Serial.print("Time in milliseconds: ");
        Serial.println(ESP.getCycleCount() / 80000);
      }
      //sha1( sha1(mac_address) . sha1(raw_image) . sha1(image_key) )
      //the server does the same thing, and the client compares them
      //move image key hash to the end
      memcpy(hash + 60, hash + 40, 20);
      //hash the image


      uint8_t* finalImageHash = (uint8_t*) malloc(20);
      sha1(hash + 20, 60, finalImageHash);
      if (eeprom.debug) {
        Serial.println("local image hash: ");
        for (int i = 0; i < 20; i++) {
          Serial.print(String(finalImageHash[i], HEX));
        }
        Serial.println("");
        Serial.println("");
      }
      /*bool imageVerified = true;
      for (int i = 0; i < 20; i++) {
        if (rtcData.imageHash[i] != finalImageHash[i]) {
          imageVerified = false;
        }
      }
      //sha1( sha1(time_data) . sha1(mac_address) . sha1(raw_image) . sha1(image_key) )
      //the server does the same thing, and the client compares them
      //this is used to defend against certain types of replay attacks
      uint8_t* finalEverythingHash = (uint8_t*) malloc(20);
      sha1(hash, 80, finalEverythingHash);
      if (eeprom.debug) {
        Serial.println("local hash with everything: ");
        for (int i = 0; i < 20; i++) {
          Serial.print(String(finalEverythingHash[i], HEX));
        }
        Serial.println("");
        Serial.println("");
      }
      for (int i = 0; i < 20; i++) {
        if (serverEverythingHash[i] != finalEverythingHash[i]) {

          imageVerified = false;
        }
        Serial.print(String(serverEverythingHash[i], HEX));
      }
      Serial.println("");
      /*  if (!imageVerified) {
          if (eeprom.debug) {
            Serial.println("Image or password failed verification test, sleeping");
            Serial.print("Time in milliseconds: ");
            Serial.println(ESP.getCycleCount() / 80000);
          }
          WiFi.disconnect();
          delay(10);
          //WiFi.forceSleepBegin();
          delay(10);
          rtcData.errorCode = 3;
          crash("Image failed verification test");
          sleep();
        }*/
      if (eeprom.debug) {
        Serial.println("Updating display");
        Serial.print("Time in milliseconds: ");
        Serial.println(ESP.getCycleCount() / 80000);
        // memcpy_P(display._buffer, debug_image, 1120);
      }
      //display.update();
      rtcData.errorCode = 0;
      if (eeprom.debug) {
        Serial.println("Display updated, sleeping");
        Serial.print("Time in milliseconds: ");
        Serial.println(ESP.getCycleCount() / 80000);
      }
      sleep();
      if (eeprom.debug) {
        Serial.println();
        Serial.print("[HTTP] connection closed or file end.\n");
      }
    } else {
      if (eeprom.debug) {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      rtcData.errorCode = httpCode;
      crash("Error connecting to server");
    }

    http.end();
  } else if (WiFi.status() == WL_CONNECT_FAILED) {
    rtcData.errorCode = 4;
    crash("WiFi connection failed");
  } else if (WiFi.status() == WL_NO_SSID_AVAIL) {
    rtcData.errorCode = 5;
    crash("SSID not available");
  } else if (WiFi.status() == WL_CONNECTION_LOST) {
    rtcData.errorCode = 6;
    crash("WiFi connection lost");
  }
  attempts++;
  if (attempts == 15) {
    //WiFi.disconnect();
    delay(1);
    //WiFiMulti.addAP(WIFI_SSID0, WIFI_PASSWORD0);
    //WiFiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
    //WiFiMulti.run();
  } else if (attempts > 40) {
    rtcData.errorCode = 7;
    crash("Error connecting to WiFi");
  }
  delay(500);
  if (eeprom.debug) {
    String a = "Attempt " + String(attempts);
    Serial.println(a);
  }
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}
