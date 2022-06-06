#include <WebServer.h>
#include "WebConfig.h"
#include <SoftwareSerial.h>
#include <DHT.h>

#define DHTPIN      27
#define DHTTYPE     DHT11

#define VMET_PIN  35
#define AMET_PIN  34
#define TEMP_PIN  27
#define GSMP_PIN  25
#define GSMR_PIN  26

#define AT_CHK      0
#define AT_CSQ      1
#define AT_APN      2
#define AT_NET_ON   3
#define AT_NET_OFF  4
#define AT_NET_CHK  5
#define AT_HTTPGET  6
#define AT_GPS_ON   7
#define AT_GPS_OFF  8
#define AT_LOCATION 9
#define AT_IP_CHK   10


String commands[] = {
  "AT",
  "AT+CSQ",
  "AT+CGDCONT=1,\"IP\",\"DEFAULT\"",
  "AT+CGACT=1,1",
  "AT+CGACT=0,0",
  "AT+CGACT?",
  "AT+HTTPGET=\"%s\"",
  "AT+GPS=1",
  "AT+GPS=0",
  "AT+LOCATION=2",
  "AT+CGDCONT?",
};
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial gsmSerial(33, 32);
SoftwareSerial RS485Serial(18, 19);

byte readDistance [8] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA};
byte ultrasonic_data [7];
byte readLevel [8] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0B};
uint32_t persecond_time = 0, permill_time = 0;
uint16_t distance = 0, fix_length = 0;
float voltage = 0.0, tmp = 26.50, hmt = 31.40;
uint32_t vcounter = 0, curtime_http = 0;
String location = "40.082016,65.909041", ip_addr = "0.0.0.0", device_id = "", server_url = "";
uint8_t old_cmd_gsm = 0, csq = 0, httpget_time = 0;
bool next_cmd = true, waitHttpAction = false, star_project = false;
bool internet = false, httpinit = false, checked_internet = false;
uint32_t per_hour_time = 0, message_count = 0;
uint8_t _counter_httpget = 0;

String params = "["
  "{"
    "'name':'ssid',"
    "'label':'WLAN nomi',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'AKA-AP'"
  "},"
  "{"
    "'name':'pwd',"
    "'label':'WLAN paroli',"
    "'type':"+String(INPUTPASSWORD)+","
    "'default':'salomaka'"
  "},"
  "{"
    "'name':'username',"
    "'label':'WEB login',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'admin'"
  "},"
  "{"
    "'name':'password',"
    "'label':'WEB parol',"
    "'type':"+String(INPUTPASSWORD)+","
    "'default':'esp32'"
  "},"
  "{"
    "'name':'server_url',"
    "'label':'URL server',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'http://m.and-water.uz/bot/app.php?'"
  "},"
  "{"
    "'name':'timeout',"
    "'label':'Xabar vaqti (soat)',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'1'"
  "},"
  "{"
    "'name':'fixing',"
    "'label':'Tuzatish',"
    "'type':"+String(INPUTTEXT)+","
    "'default':'0'"
  "}"
"]";

WebServer server;
WebConfig conf;

struct cmdQueue {
  String cmd [16];
  uint8_t cmd_id[16];
  int k;
  void init () {
    k = 0;
    for (int i = 0; i < 16; i++) {
      cmd_id[i] = -1;
    }
  }
  void addQueue (String msg, uint8_t msg_id) {
    cmd[k] = msg;
    cmd_id[k++] = msg_id;
    if (k > 15) k = 0;
  }
  void sendCmdQueue () {
    if (k > 0) {
      if (!waitHttpAction) {
        gsmSerial.println(cmd[0]);
//        Serial.println(cmd[0]);
        old_cmd_gsm = cmd_id[0];
        if (cmd_id[0] == AT_HTTPGET) {
          waitHttpAction = true;
        }
        k --;
        next_cmd = false;
        for (int i = 0; i < k; i++) {
          cmd[i] = cmd[i+1];
          cmd[i+1] = "";
          cmd_id[i] = cmd_id[i+1];
          cmd_id[i+1] = -1;
        }
      }
    }
  }
};
cmdQueue queue;

void configRoot() {
  if (!server.authenticate(conf.values[2].c_str(), conf.values[3].c_str())) {
    return server.requestAuthentication();
  }
  conf.handleFormRequest(&server, 2);
}

void handleRoot () {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) {
    return server.requestAuthentication();
  }
  conf.handleFormRequest(&server, 1);
}

void tableRoot () {
  if (!server.authenticate(conf.getValue("username"), conf.getValue("password"))) {
    return server.requestAuthentication();
  }
  conf.handleFormRequest(&server, 3);
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

String make_api() {
  return ("?id=" + device_id + "&location={" + location + "}&data=" + String(conf.table_values[distance + fix_length]) + "&temperature=" + String(tmp) + "&humidity=" + String(hmt) + "&power=" + String(voltage));
}

void setup() {
  Serial.begin(115200);
  gsmSerial.begin(9600);
  RS485Serial.begin(9600);
  
  pinMode(VMET_PIN, INPUT);
  pinMode(AMET_PIN, INPUT);
  pinMode(GSMP_PIN, OUTPUT);
  pinMode(GSMR_PIN, OUTPUT);
  
  digitalWrite(GSMP_PIN, HIGH);
  delay(3000);
  digitalWrite(GSMP_PIN, LOW);
  Serial.println("Wait for GSM modem...");
  uint32_t ms_check = millis();
  while (!star_project) {
    if (millis() - ms_check > 20000) {
      ms_check = millis();
      gsmSerial.println("AT");
    }
    checkCommandGSM();
  }
  Serial.println("Start.");
  delay(1000);
  queue.init();
  queue.addQueue(commands[AT_CHK], AT_CHK);
  queue.addQueue(commands[AT_CSQ], AT_CSQ);
  queue.addQueue(commands[AT_NET_OFF], AT_NET_OFF);
  queue.addQueue(commands[AT_APN], AT_APN);
  queue.addQueue(commands[AT_NET_ON], AT_NET_ON);
  queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
  queue.addQueue(commands[AT_GPS_OFF], AT_GPS_OFF);
  queue.addQueue(commands[AT_GPS_ON], AT_GPS_ON);
  queue.addQueue(commands[AT_IP_CHK], AT_IP_CHK);
  dht.begin();
  delay(1000);
  device_id = WiFi.macAddress();
  device_id.replace(":","");
  conf.clearStatistics();
  conf.addStatistics("Qurilma ID", device_id);                        // 0 - index Qurilma id
  conf.addStatistics("Joylashuv nuqtasi", location);                  // 1 - index Location
  conf.addStatistics("Internet IP", ip_addr);                         // 2 - index IP
  conf.addStatistics("Xabarlar soni", String(message_count));         // 3 - index Counter
  conf.addStatistics("Harorat", "26.35");                             // 4 - index Harorat
  conf.addStatistics("Namlik", "30.00");                              // 5 - index Namlik
  conf.addStatistics("Quvvat", "12.45");                              // 6 - index Quvvat
  conf.addStatistics("ANT Signal", "29");                             // 7 - index ANT
  conf.addStatistics("Suv sarfi", "0");                               // 8 - index ANT
  conf.setDescription(params);
  conf.readConfig();
  fix_length = uint16_t(conf.getInt("fixing"));
  httpget_time = uint8_t(conf.getInt("timeout"));
  server_url = conf.getString("server_url");
  WiFi.softAP(conf.getValue("ssid"), conf.getValue("pwd"));
  Serial.print("WebServer IP-Adress = ");
  Serial.println(WiFi.softAPIP());
  delay(1000);
  server.on("/config", configRoot);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/table", tableRoot);
  server.begin(80);
  curtime_http = millis();
  per_hour_time = millis();
  persecond_time = millis();
  permill_time = millis();
  
}

void loop() {
  server.handleClient();
  if (per_hour_time >= 60) { // 3600 qilsa 1 soatda ++ bo'ladi
    _counter_httpget ++; // har minutta ++
    per_hour_time = 0;
  }
  // har 1 sekundda 1 marta ishlash;
  if (millis() - persecond_time > 1000) {
    persecond_time = millis();
    RS485Serial.write(readLevel, 8);
    per_hour_time ++; // har sekundda ++
  }
  // har 100 millisikundda 1 marta ishlash;
  if (millis() - permill_time > 100) {
    permill_time = millis();
    voltage += float(analogRead(VMET_PIN));
    vcounter ++;
  }
  // batareyka voltini hisoblash va ekranga chiqarish;
  if (vcounter >= 30) {
    voltage = voltage / vcounter;
    voltage = map(voltage, 0, 4096, 0, 21.72);              // Voltage value
    voltage = map(voltage, 8.0, 12.0, 0, 100);              // Voltage percent
    if (voltage < 0) voltage = 0;
    if (voltage > 100) voltage = 100;
    int volt = int(voltage);
    int water_level = conf.table_values[distance + fix_length];
    tmp = dht.readTemperature();
    hmt = dht.readHumidity();
    conf.setStatistics(1, location);                        // 1 - index Location
    conf.setStatistics(2, ip_addr);                         // 2 - index IP
    conf.setStatistics(3, String(message_count));           // 3 - index Counter
    conf.setStatistics(4, String(tmp) + "C");
    conf.setStatistics(5, String(hmt) + "%");
    conf.setStatistics(6, String(volt) + "%");
    conf.setStatistics(7, String(csq) + "dB");
    conf.setStatistics(8, String(water_level));
    if (next_cmd && !waitHttpAction) {
      queue.addQueue(commands[AT_CSQ], AT_CSQ);
      queue.addQueue(commands[AT_NET_CHK], AT_NET_CHK);
      if (ip_addr == "0.0.0.0") queue.addQueue(commands[AT_IP_CHK], AT_IP_CHK);
      queue.addQueue(commands[AT_LOCATION], AT_LOCATION);
    }
    if (_counter_httpget >= httpget_time && internet) {
      char temporary[200];
      sprintf(temporary, commands[AT_HTTPGET].c_str(), (server_url + make_api()).c_str());
      queue.addQueue(String(temporary), AT_HTTPGET);
      _counter_httpget = 0;
    }
    vcounter = 0;
    voltage = 0;
  }
  // masofa sensoridan ma'lumot olish;
  if (RS485Serial.available()) {
    RS485Serial.readBytes(ultrasonic_data, 7);
    distance = ultrasonic_data[3] << 8 | ultrasonic_data[4];
    
  }
  // navbatni bo'shatish
  if (next_cmd && millis() - curtime_http > 500) {
    curtime_http = millis();
    queue.sendCmdQueue();
  }
  checkCommandGSM();
//  if (Serial.available()) {
//    gsmSerial.println(Serial.readString());
//  }
//  if (gsmSerial.available()) {
//    Serial.print(char(gsmSerial.read()));
//  }
}

String gsm_data = "";
void checkCommandGSM () {
  if (gsmSerial.available()) {
    char a = gsmSerial.read();
    if (a != '\n') {
      gsm_data += a;
    } else {
      if (gsm_data.length() > 1) {
//        Serial.println(gsm_data);
        check_CMD(gsm_data);
      }
      gsm_data = "";
    }
  }
}

void check_CMD (String str) {
  if (old_cmd_gsm == AT_HTTPGET) {
    if (str.indexOf("Server:") >= 0) {
      message_count ++;
      waitHttpAction = 0;
      next_cmd = 1;
    } else if (str.indexOf("+CME") >= 0) {
      waitHttpAction = 0;
    }
  } 
  else if (old_cmd_gsm == AT_CSQ) {
    if (str.indexOf("+CSQ") >= 0) {
      csq = str.substring(6, 8).toInt();
    }
  }
  else if (old_cmd_gsm == AT_IP_CHK) {
    if (str.indexOf("+CGDCONT:1") >= 0) {
      int def = str.indexOf("DEFAULT");
      ip_addr = str.substring(def + 10, def + 24);
    }
  }
  else if (old_cmd_gsm == AT_NET_CHK) {
    if (str.indexOf("+CGACT: 1") >= 0) {
      internet = true;
    } else if (str.indexOf("+CGACT: 0") >= 0){
      internet = false;
    }
  } else if (old_cmd_gsm == AT_LOCATION) {
    if (str.indexOf(",") >= 0) {
      location = str;
      location.trim();
    }
  }
  if (str.indexOf("HTTP/1.1 200 OK") >= 0) {
    message_count ++;
    waitHttpAction = 0;
    return;
  }
  if (str.indexOf("OK") >= 0 || str.indexOf("+CME") >= 0) {
    next_cmd = true;
    star_project = true;
    waitHttpAction = 0;
    return;
  }
  if (str.indexOf("READY") >= 0) {
    star_project = true;
    return;
  }
}
