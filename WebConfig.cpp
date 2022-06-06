/*

File WebConfig.cpp
Version 1.4
Author Gerald Lechner
contakt lechge@gmail.com

Description
This library builds a web page with a smart phone friendly form to edit
a free definable number of configuration parameters.
The submitted data will bestored in the SPIFFS
The library works with ESP8266 and ESP32

Dependencies:
  ESP8266WebServer.h
  ArduinoJson.h

*/
#include "WebConfig.h"
#include <Arduino.h>
#if defined(ESP32)
  #include "SPIFFS.h"
  #include <WebServer.h>
#else
  #include <ESP8266WebServer.h>
#endif
#include <ArduinoJson.h>
#include <FS.h>

const char * inputtypes[] = {"text","password","number","date","time","range","check","radio","select","color","float"};

//HTML templates
//Template for header and begin of form
const char HTML_START[] PROGMEM = 
" <!DOCTYPE html>"
"<html>"
"<head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<style>"
"body {"
"background-color: #ddd;"
"margin: 0;"
"font-family: Arial, Helvetica, sans-serif;"
"}"
".header {"
"overflow: hidden;"
"background-color: #22ab42;"
"padding: 0;"
"margin-bottom: 30px;"
"}"
".amaki {"
"text-align:center;"
"bottom: 0%;"
"width: 100%;"
"}"
".header a {"
"float: left;"
"color: black;"
"text-align: center;"
"padding: 10px;"
"text-decoration: none;"
"font-size: 16px; "
"line-height: 20px;"
"border-radius: 4px;"
"}"
".header a:hover {"
"background-color: #71ad7f;"
"color: black;"
"}"
".header a.active {"
"background-color: #71ad7f;"
"color: white;"
"}"
"th, td {"
"border:1px solid black;"
"width:200px;"
"height:50px;"
"padding:5px;"
"}"
".center {"
"margin-left: auto;"
"margin-right: auto;"
"}"
"input {"
"font-size:16px;"
"border:none;"
"background-color: #ddd;"
"color: blue;"
"}"
"</style>"
"</head>"
"<body>"
"<div class='header'>"
"<div class='header-left'>"
"<a href='/'>Bosh sahifa</a>"
"<a href='/config'>Sozlamalar</a>"
"<a href='/table'>Jadval</a>"
"</div>"
"<div style='float:right;'>"
"<a href='#logout' onclick='self.Close()'>Chiqish</a>"
"</div>"
"</div>";

const char HEADER_HTML[] PROGMEM =
"<h3 style='width:100%%;text-align:center;color:#22ab42;'>%s</h3>"
"<form method='post'>"
"<table class='center'>";

const char TABLE_STYLE_SHEET[] PROGMEM = 
"<style>"
"td {"
"border:1px solid black;"
"height: 15px;"
"width:30px;"
"text-align:center;"
"}"
"input {"
"height: 15px;"
"width: 45px;"
"text-align:center;"
"}"
"</style>";

const char TABLE_ROW[] PROGMEM = 
"<td><input name='%s' type='text' value='%s'/></td>";

const char TABLE_INPUT_HTML[] PROGMEM =
"<tr><th><p style='float:left;'>%s</p></th><td><input name='%s' value='%s' type='%s'/></td></tr>\n";

const char TABLE_STAT_HTML[] PROGMEM =
"<tr><th><p style='float:left;'>%s</p></th><td>%s</td></tr>\n";

const char HTML_INPUT_END[] PROGMEM =
"</table>"
"<div class='amaki'><p>"
"<button name='SAVE' type='submit'>Saqlash</button>"
"<button name='RESET' type='submit'>Qayta yuklash</button>"
"</div>"
"</form>"
"<p style='bottom:5px;text-align:center;width:100%'>Copyright by a.k.a</p>"
"</body></html>";

const char HTML_STAT_END[] PROGMEM =
"</table>"
"</form>"
"<div class='amaki'>"
"<p style='bottom:5px;text-align:center;width:100%'>Copyright by a.k.a</p>"
"</div></body></html>";

WebConfig::WebConfig() {
};

void WebConfig::clearStatistics() {
  _count_stat = 0;
}

void WebConfig::addStatistics (String lbl, String val) {
  _statistics[_count_stat].label = lbl;
  _statistics[_count_stat].value = val;
  _count_stat ++;
}

void WebConfig::setStatistics (uint8_t index, String val) {
  _statistics[index].value = val;
}

void WebConfig::setDescription(String parameter){
  _count = 0;
  addDescription(parameter);
}

void WebConfig::addDescription(String parameter){
  DeserializationError error;
  const int capacity = JSON_ARRAY_SIZE(MAXVALUES) + MAXVALUES*JSON_OBJECT_SIZE(8);
  DynamicJsonDocument doc(capacity);
  char tmp[40];
  error = deserializeJson(doc, parameter);
  if (error) {
    // Serial.println(parameter);
    // Serial.print("JSON AddDescription: ");
    // Serial.println(error.c_str());
  } else {
    JsonArray array = doc.as<JsonArray>();
    uint8_t j = 0;
    for (JsonObject obj : array) {
      if (_count < MAXVALUES) {
        _description[_count].optionCnt = 0;
        if (obj.containsKey("name")) strlcpy(_description[_count].name, obj["name"], NAMELENGTH);
        if (obj.containsKey("label"))strlcpy(_description[_count].label, obj["label"], LABELLENGTH);
        if (obj.containsKey("type")) {
          if (obj["type"].is<char *>()) {
            uint8_t t = 0;
            strlcpy(tmp, obj["type"], 30);
            while ((t < INPUTTYPES) && (strcmp(tmp, inputtypes[t]) != 0)) t++;
            if (t>INPUTTYPES) t = 0;
            _description[_count].type = t;
          } else {
            _description[_count].type = obj["type"];
          }
        } else {
          _description[_count].type = INPUTTEXT;
        }
        _description[_count].max = (obj.containsKey("max"))?obj["max"] :100;
        _description[_count].min = (obj.containsKey("min"))?obj["min"] : 0;
        if (obj.containsKey("default")) {
          strlcpy(tmp,obj["default"],30);
          values[_count] = String(tmp);
        } else {
          values[_count]="0";
        }
        if (obj.containsKey("options")) {
          JsonArray opt = obj["options"].as<JsonArray>();
          j = 0;
          for (JsonObject o : opt) {
            if (j<MAXOPTIONS) {
              _description[_count].options[j] = o["v"].as<String>();
              _description[_count].labels[j] = o["l"].as<String>();
            }
            j++;
          }
          _description[_count].optionCnt = opt.size();
        }
      }
      _count++;
    }

  }
  if (!SPIFFS.begin()) {
    SPIFFS.begin();
    if (SPIFFS.exists(CONFFILE)) {
      // Serial.printf("%s - file is found!\n");
      readConfig(CONFFILE);
    } else {
      // Serial.println("File not found!");
      SPIFFS.format();
    }
  }
};

void createTableInput(char * buf, const char * name, const char * label, const char * type, String value) {
  sprintf(buf, TABLE_INPUT_HTML, label, name, value.c_str(), type);
}

void createTableStat(char * buf, const char * label, String value) {
  sprintf(buf, TABLE_STAT_HTML, label, value.c_str());
}

void createHeader(char * buf, String value) {
  sprintf(buf, HEADER_HTML, value.c_str());
}

#if defined(ESP32)
  void WebConfig::handleFormRequest(WebServer * server, int uri){
    handleFormRequest(server, CONFFILE, uri);
  }
  void WebConfig::handleFormRequest(WebServer * server, const char * filename, int uri){
#else
  void WebConfig::handleFormRequest(ESP8266WebServer * server, int uri){
    handleFormRequest(server, CONFFILE, uri);
  }
  void WebConfig::handleFormRequest(ESP8266WebServer * server, const char * filename, int uri){
#endif
  for (uint8_t i = 0; i < server->args(); i++) {
    if (server->args() < MAXVALUES) {
      if (server->hasArg(_description[i].name)) values[i] = server->arg(_description[i].name);
    }
    else {
      if (server->hasArg("t" + String(i)) && i < MAXVALUESTABLE) table_values[i] = server->arg("t"+String(i)).toInt();
    }
  }
  if (server->hasArg(F("SAVE")) || server->hasArg(F("RESET"))) {
    writeConfig(filename);
    if (server->hasArg(F("RESET"))) ESP.restart();
  }
  boolean exit = false;
  if (server->hasArg(F("SAVE")) && _onSave) {
    _onSave(getResults());
    exit = true;
  }
  if (server->hasArg(F("DONE")) && _onDone) {
    _onDone(getResults());
    exit = true;
  }
  if (server->hasArg(F("CANCEL")) && _onCancel) {
    _onCancel();
    exit = true;
  }
  if (server->hasArg(F("DELETE")) && _onDelete) {
    _onDelete(_apName);
    exit = true;
  }
  if (!exit) {
    server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    server->send(200, "text/html", HTML_START);
    if (uri == 1) {
      sprintf(_buf, HEADER_HTML, "Statistika");   //////////////////////////////////////////////////////////////////////////////////////////////////////////
      server->sendContent(_buf);
      for (uint8_t i = 0; i < _count_stat; i++) {
        createTableStat(_buf, _statistics[i].label.c_str(), _statistics[i].value);
        server->sendContent(_buf);
      }
      for (uint8_t i = 0; i<_count; i++) {
        switch (_description[i].type) {
          case INPUTTEXT: createTableStat(_buf, _description[i].label, values[i]);
            break;
          case INPUTPASSWORD: createTableStat(_buf, _description[i].label, "******");
            break;
          default: break;
        }
        server->sendContent(_buf);
      }
      server->sendContent(HTML_STAT_END);
    } else if (uri == 2) {
      sprintf(_buf, HEADER_HTML, "Sozlamalar");   //////////////////////////////////////////////////////////////////////////////////////////////////////////
      server->sendContent(_buf);
      for (uint8_t i = 0; i<_count; i++) {
        switch (_description[i].type) {
          case INPUTTEXT: createTableInput(_buf, _description[i].name, _description[i].label, "text", values[i]);
            break;
          case INPUTPASSWORD: createTableInput(_buf, _description[i].name, _description[i].label, "password", values[i]);
            break;
          default: break;
        }
        server->sendContent(_buf);
      }
      server->sendContent(HTML_INPUT_END);
    } else if (uri == 3) {
      sprintf(_buf, HEADER_HTML, "Koordinatalar jadvali");   //////////////////////////////////////////////////////////////////////////////////////////////////////////
      server->sendContent(_buf);
      server->sendContent(TABLE_STYLE_SHEET);
      for (int8_t i = -1; i<TABLECOLUMN; i++) {
        server->sendContent("<tr>");
        if (i < 0) {
          server->sendContent("<td>HQ</td>");
        } else {
          server->sendContent("<td>" + String(i*10) + "</td>");
        }
        for (uint8_t j = 0; j < TABLEROW; j++) {
          if (i < 0) {
            server->sendContent("<td>" + String(j) + "</td>");
          } else {
            server->sendContent("<td><input name='" + String(i*10 + j) + "' type='text' value='" + String(table_values[i*10 + j]) + "'/></td>");
          }
        }
        server->sendContent("</tr>");
      }
      server->sendContent(HTML_INPUT_END);
    }
  }
}

int16_t WebConfig::getIndex(const char * name){
  int16_t i = _count-1;
  while ((i>=0) && (strcmp(name,_description[i].name)!=0)) {
    i--;
  }
  return i;
}

boolean WebConfig::readConfig(const char * filename){
  String line,name,value;
  for (int i = 0; i < MAXVALUESTABLE;i++){
    table_values[i] = 0;
  }
  if (SPIFFS.exists(CONFTABLE)) {
    File f = SPIFFS.open(CONFTABLE, "r");
    int k = 0, sz = f.size();
    while (f.position() < sz && k <= MAXVALUESTABLE)
    {
      table_values[k] = f.readStringUntil('\n').toInt();
      k++;
    }
  } else {
    writeTableConfig(CONFTABLE);
  }
  uint8_t pos;
  int16_t index;
  if (!SPIFFS.exists(filename)) {
    writeConfig(filename);
  }
  File f = SPIFFS.open(filename, "r");
  if (f) {
    // Serial.println(F("Read configuration"));
    uint32_t size = f.size();
    while (f.position() < size) {
      line = f.readStringUntil(10);
      pos = line.indexOf('=');
      name = line.substring(0,pos);
      value = line.substring(pos+1);
      if ((name == "apName") && (value != "")) {
        _apName = value;
        // Serial.println(line);
      } else {
        index = getIndex(name.c_str());
        if (!(index < 0)) {
          value.replace("~","\n");
          values[index] = value;
          if (_description[index].type == INPUTPASSWORD) {
            // Serial.printf("%s=*************\n",_description[index].name);
          } else {
            // Serial.println(line);
          }
        }
      }
    }
    f.close();
    return true;
  } else {
    // Serial.println(F("Cannot read configuration"));
    return false;
  }
}

boolean WebConfig::readConfig(){
  return readConfig(CONFFILE);
}

boolean WebConfig::writeTableConfig(const char* filename) {
  File f = SPIFFS.open(filename, "w");
  if (f) {
    for (int i = 0; i < MAXVALUESTABLE; i++) {
      f.println(table_values[i]);
    }
    return true;
  }
  else {
    return false;
  }
}

boolean WebConfig::writeConfig(const char * filename){
  String val;
  File f = SPIFFS.open(filename, "w");
  if (f) {
    f.printf("apName=%s\n", _apName.c_str());
    for (uint8_t i = 0; i<_count; i++){
      val = values[i];
      val.replace("\n","~");
      f.printf("%s=%s\n",_description[i].name, val.c_str());
    }
    writeTableConfig(CONFTABLE);
    return true;
  } else {
    // Serial.println(F("Cannot write configuration"));
    return false;
  }

}
//write configuration to default file
boolean WebConfig::writeConfig(){
  return writeConfig(CONFFILE);
}
//delete configuration file
boolean WebConfig::deleteConfig(const char * filename){
  return SPIFFS.remove(filename);
}
//delete default configutation file
boolean WebConfig::deleteConfig(){
  return deleteConfig(CONFFILE);
}

//get a parameter value by its name
const String WebConfig::getString(const char * name) {
  int16_t index;
  index = getIndex(name);
  if (index < 0) {
    return "";
  } else {
    return values[index];
  }
}


//Get results as a JSON string
String WebConfig::getResults(){
  char buffer[1024];
  StaticJsonDocument<1000> doc;
  for (uint8_t i = 0; i<_count; i++) {
    switch (_description[i].type) {
      case INPUTPASSWORD :
      case INPUTSELECT :
      case INPUTDATE :
      case INPUTTIME :
      case INPUTRADIO :
      case INPUTCOLOR :
      case INPUTTEXT : doc[_description[i].name] = values[i]; break;
      case INPUTCHECKBOX :
      case INPUTRANGE :
      case INPUTNUMBER : doc[_description[i].name] = values[i].toInt(); break;
      case INPUTFLOAT : doc[_description[i].name] = values[i].toFloat(); break;

    }
  }
  serializeJson(doc,buffer);
  return String(buffer);
}

//Ser values from a JSON string
void WebConfig::setValues(String json){
  int val;
  float fval;
  char sval[255];
  DeserializationError error;
  StaticJsonDocument<1000> doc;
  error = deserializeJson(doc, json);
  if (error ) {
    // Serial.print("JSON: ");
    // Serial.println(error.c_str());
  } else {
    for (uint8_t i = 0; i<_count; i++) {
      if (doc.containsKey(_description[i].name)){
        switch (_description[i].type) {
          case INPUTPASSWORD :
          case INPUTSELECT :
          case INPUTDATE :
          case INPUTTIME :
          case INPUTRADIO :
          case INPUTCOLOR :
          case INPUTTEXT : strlcpy(sval,doc[_description[i].name],255);
            values[i] = String(sval); break;
          case INPUTCHECKBOX :
          case INPUTRANGE :
          case INPUTNUMBER : val = doc[_description[i].name];
            values[i] = String(val); break;
          case INPUTFLOAT : fval = doc[_description[i].name];
            values[i] = String(fval); break;

        }
      }
    }
  }
}


const char * WebConfig::getValue(const char * name){
  int16_t index;
  index = getIndex(name);
  if (index < 0) {
    return "";
  } else {
    return values[index].c_str();
  }
}

int WebConfig::getInt(const char * name){
  return getString(name).toInt();
}

float WebConfig::getFloat(const char * name){
  return getString(name).toFloat();
}

boolean WebConfig::getBool(const char * name){
  return (getString(name) != "0");
}

//get the accesspoint name
const char * WebConfig::getApName(){
  return _apName.c_str();
}
//get the number of parameters
uint8_t WebConfig::getCount(){
  return _count;
}

//get the name of a parameter
String WebConfig::getName(uint8_t index){
  if (index < _count) {
    return String(_description[index].name);
  } else {
    return "";
  }
}

//set the value for a parameter
void WebConfig::setValue(const char*name,String value){
  int16_t i = getIndex(name);
  if (i>=0) values[i] = value;
}

//set the label for a parameter
void WebConfig::setLabel(const char * name, const char* label){
  int16_t i = getIndex(name);
  if (i>=0) strlcpy(_description[i].label,label,LABELLENGTH);
}

//remove all options
void WebConfig::clearOptions(uint8_t index){
  if (index < _count) _description[index].optionCnt = 0;
}

void WebConfig::clearOptions(const char * name){
  int16_t i = getIndex(name);
  if (i >= 0) clearOptions(i);
}

//add a new option
void WebConfig::addOption(uint8_t index, String option){
  addOption(index,option,option);
}

void WebConfig::addOption(uint8_t index, String option, String label){
  if (index < _count) {
    if (_description[index].optionCnt < MAXOPTIONS) {
      _description[index].options[_description[index].optionCnt]=option;
      _description[index].labels[_description[index].optionCnt]=label;
      _description[index].optionCnt++;
    }
  }
}

//modify an option
void WebConfig::setOption(uint8_t index, uint8_t option_index, String option, String label){
  if (index < _count) {
    if (option_index < _description[index].optionCnt) {
      _description[index].options[option_index] = option;
      _description[index].labels[option_index] = label;
    }
  }

}

void WebConfig::setOption(char * name, uint8_t option_index, String option, String label){
  int16_t i = getIndex(name);
  if (i >= 0) setOption(i,option_index,option,label);
}

//get the options count
uint8_t WebConfig::getOptionCount(uint8_t index){
  if (index < _count) {
    return _description[index].optionCnt;
  } else {
    return 0;
  }
}

uint8_t WebConfig::getOptionCount(char * name){
  int16_t i = getIndex(name);
  if (i >= 0) {
    return getOptionCount(i);
  } else {
    return 0;
  }

}

//set form type to doen cancel
void WebConfig::setButtons(uint8_t buttons){
  _buttons = buttons;
}
//register onSave callback
void WebConfig::registerOnSave(void (*callback)(String results)){
  _onSave = callback;
}
//register onSave callback
void WebConfig::registerOnDone(void (*callback)(String results)){
  _onDone = callback;
}
//register onSave callback
void WebConfig::registerOnCancel(void (*callback)()){
  _onCancel = callback;
}
//register onDelete callback
void WebConfig::registerOnDelete(void (*callback)(String name)){
  _onDelete = callback;
}
