#ifndef WebConfig_h
#define WebConfig_h

#include <Arduino.h>
#if defined(ESP32)
  #include <WebServer.h>
#else
  #include <ESP8266WebServer.h>
#endif

#define MAXVALUES 20
#define TABLEROW  10
#define TABLECOLUMN 15
#define MAXVALUESTABLE TABLEROW * TABLECOLUMN
#define MAXOPTIONS 15

#define NAMELENGTH 20
#define LABELLENGTH 50

#define CONFFILE "/WebConf.conf"
#define CONFTABLE "/TableConf.conf"

#define INPUTTEXT 0
#define INPUTPASSWORD 1
#define INPUTNUMBER 2
#define INPUTDATE 3
#define INPUTTIME 4
#define INPUTRANGE 5
#define INPUTCHECKBOX 6
#define INPUTRADIO 7
#define INPUTSELECT 8
#define INPUTCOLOR 9
#define INPUTFLOAT 10
#define INPUTTEXTAREA 11
#define INPUTMULTICHECK 12

#define INPUTTYPES 13
#define BTN_CONFIG 0
#define BTN_DONE 1
#define BTN_CANCEL 2
#define BTN_DELETE 4

typedef struct  {
  char name[NAMELENGTH];
  char label[LABELLENGTH];
  uint8_t type;
  int min;
  int max;
  uint8_t optionCnt;
  String options[MAXOPTIONS];
  String labels[MAXOPTIONS];
} DESCRIPTION;

struct STATISTICS
{
  String label;
  String value;
};

class WebConfig {
public:
  WebConfig();
  void setDescription(String parameter);
  void addDescription(String parameter);
  void clearStatistics();
  void addStatistics(String lbl, String val);
  void setStatistics(uint8_t index, String val);
#if defined(ESP32)
  void handleFormRequest(WebServer * server, const char * filename, int uri);
  void handleFormRequest(WebServer * server, int uri);
#else
  void handleFormRequest(ESP8266WebServer * server, const char * filename, int uri);
  void handleFormRequest(ESP8266WebServer * server, int uri);
#endif
  int16_t getIndex(const char * name);
  boolean readConfig(const char *  filename);
  boolean readConfig();
  boolean writeTableConfig(const char* filename);
  boolean writeConfig(const char *  filename);
  boolean writeConfig();
  boolean deleteConfig(const char *  filename);
  boolean deleteConfig();
  const String getString(const char * name);
  const char * getValue(const char * name);
  int getInt(const char * name);
  float getFloat(const char * name);
  boolean getBool(const char * name);
  const char * getApName();
  uint8_t getCount();
  String getName(uint8_t index);
  String getResults();
  void setValues(String json);
  void setValue(const char*name,String value);
  void setLabel(const char * name, const char* label);
  void clearOptions(uint8_t index);
  void clearOptions(const char * name);
  void addOption(uint8_t index, String option);
  void addOption(uint8_t index, String option, String label);
  void setOption(uint8_t index, uint8_t option_index, String option, String label);
  void setOption(char * name, uint8_t option_index, String option, String label);
  uint8_t getOptionCount(uint8_t index);
  uint8_t getOptionCount(char * name);
  void setButtons(uint8_t buttons);
  void registerOnSave(void (*callback)(String results));
  void registerOnDone(void (*callback)(String results));
  void registerOnCancel(void (*callback)());
  void registerOnDelete(void (*callback)(String name));

  String values[MAXVALUES];
  int table_values[MAXVALUESTABLE];
private:
  char _buf[2000];
  uint8_t _count;
  uint8_t _count_stat;
  String _apName;
  int _fixing_len;
  uint8_t _buttons = BTN_CONFIG;
  DESCRIPTION _description[MAXVALUES];
  STATISTICS _statistics[MAXVALUES];
  void (*_onSave)(String results) = NULL;
  void (*_onDone)(String results) = NULL;
  void (*_onCancel)() = NULL;
  void (*_onDelete)(String name) = NULL;
};

#endif
