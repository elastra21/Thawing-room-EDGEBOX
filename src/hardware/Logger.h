#ifndef LOGGER_H
#define LOGGER_H

#define WebSerial Serial

#include <Arduino.h>

#ifdef WebSerial
  // No incluir WebSerialLite.h
#else
  #include "WebSerialLite.h"
#endif

class Logger {
private:
  #ifdef WebSerial_h
      #define outputStream WebSerial
  #else
      #define outputStream Serial
  #endif

public:
  void init(unsigned long baudRate);
  
  void print(const String &message);

  void println(const String &message);
  
  void printValue(const String &key, const String &value);

  void printTime(const String &prefix, int hour, int minute, int day, int month);
};

extern Logger logger; 

#endif // LOGGER_H