#include "Logger.h"

Logger logger; 

void Logger::init(unsigned long baudRate) {
    #ifdef WebSerial_h // Verifica si WebSerialLite.h está incluido 
        // find way to init the webserial from here
    #else
        outputStream.begin(baudRate);
    #endif
}

void Logger::print(const String &message) {
    outputStream.print(message);
}

void Logger::println(const String &message) {
    outputStream.println(message);
}

void Logger::printValue(const String &key, const String &value) {
    outputStream.println(key + ": " + value);
}

void Logger::printTime(const String &prefix, int hour, int minute, int day, int month) {
    String timeStr = prefix + ": " + 
                     String(hour) + "h " + 
                     String(minute) + "min " + 
                     String(day) + "day " + 
                     String(month) + "month";
    outputStream.println(timeStr);
}