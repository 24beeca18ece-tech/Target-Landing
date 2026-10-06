#pragma once
#include "Arduino.h"
struct File : Print {
  bool open = false;
  explicit operator bool() const { return open; }
  void close() { open = false; }
  void flush() {}
  size_t write(uint8_t) override { return 1; }
};
#define FILE_WRITE 1
struct SDClass { bool begin(int) { return false; } bool exists(const char *) { return false; } File open(const char *, int) { return File(); } };
extern SDClass SD;
