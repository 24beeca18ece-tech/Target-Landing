#pragma once
#include <string.h>
struct EEPROMClass {
  unsigned char mem[4096] = {0};
  template <class T> void put(int a, const T &v) { memcpy(mem + a, &v, sizeof(T)); }
  template <class T> void get(int a, T &v) { memcpy(&v, mem + a, sizeof(T)); }
};
extern EEPROMClass EEPROM;
