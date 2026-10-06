#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <ctype.h>
#include <string>
extern uint64_t g_us;
inline uint32_t millis() { return (uint32_t)(g_us / 1000); }
inline uint32_t micros() { return (uint32_t)g_us; }
inline void delay(uint32_t ms) { g_us += (uint64_t)ms * 1000; }
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define HEX 16
#define BUILTIN_SDCARD 254
#define SERIAL_8N1_HALF_DUPLEX 0x100
struct Print {
  virtual size_t write(uint8_t) { return 1; }
  size_t print(const char *s) { return fputs(s, stdout) >= 0; }
  size_t print(char c) { return putchar(c); }
  size_t print(int v, int base = 10) { return base == 16 ? printf("%X", v) : printf("%d", v); }
  size_t print(unsigned v, int base = 10) { return base == 16 ? printf("%X", v) : printf("%u", v); }
  size_t print(long v) { return printf("%ld", v); }
  size_t print(unsigned long v) { return printf("%lu", v); }
  size_t print(double v, int d = 2) { return printf("%.*f", d, v); }
  size_t println() { return puts(""); }
  template <class T> size_t println(T v) { size_t n = print(v); puts(""); return n; }
  template <class T> size_t println(T v, int d) { size_t n = print(v, d); puts(""); return n; }
};
struct HardwareSerial : Print {
  std::string in; size_t pos = 0; bool usb = false;
  void begin(uint32_t, int = 0) {}
  int available() { return (int)(in.size() - pos); }
  int read() { return pos < in.size() ? (uint8_t)in[pos++] : -1; }
  size_t write(uint8_t) override { return 1; }
  void addMemoryForRead(void *, size_t) {}
  explicit operator bool() const { return true; }
};
extern HardwareSerial Serial, Serial1, Serial3;
