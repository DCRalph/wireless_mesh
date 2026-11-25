#pragma once
// Host stand-ins for the bits of Arduino that wireless_mesh's Direct and Mesh use.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

extern uint32_t fakeMillis;
inline uint32_t millis() { return fakeMillis; }
inline uint32_t esp_random() { static uint32_t x = 0x12345678; x ^= x << 13; x ^= x >> 17; x ^= x << 5; return x; }

#define HEX 16
#define F(x) x

class String : public std::string
{
public:
  String() = default;
  String(const char *s) : std::string(s ? s : "") {}
  String(const std::string &s) : std::string(s) {}
  template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  String(T v) : std::string(std::to_string(v)) {}
  String(unsigned long v, int base)
  {
    char b[16];
    snprintf(b, sizeof b, base == 16 ? "%lx" : "%lu", v);
    assign(b);
  }
  String(unsigned v, int base) : String(static_cast<unsigned long>(v), base) {}
};
inline String operator+(const String &a, const String &b) { return String(static_cast<const std::string &>(a) + static_cast<const std::string &>(b)); }
inline String operator+(const String &a, const char *b) { return String(static_cast<const std::string &>(a) + b); }
inline String operator+(const char *a, const String &b) { return String(std::string(a) + static_cast<const std::string &>(b)); }

struct FakeSerial
{
  bool quiet = true;
  void println() {}
  void println(const std::string &s) { if (!quiet) puts(s.c_str()); }
  void println(const char *s) { if (!quiet) puts(s); }
  template <typename... A>
  void printf(const char *, A...) {}
};
inline FakeSerial Serial;
