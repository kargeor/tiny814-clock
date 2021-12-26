#include <stdint.h>

typedef struct {
  uint8_t hr1;  // hours X10 (0-23)
  uint8_t hr0;  // hours  X1 (0-9)
  uint8_t min1; // mins  X10 (0-5)
  uint8_t min0; // mins   X1 (0-9)
  uint8_t sec1; // sec   X10
  uint8_t sec0; // sec    X1
  uint8_t frac; // fraction 1/8
} MYTIME;

void mytime_add_hour(volatile MYTIME *self);
void mytime_add_min(volatile MYTIME *self);
void mytime_add_second(volatile MYTIME *self);
void mytime_add_fraction(volatile MYTIME *self);
