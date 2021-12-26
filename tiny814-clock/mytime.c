#include "mytime.h"

void mytime_add_hour(volatile MYTIME *self) {
  self->hr0++;
  if (self->hr0 >= 10) {
    self->hr0 = 0;
    self->hr1++;
  }
  if (self->hr1 == 2 && self->hr0 == 4) {
    // reset 24h to 00h
    self->hr0 = 0;
    self->hr1 = 0;
  }
}

void mytime_add_min(volatile MYTIME *self) {
  self->min0++;
  if (self->min0 >= 10) {
    self->min0 = 0;
    self->min1++;
    if (self->min1 >= 6) {
      self->min1 = 0;
      mytime_add_hour(self);
    }
  }
}

void mytime_add_second(volatile MYTIME *self) {
  self->sec0++;
  if (self->sec0 >= 10) {
    self->sec0 = 0;
    self->sec1++;
    if (self->sec1 >= 6) {
      self->sec1 = 0;
      mytime_add_min(self);
    }
  }
}

void mytime_add_fraction(volatile MYTIME *self) {
  self->frac++;
  if (self->frac >= 8) {
    self->frac = 0;
    mytime_add_second(self);
  }
}
