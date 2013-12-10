#ifndef HELPER_H
#define HELPER_H


#include <stdint.h>

//Originally from optoforce with adaptations to get rid of the QT stuff (markho, Aug.2013)


namespace sp {

struct read_package {
  uint16_t s1;
  uint16_t s2;
  uint16_t s3;
  uint16_t s4;
  uint16_t temp;

  read_package (uint16_t a, uint16_t b, uint16_t c, uint16_t d)
      : s1(a), s2(b), s3(c), s4(d), temp(0) { }
  read_package ()
      : s1(0), s2(0), s3(0), s4(0), temp(0) { }
  const read_package& operator= (uint16_t pack) {
      s1 = pack;
      s2 = pack;
      s3 = pack;
      s4 = pack;
      temp = pack;
      return *this;
  }
};
struct display_package {
  int s1;
  int s2;
  int s3;
  int s4;
  int temp;

  display_package (int a, int b, int c, int d)
      : s1(a), s2(b), s3(c), s4(d), temp(0) { }
  display_package ()
      : s1(0), s2(0), s3(0), s4(0), temp(0) { }
  const display_package& operator= (int pack) {
      s1 = pack;
      s2 = pack;
      s3 = pack;
      s4 = pack;
      temp = pack;
      return *this;
  }
  display_package operator+(const display_package& pack) {
    display_package p;
    p.s1 = this->s1 + pack.s1;
    p.s2 = this->s2 + pack.s2;
    p.s3 = this->s3 + pack.s3;
    p.s4 = this->s4 + pack.s4;
    p.temp = this->temp;
    return p;
  }
  display_package operator*=(const display_package& pack) {
    this->s1 *= pack.s1;
    this->s2 *= pack.s2;
    this->s3 *= pack.s3;
    this->s4 *= pack.s4;
    return *this;
  }
};

void align_offset(display_package& pack, display_package const & offset);

void read_to_display_package(display_package &dest, read_package const & source);

}


#endif // HELPER_H
