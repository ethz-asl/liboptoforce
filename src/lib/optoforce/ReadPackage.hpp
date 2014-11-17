#ifndef READPACKAGE_H
#define READPACKAGE_H

#include <stdint.h>

//Originally from optoforce with adaptations to get rid of the QT stuff (markho, Aug.2013)

namespace sp {
  class ReadPackage {
  public:
    /* Raw data */
    uint16_t s1;
    uint16_t s2;
    uint16_t s3;
    uint16_t s4;

    uint16_t temp;

    inline ReadPackage (uint16_t a, uint16_t b, uint16_t c, uint16_t d) :
      s1(a), s2(b), s3(c), s4(d), temp(0) {
    };
        
    inline ReadPackage () :
      s1(0), s2(0), s3(0), s4(0), temp(0) {
    };
        
    inline const ReadPackage& operator= (uint16_t pack) {
      s1 = pack;
      s2 = pack;
      s3 = pack;
      s4 = pack;
      temp = pack;
      return *this;
    };
  };
};


#endif // READPACKAGE_H
