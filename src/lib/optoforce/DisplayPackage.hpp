#ifndef DISPLAYPACKAGE_H
#define DISPLAYPACKAGE_H

#include <stdint.h>

//Originally from optoforce with adaptations to get rid of the QT stuff (markho, Aug.2013)

namespace sp {
  class DisplayPackage {
  public:
    int s1;
    int s2;
    int s3;
    int s4;
    /* Compensated raw datas */
    int s1c;
    int s2c;
    int s3c;
    int s4c;

    /* Force vectors */
    int x;
    int y;
    int z;

    /* Compensated force vectors */
    int xc;
    int yc;
    int zc;

    int temp;

    inline DisplayPackage (int a, int b, int c, int d) :
      s1(a), s2(b), s3(c), s4(d), temp(0) {
    };
        
    inline DisplayPackage () :
      s1(0), s2(0), s3(0), s4(0), temp(0) {        
    };
        
    inline const DisplayPackage& operator= (int pack) {
      s1 = pack;
      s2 = pack;
      s3 = pack;
      s4 = pack;
      temp = pack;
      return *this;
    };
    
    inline DisplayPackage operator+(const DisplayPackage& pack) {
      DisplayPackage p;
      p.s1 = this->s1 + pack.s1;
      p.s2 = this->s2 + pack.s2;
      p.s3 = this->s3 + pack.s3;
      p.s4 = this->s4 + pack.s4;
      p.temp = this->temp;
      return p;
    };
    
    inline DisplayPackage operator*=(const DisplayPackage& pack) {
      this->s1 *= pack.s1;
      this->s2 *= pack.s2;
      this->s3 *= pack.s3;
      this->s4 *= pack.s4;
      return *this;
    };
  };
};


#endif // DISPLAYPACKAGE_H
