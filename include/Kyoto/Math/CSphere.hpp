#ifndef _CSPHERE_HPP
#define _CSPHERE_HPP

#include "types.h"

#include "Kyoto/Math/CVector3f.hpp"

class CSphere {
public:
  CSphere(const CVector3f& pos, f32 radius) : x0_pos(pos), xc_radius(radius) {}

  // TODO

private:
  CVector3f x0_pos;
  f32 xc_radius;
};
CHECK_SIZEOF(CSphere, 0x10)

#endif
