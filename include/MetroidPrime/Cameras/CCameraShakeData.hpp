#ifndef _CCAMERASHAKEDATA
#define _CCAMERASHAKEDATA

#include "types.h"

#include "Kyoto/Math/CVector3f.hpp"

struct SCameraShakePoint {
  bool x0_useEnvelope;
  float x4_value;
  float x8_magnitude;
  float xc_attackTime;
  float x10_sustainTime;
  float x14_duration;
};
CHECK_SIZEOF(SCameraShakePoint, 0x18)

class CCameraShakerComponent {
public:
  virtual ~CCameraShakerComponent();

private:
  bool x4_useModulation;
  SCameraShakePoint x8_am;
  SCameraShakePoint x20_fm;
  float x38_value;
};
CHECK_SIZEOF(CCameraShakerComponent, 0x3c)

class CCameraShakeData {
private:
  float x0_duration;
  float x4_curTime;
  CCameraShakerComponent x8_shakerX;
  CCameraShakerComponent x44_shakerY;
  CCameraShakerComponent x80_shakerZ;
  uint xbc_shakerId;
  uint xc0_flags;
  CVector3f xc4_sfxPos;
  float xd0_sfxDist;
};
CHECK_SIZEOF(CCameraShakeData, 0xd4)

#endif // _CCAMERASHAKEDATA
