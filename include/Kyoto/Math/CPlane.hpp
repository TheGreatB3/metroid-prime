#ifndef _CPLANE
#define _CPLANE

#include "types.h"

#include "Kyoto/Math/CUnitVector3f.hpp"
#include "Kyoto/Math/CVector3f.hpp"

class CPlane {
public:
  CPlane(const CVector3f&, const CUnitVector3f&); // TODO weak
  CPlane(float, const CUnitVector3f&);            // TODO weak
  CPlane(const CVector3f&, const CVector3f&, const CVector3f&);
  // TODO

  const CUnitVector3f& GetNormal() const { return x0_normal; }
  f32 GetConstant() const { return xc_constant; }
  // GetHeight__6CPlaneCFRC9CVector3f
  // IsFacing__6CPlaneCFRC9CVector3f
  float ClipLineSegment(const CVector3f& start, const CVector3f& end) const;

private:
  CUnitVector3f x0_normal;
  f32 xc_constant;
};
CHECK_SIZEOF(CPlane, 0x10)

#endif // _CPLANE
