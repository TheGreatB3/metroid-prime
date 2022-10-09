#ifndef _CCOLOR
#define _CCOLOR

#include "types.h"

#include "Kyoto/Basics/CCast.hpp"

#include <dolphin/gx/GXStruct.h>

#ifdef __MWERKS__
#pragma cpp_extensions on
#endif

class CInputStream;
class CColor {
public:
  CColor() {}
  CColor(uint col) : mRgba(col) {}
  CColor(CInputStream& in);
  CColor(float r, float g, float b, float a = 1.f);
  CColor(uchar r, uchar g, uchar b, uchar a = 255) {
    mR = r;
    mG = g;
    mB = b;
    mA = a;
  }

  void Set(float r, float g, float b, float a);
  void Get(float& r, float& g, float& b, float& a) const;
  void Get(float& r, float& g, float& b) const;
  static CColor Lerp(const CColor& a, const CColor& b, float t);
  static uint Lerp(uint a, uint b, float t);
  static CColor Modulate(const CColor& a, const CColor& b);
  static CColor Add(const CColor& a, const CColor& b);
  float GetRed() const { return CCast::ToReal32(mR) * (1 / 255.f); }
  float GetGreen() const { return CCast::ToReal32(mG) * (1 / 255.f); }
  float GetBlue() const { return CCast::ToReal32(mB) * (1 / 255.f); }
  float GetAlpha() const { return CCast::ToReal32(mA) * (1 / 255.f); }
  uchar GetReduchar() const { return mR; }
  uchar GetGreenuchar() const { return mG; }
  uchar GetBlueuchar() const { return mB; }
  uchar GetAlphauchar() const { return mA; }
  ushort ToRGB5A3() const;
  GXColor ToGX(uint);

  static const CColor& Black();
  static const CColor& White();
  static const CColor& Grey();
  static const CColor& Red();
  static const CColor& Green();
  static const CColor& Blue();
  static const CColor& Yellow();
  static const CColor& Purple();
  static const CColor& Orange();

private:
  union {
    struct {
      uchar mR;
      uchar mG;
      uchar mB;
      uchar mA;
    };
    uint mRgba;
  };

  static const CColor sBlackColor;
  static const CColor sWhiteColor;
  static const CColor sGreyColor;
  static const CColor sRedColor;
  static const CColor sGreenColor;
  static const CColor sBlueColor;
  static const CColor sYellowColor;
  static const CColor sPurpleColor;
  static const CColor sOrangeColor;
};
CHECK_SIZEOF(CColor, 0x4)

#ifdef __MWERKS__
#pragma cpp_extensions off
#endif

#endif // _CCOLOR
