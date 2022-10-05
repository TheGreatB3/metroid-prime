#ifndef _CMODELFLAGS_HPP
#define _CMODELFLAGS_HPP

#include "types.h"

#include "Kyoto/Graphics/CColor.hpp"

class CModelFlags {
public:
  enum ETrans {
    kT_Opaque = 0,
    kT_Two = 2, // ?
    kT_Blend = 5,
    kT_Additive = 7,
  };
  enum EFlags {
    kF_DepthCompare = 0x1,
    kF_DepthUpdate = 0x2,
    kF_NoTextureLock = 0x4,
    kF_DepthGreater = 0x8,
    kF_DepthNonInclusive = 0x10,
    kF_DrawNormal = 0x20,
    kF_ThermalUnsortedOnly = 0x40,
  };

  CModelFlags(ETrans trans, f32 rgba)
  : x0_blendMode(trans)
  , x1_matSetIdx(0)
  , x2_flags(kF_DepthCompare | kF_DepthUpdate)
  , x4_color(1.f, 1.f, 1.f, rgba) {}
  CModelFlags(ETrans trans, CColor color)
  : x0_blendMode(trans)
  , x1_matSetIdx(0)
  , x2_flags(kF_DepthCompare | kF_DepthUpdate)
  , x4_color(color) {}

  CModelFlags(ETrans blendMode, u8 shadIdx, EFlags flags, const CColor& col)
  : x0_blendMode(blendMode), x1_matSetIdx(shadIdx), x2_flags(flags), x4_color(col) {}

  CModelFlags(const CModelFlags& flags, uint otherFlags)
  : x0_blendMode(flags.x0_blendMode)
  , x1_matSetIdx(flags.x1_matSetIdx)
  , x2_flags(otherFlags)
  , x4_color(flags.x4_color) {}
  CModelFlags(const CModelFlags& flags, bool b /* TODO what's this? */, int shaderSet)
  : x0_blendMode(flags.x0_blendMode)
  , x1_matSetIdx(shaderSet)
  , x2_flags(flags.x2_flags)
  , x4_color(flags.x4_color) {}

  // ?
  CModelFlags(const CModelFlags& flags, ETrans trans, CColor color)
  : x0_blendMode(trans)
  , x1_matSetIdx(flags.x1_matSetIdx)
  , x2_flags(flags.x2_flags)
  , x4_color(color) {}

  // CModelFlags(const CModelFlags& other) : x0_blendMode(other.x0_blendMode),
  // x1_matSetIdx(other.x1_matSetIdx), x2_flags(other.x2_flags), x4_color(other.x4_color) {}
  CModelFlags& operator=(const CModelFlags& other) {
    x0_blendMode = other.x0_blendMode;
    x1_matSetIdx = other.x1_matSetIdx;
    x2_flags = other.x2_flags;
    x4_color = other.x4_color;
    return *this;
  }

  CModelFlags UseShaderSet(int matSet) const { return CModelFlags(*this, false, matSet); }
  CModelFlags DontLoadTextures() const {
    return CModelFlags(*this, GetOtherFlags() | kF_NoTextureLock);
  }
  CModelFlags DepthCompareUpdate(bool compare, bool update) const {
    uint newFlags = 0;
    if (compare) {
      newFlags |= kF_DepthCompare;
    }
    if (update) {
      newFlags |= kF_DepthUpdate;
    }
    return CModelFlags(*this, (x2_flags & ~(kF_DepthCompare | kF_DepthUpdate)) | newFlags);
  }
  CModelFlags DepthBackwards() const {
    return CModelFlags(*this, GetOtherFlags() | kF_DepthGreater);
  }

  ETrans GetTrans() const { return static_cast< ETrans >(x0_blendMode); }
  int GetShaderSet() const { return x1_matSetIdx; }
  uint GetOtherFlags() const { return x2_flags; }
  CColor GetColor() const { return x4_color; }

  static CModelFlags Normal() { return CModelFlags(kT_Opaque, 1.f); }
  static CModelFlags AlphaBlended(f32 alpha) { return CModelFlags(kT_Blend, alpha); }
  static CModelFlags AlphaBlended(const CColor& color);
  static CModelFlags Additive(f32 f);
  static CModelFlags Additive(const CColor& color);
  static CModelFlags AdditiveRGB(const CColor& color);
  static CModelFlags ColorModulate(const CColor& color);

private:
  u8 x0_blendMode;
  u8 x1_matSetIdx;
  u16 x2_flags;
  CColor x4_color;
};
CHECK_SIZEOF(CModelFlags, 0x8)

#endif
