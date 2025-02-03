#ifndef _CPARTICLESWOOSH
#define _CPARTICLESWOOSH

#include "Kyoto/Particles/CParticleGen.hpp"
#include "Kyoto/Particles/IElement.hpp"
#include "Kyoto/TToken.hpp"

class CSwooshDescription;

class CParticleSwoosh : CParticleGen {
public:
  struct SSwooshData;

  explicit CParticleSwoosh(TToken< CSwooshDescription > desc);

  ~CParticleSwoosh() override;
  void Update(double) override;
  void Render() const override;
  void SetOrientation(const CTransform4f& orientation) override;
  void SetTranslation(const CVector3f& translation) override;
  void SetGlobalOrientation(const CTransform4f& orientation) override;
  void SetGlobalTranslation(const CVector3f& translation) override;
  void SetGlobalScale(const CVector3f& scale) override;
  void SetLocalScale(const CVector3f& scale) override;
  void SetParticleEmission(bool emission) override;
  void SetModulationColor(const CColor& col) override;
  void SetGeneratorRate(float rate) override;
  const CTransform4f& GetOrientation() const override;
  const CVector3f& GetTranslation() const override;
  CTransform4f GetGlobalOrientation() const override;
  CVector3f GetGlobalTranslation() const override;
  CVector3f GetGlobalScale() const override;
  bool GetParticleEmission() const override;
  CColor GetModulationColor() const override;
  bool IsSystemDeletable() const override;
  rstl::optional_object< CAABox > GetBounds() const override;
  int GetParticleCount() const override;
  bool SystemHasLight() const override;
  CLight GetLight() override;
  void DestroyParticles() override;
  void AddModifier(CWarp*) override;
  uint Get4CharId() const override;

  void SetDirty(bool val) { x1d0_26_isDirty = val; }

private:
  TCachedToken< CSwooshDescription > x1c_desc;
  uint x28_curFrame;
  int x2c_PSLT;
  double x30_curTime;
  CVector3f x38_translation;
  CTransform4f x44_orientation;
  CTransform4f x74_invOrientation;
  CVector3f xa4_globalTranslation;
  CTransform4f xb0_globalOrientation;
  CVector3f xe0_globalScale;
  CTransform4f xec_scaleXf;
  CTransform4f x11c_invScaleXf;
  CVector3f x14c_localScale;
  uint x158_curParticle;
  rstl::vector< SSwooshData > x15c_swooshes;
  rstl::vector< CVector3f > x16c_p0;
  rstl::vector< CVector3f > x17c_p1;
  rstl::vector< CVector3f > x18c_p2;
  rstl::vector< CVector3f > x19c_p3;
  uint x1ac_particleCount;
  int x1b0_SPLN;
  int x1b4_LENG;
  int x1b8_SIDE;
  GXPrimitive x1bc_prim; /* GX::Primitive */
  CRandom16 x1c0_rand;
  float x1c4_;
  float x1c8_;
  float x1cc_TSPN;
  // char x1d0_flags;
  bool x1d0_24_ : 1;
  bool x1d0_25_ : 1;
  bool x1d0_26_isDirty : 1;
  bool x1d0_27_ : 1;
  bool x1d0_28_ : 1;
  bool x1d0_29_ : 1;
  bool x1d0_30_ : 1;
  bool x1d0_31_ : 1;
  char x1d1_flags;
  char field32_0x1d2;
  char field33_0x1d3;
  SUVElementSet x1d4_uvs;
  CTexture* x1e4_tex;
  float x1e8_uvSpan;
  int x1ec_TSPN;
  CVector3f x1f0_aabbMin;
  CVector3f x1fc_aabbMax;
  float x208_maxRadius;
  CColor x20c_moduColor;
};
CHECK_SIZEOF(CParticleSwoosh, 0x210)

#endif // _CPARTICLESWOOSH
