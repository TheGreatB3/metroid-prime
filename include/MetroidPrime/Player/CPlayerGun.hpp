#ifndef _CPLAYERGUN_HPP
#define _CPLAYERGUN_HPP

#include "types.h"

#include "MetroidPrime/CActorLights.hpp"
#include "MetroidPrime/Player/CFidget.hpp"
#include "MetroidPrime/Player/CPlayerCameraBob.hpp"

#include "Kyoto/Audio/CSfxHandle.hpp"
#include "Kyoto/Math/CAABox.hpp"
#include "Kyoto/Math/CTransform4f.hpp"

#include "rstl/pair.hpp"
#include "rstl/reserved_vector.hpp"
#include "rstl/single_ptr.hpp"
#include "rstl/vector.hpp"

enum EChargeState {
  kCS_Normal,
  kCS_Charged,
};

class CGunWeapon;
class CGunMotion;
class CGrappleArm;
class CAuxWeapon;
class CRainSplashGenerator;
class CPowerBeam;
class CIceBeam;
class CWaveBeam;
class CPlasmaBeam;
class CPhazonBeam;
class CElementGen;
class CWorldShadow;

class CPlayerGun {
  static float skTractorBeamFactor;

public:
  enum EMissileMode {
    kMM_Inactive,
    kMM_Active,
  };
  enum EBWeapon {
    kBW_Bomb,
    kBW_PowerBomb,
  };
  enum EPhazonBeamState {
    kKBS_Inactive,
    kKBS_Entering,
    kKBS_Exiting,
    kKBS_Active,
  };
  enum EChargePhase {
    kCP_NotCharging,
    kCP_ChargeRequested,
    kCP_AnimAndSfx,
    kCP_FxGrowing,
    kCP_FxGrown,
    kCP_ComboXfer,
    kCP_ComboXferDone,
    kCP_ComboFire,
    kCP_ComboFireDone,
    kCP_ChargeCooldown,
    kCP_ChargeDone,
  };
  enum ENextState {
    kNS_StatusQuo,
    kNS_EnterMissile,
    kNS_ExitMissile,
    kNS_MissileShotDone,
    kNS_MissileReload,
    kNS_ChangeWeapon,
    kNS_SetupBeam,
    kNS_Seven,
    kNS_EnterPhazonBeam,
    kNS_ExitPhazonBeam,
  };
  enum EIdleState {
    kIS_NotIdle,
    kIS_Wander,
    kIS_Idle,
    kIS_Three,
    kIS_Four,
  };

  bool IsCharging() const { return x834_24_charging; }
  float GetChargeBeamFactor() const { return x834_24_charging ? x340_chargeBeamFactor : 0.f; }

  static float GetTractorBeamFactor() { return skTractorBeamFactor; }

private:
  class CGunMorph {
  public:
    enum EGunState {
      kGS_InWipeDone,
      kGS_OutWipeDone,
      kGS_InWipe,
      kGS_OutWipe,
    };
    enum EMorphEvent {
      kME_None,
      kME_InWipeDone,
      kME_OutWipeDone,
    };
    enum EDir {
      kD_In,
      kD_Out,
      kD_Done,
    };

    CGunMorph(float gunTransformTime, float holoHoldTime)
    : x4_gunTransformTime(gunTransformTime), x10_holoHoldTime(fabs(holoHoldTime)) {}

    EMorphEvent Update(float inY, float outY, float dt);
    void StartWipe(EDir dir);

    float GetYLerp() const { return x0_yLerp; }
    float GetTransitionFactor() const { return x18_transitionFactor; }
    EGunState GetGunState() const { return x20_gunState; }
    void SetWeaponChanged() { x24_25_weaponChanged = true; }

  private:
    float x0_yLerp;
    float x4_gunTransformTime;
    float x8_remTime;
    float xc_speed;
    float x10_holoHoldTime;
    float x14_remHoldTime;
    float x18_transitionFactor;
    EDir x1c_dir;
    EGunState x20_gunState;
    bool x24_24_morphing : 1;
    bool x24_25_weaponChanged : 1;
  };

  class CMotionState {
  public:
    enum EMotionState {
      kMS_Zero,
      kMS_One,
      kMS_LockOn,
      kMS_CancelLockOn,
    };
    enum EFireState {
      kFS_NotFiring,
      kFS_StartFire,
      kFS_Firing,
    };

    static void SetExtendDistance(float d) { kGunExtendDistance = d; }

    void SetState(EMotionState state) { x20_state = state; }
    void Update(bool firing, float dt, CTransform4f& xf, CStateManager& mgr);

  private:
    bool x0_24_extendParabola : 1;
    float x4_extendParabolaDelayTimer;
    float x8_fireTime;
    float xc_curExtendDist;
    float x10_curRotation;
    float x14_rotationT;
    float x18_startRotation;
    float x1c_endRotation;
    EMotionState x20_state;
    EFireState x24_fireState;

    static float kGunExtendDistance;
  };

  CActorLights x0_lights;
  CSfxHandle x2e0_chargeSfx;
  CSfxHandle x2e4_invalidSfx;
  CSfxHandle x2e8_phazonBeamSfx;
  // 0x1: FireOrBomb, 0x2: MissileOrPowerBomb
  uint x2ec_lastFireButtonStates;
  uint x2f0_pressedFireButtonStates;
  uint x2f4_fireButtonStates;
  // 0x1: beam mode, 0x2: missile mode, 0x4: missile ready, 0x8: morphing, 0x10: combo fire
  uint x2f8_stateFlags;
  uint x2fc_fidgetAnimBits;
  uint x300_remainingMissiles;
  uint x304_;
  uint x308_bombCount;
  uint x30c_rapidFireShots;
  CPlayerState::EBeamId x310_currentBeam;
  CPlayerState::EBeamId x314_nextBeam;
  uint x318_comboAmmoIdx;
  EMissileMode x31c_missileMode;
  CPlayerState::EBeamId x320_currentAuxBeam;
  EIdleState x324_idleState;
  float x328_animSfxPitch;
  EChargePhase x32c_chargePhase;
  EChargeState x330_chargeState;
  uint x334_;
  ENextState x338_nextState;
  EPhazonBeamState x33c_phazonBeamState;
  float x340_chargeBeamFactor;
  float x344_comboXferTimer;
  float x348_chargeCooldownTimer;
  float x34c_shakeX;
  float x350_shakeZ;
  float x354_bombFuseTime;
  float x358_bombDropDelayTime;
  float x35c_bombTime;
  float x360_;
  float x364_gunStrikeCoolTimer;
  float x368_idleWanderDelayTimer;
  float x36c_;
  float x370_gunMotionSpeedMult;
  float x374_;
  float x378_shotSmokeStartTimer;
  float x37c_rapidFireShotsDecayTimer;
  float x380_shotSmokeTimer;
  float x384_gunStrikeDelayTimer;
  float x388_enterFreeLookDelayTimer;
  float x38c_muzzleEffectVisTimer;
  float x390_cooldown;
  float x394_damageTimer;
  float x398_damageAmt;
  float x39c_phazonMorphT;
  float x3a0_missileExitTimer;
  CFidget x3a4_fidget;
  CVector3f x3dc_damageLocation;
  CTransform4f x3e8_xf;
  CTransform4f x418_beamLocalXf;
  CTransform4f x448_elbowWorldXf;
  CTransform4f x478_assistAimXf;
  CTransform4f x4a8_gunWorldXf;
  CTransform4f x4d8_gunLocalXf;
  CTransform4f x508_elbowLocalXf;
  TUniqueId x538_playerId;
  TUniqueId x53a_powerBomb;
  TUniqueId x53c_lightId;
  rstl::vector< CToken > x540_handAnimTokens;
  CPlayerCameraBob x550_camBob;
  uint x658_;
  float x65c_;
  float x660_;
  float x664_;
  float x668_aimVerticalSpeed;
  float x66c_aimHorizontalSpeed;
  rstl::pair< u16, CSfxHandle > x670_animSfx;
  CGunMorph x678_morph;
  CMotionState x6a0_motionState;
  CAABox x6c8_hologramClipCube;
  CModelData x6e0_rightHandModel;
  CGunWeapon* x72c_currentBeam;
  CGunWeapon* x730_outgoingBeam;
  CGunWeapon* x734_loadingBeam;
  CGunWeapon* x738_nextBeam;
  rstl::single_ptr< CGunMotion > x73c_gunMotion;
  rstl::single_ptr< CGrappleArm > x740_grappleArm;
  rstl::single_ptr< CAuxWeapon > x744_auxWeapon;
  rstl::single_ptr< CRainSplashGenerator > x748_rainSplashGenerator;
  rstl::single_ptr< CPowerBeam > x74c_powerBeam;
  rstl::single_ptr< CIceBeam > x750_iceBeam;
  rstl::single_ptr< CWaveBeam > x754_waveBeam;
  rstl::single_ptr< CPlasmaBeam > x758_plasmaBeam;
  rstl::single_ptr< CPhazonBeam > x75c_phazonBeam;
  rstl::reserved_vector< CGunWeapon*, 4 > x760_selectableBeams;
  rstl::auto_ptr< CElementGen > x774_holoTransitionGen;
  rstl::auto_ptr< CElementGen > x77c_comboXferGen;
  rstl::reserved_vector< rstl::reserved_vector< TLockedToken< CGenDescription >, 2 >, 2 >
      x784_bombEffects;
  rstl::reserved_vector< TLockedToken< CGenDescription >, 5 > x7c0_auxMuzzleEffects;
  rstl::reserved_vector< rstl::auto_ptr< CElementGen >, 5 > x800_auxMuzzleGenerators;
  rstl::single_ptr< CWorldShadow > x82c_shadow;
  s16 x830_chargeRumbleHandle;

  bool x832_24_coolingCharge : 1;
  bool x832_25_chargeEffectVisible : 1;
  bool x832_26_comboFiring : 1;
  bool x832_27_chargeAnimStarted : 1;
  bool x832_28_readyForShot : 1;
  bool x832_29_lockedOn : 1;
  bool x832_30_requestReturnToDefault : 1;
  bool x832_31_inRestPose : 1;

  bool x833_24_notFidgeting : 1;
  bool x833_25_ : 1;
  bool x833_26_ : 1;
  bool x833_27_ : 1;
  bool x833_28_phazonBeamActive : 1;
  bool x833_29_pointBlankWorldSurface : 1;
  bool x833_30_canShowAuxMuzzleEffect : 1;
  bool x833_31_inFreeLook : 1;

  bool x834_24_charging : 1;
  bool x834_25_gunMotionFidgeting : 1;
  bool x834_26_animPlaying : 1;
  bool x834_27_underwater : 1;
  bool x834_28_requestImmediateRecharge : 1;
  bool x834_29_frozen : 1;
  bool x834_30_inBigStrike : 1;
  bool x834_31_gunMotionInFidgetBasePosition : 1;

  bool x835_24_canFirePhazon : 1;
  bool x835_25_inPhazonBeam : 1;
  bool x835_26_phazonBeamMorphing : 1;
  bool x835_27_intoPhazonBeam : 1;
  bool x835_28_bombReady : 1;
  bool x835_29_powerBombReady : 1;
};
CHECK_SIZEOF(CPlayerGun, 0x838)

#endif // _CPLAYERGUN_HPP
