#include "MetroidPrime/Player/CPlayer.hpp"

#include "MetroidPrime/CActorParameters.hpp"
#include "MetroidPrime/CControlMapper.hpp"
#include "MetroidPrime/CFluidPlaneCPU.hpp"
#include "MetroidPrime/CGameCollision.hpp"
#include "MetroidPrime/CRumbleManager.hpp"
#include "MetroidPrime/CWorld.hpp"
#include "MetroidPrime/Cameras/CBallCamera.hpp"
#include "MetroidPrime/Cameras/CCameraManager.hpp"
#include "MetroidPrime/Cameras/CCinematicCamera.hpp"
#include "MetroidPrime/Cameras/CFirstPersonCamera.hpp"
#include "MetroidPrime/HUD/CSamusHud.hpp"
#include "MetroidPrime/Player/CGameState.hpp"
#include "MetroidPrime/Player/CGrappleArm.hpp"
#include "MetroidPrime/Player/CMorphBall.hpp"
#include "MetroidPrime/Player/CPlayerCameraBob.hpp"
#include "MetroidPrime/Player/CPlayerGun.hpp"
#include "MetroidPrime/SFX/IceCrack.h"
#include "MetroidPrime/SFX/LavaWorld.h"
#include "MetroidPrime/SFX/MiscSamus.h"
#include "MetroidPrime/ScriptObjects/CHUDBillboardEffect.hpp"
#include "MetroidPrime/ScriptObjects/CScriptWater.hpp"
#include "MetroidPrime/Tweaks/CTweakPlayer.hpp"
#include "MetroidPrime/Tweaks/CTweakPlayerGun.hpp"
#include "MetroidPrime/Tweaks/CTweakPlayerRes.hpp"
#include "MetroidPrime/Weapons/CEnergyProjectile.hpp"

#include "Kyoto/Audio/CSfxManager.hpp"
#include "Kyoto/Audio/CStreamAudioManager.hpp"
#include "Kyoto/CSimplePool.hpp"
#include "Kyoto/Input/CRumbleVoice.hpp"
#include "Kyoto/Math/CRelAngle.hpp"
#include "Kyoto/Math/CloseEnough.hpp"
#include "Kyoto/Text/CStringTable.hpp"

#include "Collision/CCollisionInfo.hpp"
#include "Collision/CRayCastResult.hpp"

#include "WorldFormat/CMetroidAreaCollider.hpp"

#include "rstl/math.hpp"

static CVector3f testRayStart(0.f, 0.f, 0.f);
static CVector3f testRayNormal(0.f, 0.f, 1.f);
static CRayCastResult testRayResult;
static CCollisionInfo testBoxResult;
static CAABox testBox(CAABox::Identity());
typedef rstl::pair< CPlayerState::EItemType, ControlMapper::ECommands > TVisorToItemMapping;
static TVisorToItemMapping skVisorToItemMapping[4] = {
    TVisorToItemMapping(CPlayerState::kIT_CombatVisor, ControlMapper::kC_NoVisor),
    TVisorToItemMapping(CPlayerState::kIT_XRayVisor, ControlMapper::kC_XrayVisor),
    TVisorToItemMapping(CPlayerState::kIT_ThermalVisor, ControlMapper::kC_EnviroVisor),
    TVisorToItemMapping(CPlayerState::kIT_ScanVisor, ControlMapper::kC_ThermoVisor),
};

static const ushort skPlayerLandSfxSoft[24] = {
    0xFFFF,
    SFXsam_landstone_00,
    SFXsam_landmetl_00,
    SFXsam_landgrass_00,
    SFXsam_landice_00,
    0xFFFF,
    SFXsam_landgrate_00,
    SFXsam_landphazon_00,
    SFXsam_landdirt_00,
    SFXlav_landlava_00,
    SFXsam_landlavastone_00,
    SFXsam_landsnow_00,
    SFXsam_landmud_00,
    0xFFFF,
    SFXsam_landgrass_00,
    SFXsam_landmetl_00,
    SFXsam_landmetl_00,
    SFXsam_landdirt_00,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    SFXsam_landwood_00,
    SFXsam_b_landorg_00,
};

static const ushort skPlayerLandSfxHard[24] = {
    0xFFFF,
    SFXsam_landstone_02,
    SFXsam_b_landmetl_02,
    SFXsam_landgrass_02,
    SFXsam_landice_02,
    0xFFFF,
    SFXsam_landgrate_02,
    SFXsam_landphazon_02,
    SFXsam_landdirt_02,
    SFXlav_landlava_02,
    SFXsam_landlavastone_02,
    SFXsam_landsnow_02,
    SFXsam_landmud_02,
    0xFFFF,
    SFXsam_landgrass_02,
    SFXsam_b_landmetl_02,
    SFXsam_b_landmetl_02,
    SFXsam_landdirt_02,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    SFXsam_landwood_02,
    SFXsam_landorg_02,
};

static const ushort skLeftStepSounds[24] = {
    0xFFFF,
    SFXsam_wlkstone_00,
    SFXsam_wlkmetal_00,
    SFXsam_b_wlkgrass_00,
    SFXsam_wlkice_00,
    0xFFFF,
    SFXsam_wlkgrate_00,
    SFXsam_wlkphazon_00,
    SFXsam_wlkdirt_00,
    SFXlav_wlklava_00,
    SFXsam_wlklavastone_00,
    SFXsam_wlksnow_00,
    SFXsam_wlkmud_00,
    0xFFFF,
    SFXsam_b_wlkorg_00,
    SFXsam_wlkmetal_00,
    SFXsam_wlkmetal_00,
    SFXsam_wlkdirt_00,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    SFXsam_wlkwood_00,
    SFXsam_b_wlkorg_00,
};

static const ushort skRightStepSounds[24] = {
    0xFFFF,
    SFXsam_wlkstone_01,
    SFXsam_wlkmetal_01,
    SFXsam_b_wlkgrass_01,
    SFXsam_wlkice_01,
    0xFFFF,
    SFXsam_wlkgrate_01,
    SFXsam_wlkphazon_01,
    SFXsam_wlkdirt_01,
    SFXlav_wlklava_01,
    SFXsam_wlklavastone_01,
    SFXsam_wlksnow_01,
    SFXsam_wlkmud_01,
    0xFFFF,
    SFXsam_b_wlkorg_01,
    SFXsam_wlkmetal_01,
    SFXsam_wlkmetal_01,
    SFXsam_wlkdirt_01,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    0xFFFF,
    SFXsam_wlkwood_01,
    SFXsam_b_wlkorg_01,
};

static const char* const kGunLocator = "GUN_LCTR";

static bool gUseSurfaceHack;
static CPlayer::ESurfaceRestraints gSR_Hack;

CAnimRes MakePlayerAnimres(CAssetId resId, const CVector3f& scale) {
  return CAnimRes(resId, CAnimRes::kDefaultCharIdx, scale, 0, true);
}

CPlayer::CPlayer(TUniqueId uid, const CTransform4f& xf, const CAABox& aabb, CAssetId resId,
                 const CVector3f& playerScale, float mass, float stepUp, float stepDown,
                 float ballRadius, const CMaterialList& ml)
: CPhysicsActor(uid, true, rstl::string_l("CPlayer"),
                CEntityInfo(kInvalidAreaId, CEntity::NullConnectionList), xf,
                MakePlayerAnimres(resId, playerScale), ml, aabb, SMoverData(mass),
                CActorParameters::None(), stepUp, stepDown)
, x258_movementState(NPlayer::kMS_OnGround)
, x25c_ballTransitionsRes()
, x26c_attachedActor(kInvalidUniqueId)
, x270_attachedActorTime(0.f)
, x274_energyDrain(4)
, x288_startingJumpTimeout(0.f)
, x28c_sjTimer(0.f)
, x290_minJumpTimeout(0.f)
, x294_jumpCameraTimer(0.f)
, x298_jumpPresses(0)
, x29c_fallCameraTimer(0.f)
, x2a0_(0.f)
, x2a4_cancelCameraPitch(false)
, x2a8_timeSinceJump(1000.f)
, x2ac_surfaceRestraint(kSR_Normal)
, x2b0_outOfWaterTicks(2)
, x2b4_accelerationTable()
, x2d0_curAcceleration(1)
, x2d4_accelerationChangeTimer(0.f)
, x2d8_fpBounds(aabb)
, x2f0_ballTransHeight(1.f)
, x2f4_cameraState(kCS_FirstPerson)
, x2f8_morphBallState(kMS_Unmorphed)
, x2fc_spawnedMorphBallState(kMS_Unmorphed)
, x300_fallingTime(0.f)
, x304_orbitState(kOS_NoOrbit)
, x308_orbitType(kOT_Close)
, x30c_orbitBrokenType(kOB_Default)
, x310_orbitTargetId(kInvalidUniqueId)
, x314_orbitPoint(0.f, 0.f, 0.f)
, x320_orbitVector(0.f, 0.f, 0.f)
, x32c_orbitModeTimer(0.f)
, x330_orbitZoneMode(kZI_Targeting)
, x334_orbitType(kZT_Ellipse)
, x338_(1)
, x33c_orbitNextTargetId(kInvalidUniqueId)
, x340_(0.f)
, x344_nearbyOrbitObjects()
, x354_onScreenOrbitObjects()
, x364_offScreenOrbitObjects()
, x374_orbitLockEstablished(false)
, x378_orbitPreventionTimer(0.f)
, x37c_sidewaysDashing(false)
, x380_strafeInputAtDash(0.f)
, x384_dashTimer(0.f)
, x388_dashButtonHoldTime(0.f)
, x38c_doneSidewaysDashing(false)
, x390_orbitSource(2)
, x394_orbitingEnemy(false)
, x398_dashSpeedMultiplier(1.5f)
, x39c_noStrafeDashBlend(false)
, x3a0_dashDuration(0.5f)
, x3a4_strafeDashBlendDuration(0.449f)
, x3a8_scanState(kSS_NotScanning)
, x3ac_scanningTime(0.f)
, x3b0_curScanTime(0.f)
, x3b4_scanningObject(kInvalidUniqueId)
, x3b8_grappleState(kGS_None)
, x3bc_grappleSwingTimer(0.f)
, x3c0_grappleSwingAxis(0.f, 1.f, 0.f)
, x3cc_(0.f)
, x3d0_(0.f)
, x3d4_(0.f)
, x3d8_grappleJumpTimeout(0.f)
, x3dc_inFreeLook(false)
, x3dd_lookButtonHeld(false)
, x3de_lookAnalogHeld(false)
, x3e0_curFreeLookCenteredTime(0.f)
, x3e4_freeLookYawAngle(0.f)
, x3e8_horizFreeLookAngleVel(0.f)
, x3ec_freeLookPitchAngle(0.f)
, x3f0_vertFreeLookAngleVel(0.f)
, x3f4_aimTarget(kInvalidUniqueId)
, x3f8_targetAimPosition(CVector3f::Zero())
, x404_aimTargetAverage()
, x480_assistedTargetAim(CVector3f::Zero())
, x48c_aimTargetTimer(0.f)
, x490_gun(rs_new CPlayerGun(uid))
, x494_gunAlpha(1.f)
, x498_gunHolsterState(kGH_Drawn)
, x49c_gunHolsterRemTime(gpTweakPlayerGun->x40_gunNotFiringTime)
, x4a0_inputFilter(rs_new CInputFilter())
, x4a4_moveSpeedAvg()
, x4f8_moveSpeed(0.f)
, x4fc_flatMoveSpeed(0.f)
, x500_lookDir(GetTransform().GetColumn(kDY))
, x50c_moveDir(GetTransform().GetColumn(kDY))
, x518_leaveMorphDir(GetTransform().GetColumn(kDY))
, x524_lastPosForDirCalc(GetTransform().GetTranslation())
, x530_gunDir(GetTransform().GetColumn(kDY))
, x53c_timeMoving(0.f)
, x540_controlDir(GetTransform().GetColumn(kDY))
, x54c_controlDirFlat(GetTransform().GetColumn(kDY))
, x558_wasDamaged(false)
, x55c_damageAmt(0.f)
, x560_prevDamageAmt(0.f)
, x564_damageLocation(CVector3f::Zero())
, x570_immuneTimer(0.f)
, x574_morphTime(0.f)
, x578_morphDuration(0.f)
, x57c_(0)
, x580_(0)
, x584_ballTransitionAnim(-1)
, x588_alpha(1.f)
, x58c_transitionVel(0.f)
, x590_leaveMorphballAllowed(true)
, x594_transisionBeamXfs()
, x658_transitionModelXfs()
, x71c_transitionModelAlphas()
, x730_transitionModels()
, x740_staticTimer(0.f)
, x744_staticOutSpeed(0.f)
, x748_staticInSpeed(0.f)
, x74c_visorStaticAlpha(1.f)
, x750_frozenTimeout(0.f)
, x754_iceBreakJumps(0)
, x758_frozenTimeoutBias(0.f)
, x75c_additionalIceBreakJumps(0)
, x760_controlsFrozen(false)
, x764_controlsFrozenTimeout(0.f)
, x768_morphball()
, x76c_cameraBob(rs_new CPlayerCameraBob(CPlayerCameraBob::kCBT_One))
, x770_damageLoopSfx()
, x774_samusVoiceTimeout(0.f)
, x778_dashSfx()
, x77c_samusVoiceSfx()
, x780_samusVoicePriority(0)
, x784_damageSfxTimer(0.f)
, x788_damageLoopSfxId(0)
, x78c_footstepSfxTimer(0.f)
, x790_footstepSfxSel(kFS_None)
, x794_lastVelocity(CVector3f::Zero())
, x7a0_visorSteam(0.f, 0.f, 0.f, kInvalidAssetId)
, x7cc_transitionSuit(CPlayerState::kPS_Invalid)
, x7d0_animRes(resId, CAnimRes::kDefaultCharIdx, playerScale, 0, true)
, x7ec_beam(CPlayerState::kBI_Power)
, x7f0_ballTransitionBeamModel()
, x7f4_gunWorldXf(CTransform4f::Identity())
, x824_transitionFilterTimer(0.f)
, x828_distanceUnderWater(0.f)
, x82c_inLava(false)
, x82e_ridingPlatform(kInvalidUniqueId)
, x830_playerHint(kInvalidUniqueId)
, x834_playerHintPriority(1000)
, x838_playerHints()
, x93c_playerHintsToRemove()
, x980_playerHintsToAdd()
, x9c4_24_visorChangeRequested(false)
, x9c4_25_showCrosshairs(false)
, x9c4_26_(true)
, x9c4_27_canEnterMorphBall(true)
, x9c4_28_canLeaveMorphBall(true)
, x9c4_29_spiderBallControlXY(false)
, x9c4_30_controlDirOverride(false)
, x9c4_31_inWaterMovement(false)
, x9c5_24_(false)
, x9c5_25_splashUpdated(false)
, x9c5_26_(false)
, x9c5_27_camSubmerged(false)
, x9c5_28_slidingOnWall(false)
, x9c5_29_hitWall(false)
#if NONMATCHING
, x9c5_30_selectFluidBallSound(false)
#endif
, x9c5_31_stepCameraZBiasDirty(true)
, x9c6_24_extendTargetDistance(false)
, x9c6_25_interpolatingControlDir(false)
, x9c6_26_outOfBallLookAtHint(false)
, x9c6_27_aimingAtProjectile(false)
, x9c6_28_aligningGrappleSwingTurn(false)
, x9c6_29_disableInput(false)
#if NONMATCHING
, x9c6_30_newScanScanning(false)
#endif
, x9c6_31_overrideRadarRadius(false)
, x9c7_25_outOfBallLookAtHintActor(false)
#if NONMATCHING
, x9c7_24_noDamageLoopSfx(false)
#endif
, x9c8_eyeZBias(0.f)
, x9cc_stepCameraZBias(0.f)
, x9d0_bombJumpCount(0)
, x9d4_bombJumpCheckDelayFrames(0)
, x9d8_controlDirOverrideDir(0.f, 1.f, 0.f)
, x9e4_orbitDisableList()
, x9f4_deathTime(0.f)
, x9f8_controlDirInterpTime(0.f)
, x9fc_controlDirInterpDur(0.f)
, xa00_deathPowerBomb(kInvalidUniqueId)
, xa04_preThinkDt(0.f)
, xa08_steamTextureId(kInvalidAssetId)
#if NONMATCHING
, xa0c_iceTextureId(kInvalidAssetId)
#endif
, xa10_envDmgCounter(0)
, xa14_envDmgCameraShakeTimer(0.f)
, xa18_phazonDamageLag(0.f)
, xa1c_threatOverride(0.f)
, xa20_radarXYRadiusOverride(1.f)
, xa24_radarZRadiusOverride(1.f)
, xa28_attachedActorStruggle(0.f)
, xa2c_damageLoopSfxDelayTicks(2)
, xa30_samusExhaustedVoiceTimer(4.f) {
  CModelData ballTransitionBeamModelData(
      CStaticRes(gpTweakPlayerRes->GetBallTransitionBeamResId(x7ec_beam), playerScale));
  CModelData* ptr;
  if (ballTransitionBeamModelData.IsNull()) {
    ptr = nullptr;
  } else {
    ptr = rs_new CModelData(ballTransitionBeamModelData);
  }
  x7f0_ballTransitionBeamModel = ptr;
  x730_transitionModels.reserve(3);
  x768_morphball = rs_new CMorphBall(*this, ballRadius);
  SetInertiaTensorScalar(GetMass());
  SetLastNonCollidingState(GetMotionState());
  x490_gun->SetTransform(GetTransform());
  x490_gun->GetGrappleArm().SetTransform(GetTransform());
  InitialiseAnimation();
  CAABox bounds = GetModelData()->GetBounds(CTransform4f::Identity());
  x2f0_ballTransHeight = bounds.GetMaxPoint().GetZ() - bounds.GetMinPoint().GetZ();
  SetCalculateLighting(true);
  ActorLights()->SetCastShadows(true);
  x50c_moveDir.SetZ(0.f);
  if (x50c_moveDir.CanBeNormalized()) {
    x50c_moveDir.Normalize();
  }
  x2b4_accelerationTable.push_back(20.f);
  x2b4_accelerationTable.push_back(80.f);
  x2b4_accelerationTable.push_back(80.f);
  x2b4_accelerationTable.push_back(270.f);
  SetMaxVelocityAfterCollision(25.f);
  x354_onScreenOrbitObjects.reserve(64);
  x344_nearbyOrbitObjects.reserve(64);
  x364_offScreenOrbitObjects.reserve(64);
  ModelData()->SetScale(playerScale);
  x7f0_ballTransitionBeamModel->SetScale(playerScale);
  LoadAnimationTokens();
}

bool CPlayer::IsMorphBallTransitioning() const {
  switch (x2f8_morphBallState) {
  case kMS_Morphing:
  case kMS_Unmorphing:
    return true;
  default:
    return false;
  }
}

void CPlayer::HolsterGun(CStateManager& mgr) {
  if (x498_gunHolsterState == kGH_Holstered || x498_gunHolsterState == kGH_Holstering) {
    return;
  }

  float time = gpTweakPlayerGun->x3c_gunHolsterTime;
  if (x2f8_morphBallState == kMS_Morphing) {
    time = 0.1f;
  }
  if (x498_gunHolsterState == kGH_Drawing) {
    x49c_gunHolsterRemTime = time * (1.f - x49c_gunHolsterRemTime / 0.45f);
  } else {
    x49c_gunHolsterRemTime = time;
  }

  x498_gunHolsterState = kGH_Holstering;
  x490_gun->CancelFiring(mgr);
  SetAimTargetId(kInvalidUniqueId);
}

void CPlayer::ResetGun(CStateManager& mgr) {
  x498_gunHolsterState = kGH_Holstered;
  x49c_gunHolsterRemTime = 0.f;
  x490_gun->CancelFiring(mgr);
  SetAimTargetId(kInvalidUniqueId);
}

void CPlayer::DrawGun(CStateManager& mgr) {
  if (x498_gunHolsterState != kGH_Holstered || CheckPostGrapple()) {
    return;
  }

  x498_gunHolsterState = kGH_Drawing;
  x49c_gunHolsterRemTime = 0.45f;
  x490_gun->ResetIdle(mgr);
}

void CPlayer::UpdateGunState(const CFinalInput& input, CStateManager& mgr) {
  float dt = input.Time();
  switch (x498_gunHolsterState) {
  case kGH_Drawn: {
    bool needsHolster = false;
    if (gpTweakPlayer->mGunButtonTogglesHolster) {
      if (ControlMapper::GetPressInput(ControlMapper::kC_ToggleHolster, input)) {
        needsHolster = true;
      }
      if (!ControlMapper::GetDigitalInput(ControlMapper::kC_FireOrBomb, input) &&
          !ControlMapper::GetDigitalInput(ControlMapper::kC_MissileOrPowerBomb, input) &&
          gpTweakPlayer->mGunNotFiringHolstersGun) {
        x49c_gunHolsterRemTime -= dt;
        if (x49c_gunHolsterRemTime <= 0.f) {
          needsHolster = true;
        }
      }
    } else {
      if (!ControlMapper::GetDigitalInput(ControlMapper::kC_FireOrBomb, input) &&
          !ControlMapper::GetDigitalInput(ControlMapper::kC_MissileOrPowerBomb, input) &&
          x490_gun->IsFidgeting()) {
        if (gpTweakPlayer->mGunNotFiringHolstersGun) {
          x49c_gunHolsterRemTime -= dt;
        }
      } else {
        x49c_gunHolsterRemTime = gpTweakPlayerGun->x40_gunNotFiringTime;
      }
    }

    if (needsHolster) {
      HolsterGun(mgr);
    }
    break;
  }
  case kGH_Drawing: {
    if (x49c_gunHolsterRemTime > 0.f) {
      x49c_gunHolsterRemTime -= dt;
    } else {
      x498_gunHolsterState = kGH_Drawn;
      x49c_gunHolsterRemTime = gpTweakPlayerGun->x40_gunNotFiringTime;
    }
    break;
  }
  case kGH_Holstered: {
    bool needsDraw = false;
    if (ControlMapper::GetDigitalInput(ControlMapper::kC_FireOrBomb, input) ||
        ControlMapper::GetDigitalInput(ControlMapper::kC_MissileOrPowerBomb, input) ||
        x3b8_grappleState == kGS_None) {
      needsDraw = true;
    } else if (gpTweakPlayer->mGunButtonTogglesHolster &&
               ControlMapper::GetPressInput(ControlMapper::kC_ToggleHolster, input)) {
      needsDraw = true;
    }

    if (x3b8_grappleState != kGS_None ||
        mgr.GetPlayerState()->GetCurrentVisor() == CPlayerState::kPV_Scan ||
        mgr.GetPlayerState()->GetTransitioningVisor() == CPlayerState::kPV_Scan) {
      needsDraw = false;
    }

    if (needsDraw) {
      DrawGun(mgr);
    }
    break;
  }
  case kGH_Holstering:
    if (x49c_gunHolsterRemTime > 0.f) {
      x49c_gunHolsterRemTime -= dt;
    } else {
      x498_gunHolsterState = kGH_Holstered;
    }
    break;
  default:
    break;
  }
}

void CPlayer::SetAimTargetId(TUniqueId target) {
  if (target == kInvalidUniqueId || x3f4_aimTarget != target) {
    x404_aimTargetAverage.clear();
  }
  x3f4_aimTarget = target;
}

void CPlayer::UpdateAimTargetPrediction(const CTransform4f& xf, CStateManager& mgr) {
  if (GetAimTargetId() == kInvalidUniqueId) {
    return;
  }

  const CActor* target = TCastToConstPtr< CActor >(mgr.GetObjectById(GetAimTargetId()));
  if (!target) {
    return;
  }

  x9c6_27_aimingAtProjectile = TCastToConstPtr< CGameProjectile >(target) != nullptr;
  CVector3f instantTarget = target->GetAimPosition(mgr, 0.f);
  CVector3f gunToTarget = instantTarget - xf.GetTranslation();
  float timeToTarget = gunToTarget.Magnitude() / x490_gun->GetBeamVelocity();
  CVector3f predictTarget = target->GetAimPosition(mgr, timeToTarget);
  CVector3f predictOffset = predictTarget - instantTarget;
  x3f8_targetAimPosition = instantTarget;

  if (predictOffset.Magnitude() < 0.1f) {
    x404_aimTargetAverage.AddValue(CVector3f::Zero());
  } else {
    x404_aimTargetAverage.AddValue(predictTarget - instantTarget);
  }

  if (x404_aimTargetAverage.GetAverage() && !x9c6_27_aimingAtProjectile) {
    x480_assistedTargetAim = instantTarget + *x404_aimTargetAverage.GetAverage();
  } else {
    x480_assistedTargetAim = predictTarget;
  }
}

void CPlayer::UpdateAssistedAiming(const CTransform4f& xf, CStateManager& mgr) {
  CTransform4f assistXf = xf;
  if (const CActor* target = TCastToConstPtr< CActor >(mgr.GetObjectById(GetAimTargetId()))) {
    CVector3f gunToTarget = x480_assistedTargetAim - xf.GetTranslation();
    CVector3f gunToTargetFlat = gunToTarget.DropZ();
    float gunToTargetFlatMag = gunToTargetFlat.Magnitude();
    CVector3f gunDirFlat = xf.GetColumn(kDY);
    gunDirFlat.SetZ(0.f);
    float gunDirFlatMag = gunDirFlat.Magnitude();
    if (gunToTargetFlat.CanBeNormalized() && gunDirFlat.CanBeNormalized()) {
      gunToTargetFlat /= gunToTargetFlatMag;
      gunDirFlat /= gunDirFlatMag;
      float az1 = atan2f(gunToTarget.GetZ(), gunToTargetFlatMag);
      float az2 = atan2f(xf.GetColumn(kDY).GetZ(), gunDirFlatMag);
      float vAngleDelta = az1 - az2;
      bool hasVAngleDelta = true;
      if (!x9c6_27_aimingAtProjectile &&
          fabsf(vAngleDelta) > gpTweakPlayer->mAimAssistVerticalAngle) {
        if (gpTweakPlayer->mAssistedAimingIgnoreVertical) {
          vAngleDelta = 0.f;
          hasVAngleDelta = false;
        } else if (vAngleDelta > 0.f) {
          vAngleDelta = gpTweakPlayer->mAimAssistVerticalAngle;
        } else {
          vAngleDelta = -gpTweakPlayer->mAimAssistVerticalAngle;
        }
      }

      bool targetToLeft = CVector3f::Cross(gunDirFlat, gunToTargetFlat).GetZ() > 0.f;
      float hAngleDelta = acosf(CMath::Limit(CVector3f::Dot(gunToTargetFlat, gunDirFlat), 1.f));
      bool hasHAngleDelta = true;
      if (!x9c6_27_aimingAtProjectile &&
          fabsf(hAngleDelta) > gpTweakPlayer->mAimAssistHorizontalAngle) {
        hAngleDelta = gpTweakPlayer->mAimAssistHorizontalAngle;
        if (gpTweakPlayer->mAssistedAimingIgnoreHorizontal) {
          hAngleDelta = 0.f;
          hasHAngleDelta = false;
        }
      }

      if (targetToLeft) {
        hAngleDelta = -hAngleDelta;
      }
      if (!hasVAngleDelta || !hasHAngleDelta) {
        vAngleDelta = 0.f;
        hAngleDelta = 0.f;
      }

      gunToTarget = CVector3f(sinf(hAngleDelta) * cosf(vAngleDelta),
                              cosf(hAngleDelta) * cosf(vAngleDelta), sinf(vAngleDelta));
      gunToTarget = xf.Rotate(gunToTarget);
      assistXf = CTransform4f::LookAt(CVector3f::Zero(), gunToTarget);
    }
  }

  x490_gun->SetAssistAimTransform(assistXf);
}

void CPlayer::UpdateGunTransform(const CVector3f& gunPos, CStateManager& mgr) {
#if !NONMATCHING
  CTransform4f xf = GetTransform();
#endif
  float eyeHeight = GetEyeHeight();
  CTransform4f camXf = mgr.GetCameraManager()->GetCurrentCameraTransform(mgr);
  CTransform4f gunXf = camXf;

  CVector3f viewGunPos;
  if (x2f8_morphBallState == kMS_Morphing) {
    viewGunPos = camXf * CVector3f(gunPos - CVector3f(0.f, 0.f, eyeHeight));
  } else {
    viewGunPos = GetEyePosition() + camXf.Rotate(gunPos - CVector3f(0.f, 0.f, eyeHeight));
  }
  gunXf.SetTranslation(viewGunPos);

  CUnitVector3f rightDir(camXf.GetColumn(kDX));
  switch (x498_gunHolsterState) {
  case kGH_Drawing: {
    float liftAngle = CMath::Limit(x49c_gunHolsterRemTime / 0.45f, 1.f);
    if (liftAngle > 0.01f) {
      CQuaternion quat = CQuaternion::AxisAngle(
          rightDir, CRelAngle::FromRadians(-liftAngle * gpTweakPlayerGun->x44_fixedVerticalAim));
      gunXf = quat.BuildTransform4f() * camXf.GetRotation();
      gunXf.SetTranslation(viewGunPos);
    }
    break;
  }
  case kGH_Holstered: {
    CQuaternion quat = CQuaternion::AxisAngle(
        rightDir, CRelAngle::FromRadians(-gpTweakPlayerGun->x44_fixedVerticalAim));
    gunXf = quat.BuildTransform4f() * camXf.GetRotation();
    gunXf.SetTranslation(viewGunPos);
    break;
  }
  case kGH_Holstering: {
    float liftAngle =
        1.f - CMath::Limit(x49c_gunHolsterRemTime / gpTweakPlayerGun->x3c_gunHolsterTime, 1.f);
    if (x2f8_morphBallState == kMS_Morphing) {
      liftAngle = 1.f - CMath::Limit(x49c_gunHolsterRemTime / 0.1f, 1.f);
    }
    if (liftAngle > 0.01f) {
      CQuaternion quat = CQuaternion::AxisAngle(
          rightDir, CRelAngle::FromRadians(-liftAngle * gpTweakPlayerGun->x44_fixedVerticalAim));
      gunXf = quat.BuildTransform4f() * camXf.GetRotation();
      gunXf.SetTranslation(viewGunPos);
    }
    break;
  }
  default:
    break;
  }

  x490_gun->SetTransform(gunXf);
  CTransform4f out = gunXf;
  UpdateAimTargetPrediction(out, mgr);
  UpdateAssistedAiming(out, mgr);
}

const CTransform4f& CPlayer::GetFirstPersonCameraTransform(CStateManager& mgr) const {
  return mgr.GetCameraManager()->GetFirstPersonCamera()->GetGunFollowTransform();
}

void CPlayer::UpdateDebugCamera(CStateManager& mgr) {}

void CPlayer::UpdateArmAndGunTransforms(float dt, CStateManager& mgr) {
  CVector3f grappleOffset = CVector3f::Zero();
  CVector3f gunOffset;
  if (x2f8_morphBallState == kMS_Morphed) {
    gunOffset = CVector3f(0.f, 0.f, 0.6f);
  } else {
    gunOffset = gpTweakPlayerGun->x4c_gunPosition;
    grappleOffset = x490_gun->GetGrappleArm().IsArmMoving()
                        ? CVector3f::Zero()
                        : gpTweakPlayerGun->x64_grapplingArmPosition;
    gunOffset[kDZ] += GetEyeHeight();
    grappleOffset[kDZ] += GetEyeHeight();
  }

  UpdateGunTransform(gunOffset + x76c_cameraBob->GetGunBobTransformation().GetTranslation(), mgr);
  UpdateGrappleArmTransform(grappleOffset, mgr, dt);
}

void CPlayer::ForceGunOrientation(const CTransform4f& xf, CStateManager& mgr) {
  ResetGun(mgr);
  x530_gunDir = xf.GetColumn(kDY);
  x490_gun->SetTransform(xf);
  UpdateArmAndGunTransforms(0.01f, mgr);
}

void CPlayer::Update(float dt, CStateManager& mgr) {
  SetCoefficientOfRestitutionModifier(0.f);
  UpdateMorphBallTransition(dt, mgr);

  CPlayerState::EBeamId newBeam = mgr.GetPlayerState()->GetCurrentBeam();
  if (x7ec_beam != newBeam) {
    x7ec_beam = newBeam;
    CModelData modelData(CStaticRes(gpTweakPlayerRes->GetBallTransitionBeamResId(x7ec_beam),
                                    x7d0_animRes.GetScale()));
    CModelData* ptr;
    if (modelData.IsNull()) {
      ptr = nullptr;
    } else {
      ptr = rs_new CModelData(modelData);
    }
    x7f0_ballTransitionBeamModel = ptr;
  }

  if (!mgr.GetPlayerState()->IsAlive()) {
    if (!x9f4_deathTime) {
      CSfxManager::KillAll(CSfxManager::kSC_Game);
      CStreamAudioManager::StopAll();
      if (x2f8_morphBallState == kMS_Unmorphed) {
        DoSfxEffects(CSfxManager::SfxStart(SFXsam_death));
      }
    }

    float prevDeathTime = x9f4_deathTime;
    x9f4_deathTime += dt;
    if (x2f8_morphBallState != kMS_Unmorphed) {
      if (x9f4_deathTime >= 1.f && prevDeathTime < 1.f) {
        xa00_deathPowerBomb = x490_gun->DropPowerBomb(mgr);
      }
      if (x9f4_deathTime >= 4.f && prevDeathTime < 4.f) {
        DoSfxEffects(CSfxManager::SfxStart(SFXsam_death));
      }
    }
  }

  switch (x2f8_morphBallState) {
  case kMS_Unmorphed:
  case kMS_Morphing:
  case kMS_Unmorphing:
    CTransform4f gunXf = GetModelData()->GetScaledLocatorTransform(rstl::string_l(kGunLocator));
    x7f4_gunWorldXf = GetTransform() * gunXf;
    break;
  case kMS_Morphed:
    break;
  }

  if (x2f8_morphBallState == kMS_Unmorphed) {
    UpdateAimTargetTimer(dt);
    UpdateAimTarget(mgr);
    UpdateOrbitModeTimer(dt);
  }
  UpdateOrbitPreventionTimer(dt);
  if (x2f8_morphBallState == kMS_Morphed) {
    x768_morphball->Update(dt, mgr);
  } else {
    x768_morphball->StopSounds();
  }
  if (x2f8_morphBallState == kMS_Morphing || x2f8_morphBallState == kMS_Unmorphing) {
    x768_morphball->UpdateEffects(dt, mgr);
  }
  UpdateGunAlpha();
  UpdateDebugCamera(mgr);
  UpdateVisorTransition(dt, mgr);
  mgr.SetActorAreaId(*this, mgr.GetWorld()->GetCurrentAreaId());
  UpdatePlayerSounds(dt);
  if (x26c_attachedActor != kInvalidUniqueId) {
    x270_attachedActorTime += dt;
  }

  x740_staticTimer = rstl::max_val(0.f, x740_staticTimer - dt);
  if (x740_staticTimer > 0.f) {
    x74c_visorStaticAlpha = rstl::max_val(0.f, x74c_visorStaticAlpha - x744_staticOutSpeed * dt);
  } else {
    x74c_visorStaticAlpha = rstl::min_val(1.f, x74c_visorStaticAlpha + x748_staticInSpeed * dt);
  }

  x274_energyDrain.ProcessEnergyDrain(mgr, dt);
  x4a4_moveSpeedAvg.AddValue(x4f8_moveSpeed);

  mgr.PlayerState()->UpdateStaticInterference(mgr, dt);
  if (!ShouldSampleFailsafe(mgr)) {
    CPhysicsActor::Stop();
  }

  xa30_samusExhaustedVoiceTimer = IsEnergyLow(mgr) ? xa30_samusExhaustedVoiceTimer - dt : 4.f;

  if (!mgr.GetCameraManager()->IsInCinematicCamera() && xa30_samusExhaustedVoiceTimer <= 0.f) {
    StartSamusVoiceSfx(SFXsam_vox_exhausted, 127, 7);
    xa30_samusExhaustedVoiceTimer = 4.f;
  }
}

// TODO nonmatching
bool CPlayer::ShouldSampleFailsafe(CStateManager& mgr) const {
  const CCinematicCamera* cineCam =
      TCastToConstPtr< CCinematicCamera >(mgr.GetCameraManager()->GetCurrentCamera(mgr));
  if (!mgr.GetPlayerState()->IsAlive() ||
      (x2f4_cameraState == kCS_Spawned && cineCam && (cineCam->GetFlags() & 0x80) != 0)) {
    return false;
  }
  return true;
}

void CPlayer::UpdateVisorState(const CFinalInput& input, float dt, CStateManager& mgr) {
  x7a0_visorSteam.Update(dt);
  if (x7a0_visorSteam.AffectsThermal()) {
    mgr.SetThermalColdScale2(mgr.GetThermalColdScale2() + x7a0_visorSteam.GetAlpha());
  }

  CPlayerState* playerState = mgr.PlayerState();
  const CScriptGrapplePoint* grapplePoint =
      TCastToConstPtr< CScriptGrapplePoint >(mgr.GetObjectById(x310_orbitTargetId));
  if (GetOrbitState() != CPlayer::kOS_Grapple && !grapplePoint &&
      GetMorphballTransitionState() == kMS_Unmorphed && !playerState->GetIsVisorTransitioning() &&
      x3a8_scanState == kSS_NotScanning) {

    if (playerState->GetTransitioningVisor() == CPlayerState::kPV_Scan &&
        (ControlMapper::GetDigitalInput(ControlMapper::kC_FireOrBomb, input) ||
         ControlMapper::GetDigitalInput(ControlMapper::kC_MissileOrPowerBomb, input)) &&
        playerState->HasPowerUp(CPlayerState::kIT_CombatVisor)) {
      playerState->StartTransitionToVisor(CPlayerState::kPV_Combat);
      DrawGun(mgr);
    }

    for (int i = 0; i < ARRAY_SIZE(skVisorToItemMapping); ++i) {
      const TVisorToItemMapping& mapping = skVisorToItemMapping[i];

      if (playerState->HasPowerUp(mapping.first) &&
          ControlMapper::GetPressInput(mapping.second, input)) {
        x9c4_24_visorChangeRequested = true;

        CPlayerState::EPlayerVisor visor = static_cast< CPlayerState::EPlayerVisor >(i);
        if (playerState->GetTransitioningVisor() != visor) {
          playerState->StartTransitionToVisor(visor);
          if (visor == CPlayerState::kPV_Scan) {
            HolsterGun(mgr);
          } else {
            DrawGun(mgr);
          }
        }
      }
    }
  }
}

void CPlayer::UpdateVisorTransition(float dt, CStateManager& mgr) {
  CPlayerState* playerState = mgr.PlayerState();
  if (playerState->GetIsVisorTransitioning()) {
    playerState->UpdateVisorTransition(dt);
  }
}

void CPlayer::UpdateCrosshairsState(const CFinalInput& input) {
  x9c4_25_showCrosshairs = ControlMapper::GetDigitalInput(ControlMapper::kC_ShowCrosshairs, input);
}

void CPlayer::UpdatePlayerSounds(float dt) {
  if (x784_damageSfxTimer > 0.f) {
    x784_damageSfxTimer -= dt;
    if (x784_damageSfxTimer <= 0.f) {
      CSfxManager::SfxStop(x770_damageLoopSfx);
      x770_damageLoopSfx.Clear();
    }
  }
}

int CPlayer::SfxIdFromMaterial(const CMaterialList& mat, const ushort* idList, int tableLen,
                               ushort defId) {
  int id = defId;
  for (short i = 0; i < tableLen; ++i) {
    if (mat.HasMaterial(static_cast< EMaterialTypes >(i)) && idList[i] != 0xFFFF) {
      id = idList[i];
    }
  }
  // Odd issue with return value, should be ushort
  return ushort(id);
}

ushort CPlayer::GetMaterialSoundUnderPlayer(CStateManager& mgr, const ushort* table, int length,
                                            ushort defId) {
  int ret = defId;
  static const CVector3f skDown(0.f, 0.f, -1.f);
  static const CMaterialFilter matFilter = CMaterialFilter::MakeInclude(CMaterialList(kMT_Solid));

  TUniqueId collideId = kInvalidUniqueId;
  TEntityList nearList;
  CAABox aabb = GetBoundingBox();
  aabb.AccumulateBounds(GetTranslation() + skDown);
  mgr.BuildNearList(nearList, aabb, matFilter, nullptr);
  const CRayCastResult result =
      mgr.RayWorldIntersection(collideId, GetTranslation(), skDown, 1.5f, matFilter, nearList);
  if (result.IsValid()) {
    ret = SfxIdFromMaterial(result.GetMaterial(), table, length, defId);
  }
  return ret;
}

void CPlayer::UpdateFootstepSounds(const CFinalInput& input, CStateManager& mgr, float dt) {
  if (x2f8_morphBallState == kMS_Unmorphed && x258_movementState == NPlayer::kMS_OnGround &&
      !x3dc_inFreeLook && !x3dd_lookButtonHeld) {
    char sfxVol = 127;
    x78c_footstepSfxTimer += dt;
    float turn = TurnInput(input);
    const float forward = fabsf(ForwardInput(input, turn));
    turn = fabsf(turn);
    float sfxDelay = 0.f;
    if (forward > 0.05f || x304_orbitState != kOS_NoOrbit) {
      CVector3f velocity = GetVelocityWR();
      float mag = velocity.Magnitude();
      float vel = rstl::min_val(mag / GetActualFirstPersonMaxVelocity(dt), 1.f);
      if (vel > 0.05f) {
        sfxDelay = -0.475f * vel + 0.85f;
        if (x790_footstepSfxSel == kFS_None) {
          x790_footstepSfxSel = kFS_Left;
        }
      } else {
        x78c_footstepSfxTimer = 0.f;
        x790_footstepSfxSel = kFS_None;
      }

      sfxVol = CCast::ToInt8((vel * 38.f + 89.f) * 1.f);
    } else if (turn > 0.05f) {
      if (x790_footstepSfxSel == kFS_Left) {
        sfxDelay = -0.813f * turn + 1.f;
      } else {
        sfxDelay = -2.438f * turn + 3.f;
      }
      if (x790_footstepSfxSel == kFS_None) {
        x790_footstepSfxSel = kFS_Left;
        sfxDelay = x78c_footstepSfxTimer;
      }
      sfxVol = 96;
    } else {
      x78c_footstepSfxTimer = 0.f;
      x790_footstepSfxSel = kFS_None;
    }

    if (x790_footstepSfxSel != kFS_None && x78c_footstepSfxTimer > sfxDelay) {
      static const float earHeight = GetEyeHeight() - 0.1f;
      if (IsInFluid() && x828_distanceUnderWater > 0.f && x828_distanceUnderWater < earHeight) {
        if (x82c_inLava) {
          if (x790_footstepSfxSel == kFS_Left) {
            DoSfxEffects(CSfxManager::SfxStart(SFXlav_wlklava_00, sfxVol, 64, true));
          } else {
            DoSfxEffects(CSfxManager::SfxStart(SFXlav_wlklava_01, sfxVol, 64, true));
          }
        } else {
          if (x790_footstepSfxSel == kFS_Left) {
            DoSfxEffects(CSfxManager::SfxStart(SFXsam_wlkwater_00, sfxVol, 64, true));
          } else {
            DoSfxEffects(CSfxManager::SfxStart(SFXsam_wlkwater_01, sfxVol, 64, true));
          }
        }
      } else {
        ushort sfx;
        if (x790_footstepSfxSel == kFS_Left) {
          sfx = GetMaterialSoundUnderPlayer(mgr, skLeftStepSounds, ARRAY_SIZE(skLeftStepSounds),
                                            0xffff);
        } else {
          sfx = GetMaterialSoundUnderPlayer(mgr, skRightStepSounds, ARRAY_SIZE(skRightStepSounds),
                                            0xffff);
        }
        DoSfxEffects(CSfxManager::SfxStart(sfx, sfxVol, 64, true));
      }

      x78c_footstepSfxTimer = 0.f;
      if (x790_footstepSfxSel == kFS_Left) {
        x790_footstepSfxSel = kFS_Right;
      } else {
        x790_footstepSfxSel = kFS_Left;
      }
    }
  }
}

CPlayer::CVisorSteam::CVisorSteam(float targetAlpha, float alphaInDur, float alphaOutDur,
                                  CAssetId tex)
: x0_curTargetAlpha(targetAlpha)
, x4_curAlphaInDur(alphaInDur)
, x8_curAlphaOutDur(alphaOutDur)
, xc_tex(tex)
, x10_nextTargetAlpha(0.f)
, x14_nextAlphaInDur(0.f)
, x18_nextAlphaOutDur(0.f)
, x1c_txtr(kInvalidAssetId)
, x20_alpha(0.f)
, x24_delayTimer(0.f)
, x28_affectsThermal(false) {}

void CPlayer::CVisorSteam::Update(float dt) {
  if (x1c_txtr != kInvalidAssetId) {
    x0_curTargetAlpha = x10_nextTargetAlpha;
    x4_curAlphaInDur = x14_nextAlphaInDur;
    x8_curAlphaOutDur = x18_nextAlphaOutDur;
    xc_tex = x1c_txtr;
  } else {
    x0_curTargetAlpha = 0.f;
  }

  x1c_txtr = kInvalidAssetId;
  if (close_enough(x20_alpha, x0_curTargetAlpha) && close_enough(x20_alpha, 0.f)) {
    return;
  }

  if (x20_alpha > x0_curTargetAlpha) {
    if (x24_delayTimer <= 0.f) {
      x20_alpha -= dt / x8_curAlphaOutDur;
      if (x20_alpha < x0_curTargetAlpha) {
        x20_alpha = x0_curTargetAlpha;
      }
    } else {
      x24_delayTimer -= dt;
      if (x24_delayTimer < 0.f) {
        x24_delayTimer = 0.f;
      }
    }
    return;
  }

  if (!gpSimplePool->GetObj(SObjectTag('TXTR', xc_tex)).IsLoaded()) {
    return;
  }

  x20_alpha += dt / x4_curAlphaInDur;
  if (x20_alpha > x0_curTargetAlpha) {
    x20_alpha = x0_curTargetAlpha;
  }

  x24_delayTimer = 0.1f;
}

void CPlayer::CVisorSteam::SetSteam(float targetAlpha, float alphaInDur, float alphaOutDur,
                                    CAssetId txtr, bool affectsThermal) {
  if (x1c_txtr == kInvalidAssetId || targetAlpha > x10_nextTargetAlpha) {
    x10_nextTargetAlpha = targetAlpha;
    x14_nextAlphaInDur = alphaInDur;
    x18_nextAlphaOutDur = alphaOutDur;
    x1c_txtr = txtr;
  }
  x28_affectsThermal = affectsThermal;
}

void CPlayer::SetVisorSteam(float targetAlpha, float alphaInDur, float alphaOutDur, CAssetId txtr,
                            bool affectsThermal) {
  x7a0_visorSteam.SetSteam(targetAlpha, alphaInDur, alphaOutDur, txtr, affectsThermal);
}

const CScriptWater* CPlayer::GetVisorRunoffEffect(const CStateManager& mgr) const {
  const CScriptWater* water = nullptr;
  if (InFluidId() != kInvalidUniqueId) {
    water = TCastToConstPtr< CScriptWater >(mgr.GetObjectById(InFluidId()));
  }
  return water;
}

void CPlayer::SetMorphBallState(EPlayerMorphBallState state, CStateManager& mgr) {
  if (x2f8_morphBallState == kMS_Morphed && state != kMS_Morphed) {
    x9c5_26_ = IsInsideFluid();
  }

  x2f8_morphBallState = state;
  SetStandardCollider(state == kMS_Morphed);

  switch (state) {
  case kMS_Unmorphed:
    if (x9c5_26_ && mgr.GetCameraManager()->GetInsideFluid()) {
      if (const CScriptWater* water = GetVisorRunoffEffect(mgr)) {
        if (water->GetUnmorphVisorRunoffEffect()) {
          mgr.AddObject(rs_new CHUDBillboardEffect(
              rstl::optional_object< TToken< CGenDescription > >(
                  *water->GetUnmorphVisorRunoffEffect()),
              rstl::optional_object_null(), mgr.AllocateUniqueId(), true,
              rstl::string_l("WaterSheets"), CHUDBillboardEffect::GetNearClipDistance(mgr),
              CHUDBillboardEffect::GetScaleForPOV(mgr), CColor(1.f, 1.f, 1.f, 1.f),
              CVector3f(1.f, 1.f, 1.f), CVector3f(0.f, 0.f, 0.f)));
        }
        DoSfxEffects(CSfxManager::SfxStart(water->GetUnmorphVisorRunoffSfx()));
      }
    }
    break;
  case kMS_Morphed:
  case kMS_Morphing:
    x768_morphball->LoadMorphBallModel(mgr);
    break;
  case kMS_Unmorphing:
    break;
  }
}

void CPlayer::SetSpawnedMorphBallState(EPlayerMorphBallState state, CStateManager& mgr) {
  x2fc_spawnedMorphBallState = state;
  SetCameraState(kCS_Spawned, mgr);
  if (x2fc_spawnedMorphBallState != x2f8_morphBallState) {
    CPhysicsActor::Stop();
    BreakOrbit(kOB_Respawn, mgr);
    switch (x2fc_spawnedMorphBallState) {
    case kMS_Unmorphed: {
      CVector3f pos = CVector3f::Zero();
      if (CanLeaveMorphBallState(mgr, pos)) {
        SetTranslation(GetTranslation() + pos);
        LeaveMorphBallState(mgr);
        ForceGunOrientation(GetTransform(), mgr);
        DrawGun(mgr);
      }
      break;
    }
    case kMS_Morphed:
      EnterMorphBallState(mgr);
      ResetBallCamera(mgr);
      mgr.CameraManager()->ResetCameraHint(mgr);
      mgr.CameraManager()->BallCamera()->Reset(CreateTransformFromMovementDirection(), mgr);
      break;
    default:
      break;
    }
  }
}

void CPlayer::UpdateCinematicState(CStateManager& mgr) {
  if (mgr.GetCameraManager()->IsInCinematicCamera()) {
    if (x2f4_cameraState != kCS_Spawned) {
      x2fc_spawnedMorphBallState = x2f8_morphBallState;
      if (x2fc_spawnedMorphBallState == kMS_Unmorphing) {
        x2fc_spawnedMorphBallState = kMS_Unmorphed;
      }
      if (x2fc_spawnedMorphBallState == kMS_Morphing) {
        x2fc_spawnedMorphBallState = kMS_Morphed;
      }
      SetCameraState(kCS_Spawned, mgr);
    }
  } else {
    if (x2f4_cameraState == kCS_Spawned) {
      if (x2fc_spawnedMorphBallState == x2f8_morphBallState) {
        switch (x2fc_spawnedMorphBallState) {
        case kMS_Morphed:
          SetCameraState(kCS_Ball, mgr);
          break;
        case kMS_Unmorphed:
          SetCameraState(kCS_FirstPerson, mgr);
          if (mgr.GetPlayerState()->GetCurrentVisor() != CPlayerState::kPV_Scan) {
            ForceGunOrientation(GetTransform(), mgr);
            DrawGun(mgr);
          }
          break;
        default:
          break;
        }
      } else {
        CPhysicsActor::Stop();
        BreakOrbit(kOB_Respawn, mgr);
        switch (x2fc_spawnedMorphBallState) {
        case kMS_Unmorphed: {
          CVector3f vec = CVector3f::Zero();
          if (CanLeaveMorphBallState(mgr, vec)) {
            SetTranslation(GetTranslation() + vec);
            LeaveMorphBallState(mgr);
            SetCameraState(kCS_FirstPerson, mgr);
            ForceGunOrientation(GetTransform(), mgr);
            DrawGun(mgr);
          }
          break;
        }
        case kMS_Morphed:
          EnterMorphBallState(mgr);
          ResetBallCamera(mgr);
          mgr.CameraManager()->ResetCameraHint(mgr);
          mgr.CameraManager()->BallCamera()->Reset(CreateTransformFromMovementDirection(), mgr);
          break;
        default:
          break;
        }
      }
    }
  }
}

void CPlayer::UpdateCameraState(CStateManager& mgr) { UpdateCinematicState(mgr); }

void CPlayer::SetCameraState(EPlayerCameraState camState, CStateManager& mgr) {
  if (x2f4_cameraState == camState) {
    return;
  }

  x2f4_cameraState = camState;

  CCameraManager* camMgr = mgr.CameraManager();
  switch (camState) {
  case kCS_FirstPerson:
    camMgr->SetCurrentCameraId(camMgr->GetFirstPersonCamera()->GetUniqueId());
    x768_morphball->SetBallLightActive(mgr, false);
    break;
  case kCS_Ball:
    camMgr->SetCurrentCameraId(camMgr->GetBallCamera()->GetUniqueId());
    x768_morphball->SetBallLightActive(mgr, true);
    break;
  case kCS_Transitioning:
    camMgr->SetCurrentCameraId(camMgr->GetBallCamera()->GetUniqueId());
    x768_morphball->SetBallLightActive(mgr, true);
    break;
  case kCS_Two:
    break;
  case kCS_Spawned: {
    bool ballLight = false;
    if (const CCinematicCamera* cineCam =
            TCastToConstPtr< CCinematicCamera >(camMgr->GetCurrentCamera(mgr))) {
      ballLight = x2f8_morphBallState == kMS_Morphed && cineCam->GetFlags() & 0x40;
    }
    x768_morphball->SetBallLightActive(mgr, ballLight);
    break;
  }
  }
}

void CPlayer::UpdateFreeLookState(const CFinalInput& input, float dt, CStateManager& mgr) {
  if (x304_orbitState == kOS_ForcedOrbitObject || IsMorphBallTransitioning() ||
      x2f8_morphBallState != kMS_Unmorphed ||
      (x3b8_grappleState != kGS_None && x3b8_grappleState != kGS_Firing)) {
    x3dc_inFreeLook = false;
    x3dd_lookButtonHeld = false;
    x3de_lookAnalogHeld = false;
    x3e8_horizFreeLookAngleVel = 0.f;
    x3f0_vertFreeLookAngleVel = 0.f;
    x9c4_25_showCrosshairs = false;
    return;
  }

  if (gpTweakPlayer->mHoldButtonsForFreeLook) {
    if ((gpTweakPlayer->mTwoButtonsForFreeLook &&
         (ControlMapper::GetDigitalInput(ControlMapper::kC_LookHold1, input) &&
          ControlMapper::GetDigitalInput(ControlMapper::kC_LookHold2, input))) ||
        (!gpTweakPlayer->mTwoButtonsForFreeLook &&
         (ControlMapper::GetDigitalInput(ControlMapper::kC_LookHold1, input) ||
          ControlMapper::GetDigitalInput(ControlMapper::kC_LookHold2, input)))) {
      if (!x3dd_lookButtonHeld) {
        CVector3f lookDir =
            mgr.GetCameraManager()->GetFirstPersonCamera()->GetTransform().GetColumn(kDY);
        CVector3f lookDirFlat = lookDir;
        lookDirFlat.SetZ(0.f);
        x3e4_freeLookYawAngle = 0.f;
        if (lookDirFlat.CanBeNormalized()) {
          lookDirFlat.Normalize();
          x3ec_freeLookPitchAngle = acosf(CMath::Limit(CVector3f::Dot(lookDir, lookDirFlat), 1.f));
          if (lookDir.GetZ() < 0.f) {
            x3ec_freeLookPitchAngle = -x3ec_freeLookPitchAngle;
          }
        }
      }
      x3dc_inFreeLook = true;
      x3dd_lookButtonHeld = true;

      if (ControlMapper::GetAnalogInput(ControlMapper::kC_LookLeft, input) >= 0.1f ||
          ControlMapper::GetAnalogInput(ControlMapper::kC_LookRight, input) >= 0.1f ||
          ControlMapper::GetAnalogInput(ControlMapper::kC_LookDown, input) >= 0.1f ||
          ControlMapper::GetAnalogInput(ControlMapper::kC_LookUp, input) >= 0.1f) {
        x3de_lookAnalogHeld = true;
      } else {
        x3de_lookAnalogHeld = false;
      }
    } else {
      x3dc_inFreeLook = false;
      x3dd_lookButtonHeld = false;
      x3de_lookAnalogHeld = false;
      x3e8_horizFreeLookAngleVel = 0.f;
      x3f0_vertFreeLookAngleVel = 0.f;
    }
  } else {
    if (ControlMapper::GetAnalogInput(ControlMapper::kC_LookLeft, input) >= 0.1f ||
        ControlMapper::GetAnalogInput(ControlMapper::kC_LookRight, input) >= 0.1f ||
        ControlMapper::GetAnalogInput(ControlMapper::kC_LookDown, input) >= 0.1f ||
        ControlMapper::GetAnalogInput(ControlMapper::kC_LookUp, input) >= 0.1f) {
      x3de_lookAnalogHeld = true;
    } else {
      x3de_lookAnalogHeld = false;
    }
    x3dd_lookButtonHeld = false;
    if (fabsf(x3e4_freeLookYawAngle) < gpTweakPlayer->mFreeLookCenteredThresholdAngle &&
        fabsf(x3ec_freeLookPitchAngle) < gpTweakPlayer->mFreeLookCenteredThresholdAngle) {
      if (x3e0_curFreeLookCenteredTime > gpTweakPlayer->mFreeLookCenteredTime) {
        x3dc_inFreeLook = false;
        x3e8_horizFreeLookAngleVel = 0.f;
        x3f0_vertFreeLookAngleVel = 0.f;
      } else {
        x3e0_curFreeLookCenteredTime += dt;
      }
    } else {
      x3dc_inFreeLook = true;
      x3e0_curFreeLookCenteredTime = 0.f;
    }
  }

  UpdateCrosshairsState(input);
}

void CPlayer::UpdateCameraTimers(float dt, const CFinalInput& input) {
  if (x3dc_inFreeLook || x3dd_lookButtonHeld) {
    x294_jumpCameraTimer = 0.f;
    x29c_fallCameraTimer = 0.f;
    return;
  }

  if (gpTweakPlayer->mFiringCancelsCameraPitch) {
    if (ControlMapper::GetDigitalInput(ControlMapper::kC_FireOrBomb, input) ||
        ControlMapper::GetDigitalInput(ControlMapper::kC_MissileOrPowerBomb, input)) {
      if (x288_startingJumpTimeout > 0.f) {
        x2a4_cancelCameraPitch = true;
        return;
      }
    }
  }

  if (ControlMapper::GetPressInput(ControlMapper::kC_JumpOrBoost, input)) {
    ++x298_jumpPresses;
  }

  if (ControlMapper::GetDigitalInput(ControlMapper::kC_JumpOrBoost, input) &&
      x294_jumpCameraTimer > 0.f && !x2a4_cancelCameraPitch && x298_jumpPresses <= 2) {
    x294_jumpCameraTimer += dt;
  }

  if (x29c_fallCameraTimer > 0.f && !x2a4_cancelCameraPitch) {
    x29c_fallCameraTimer += dt;
  }
}

void CPlayer::AcceptScriptMsg(EScriptObjectMessage msg, TUniqueId sender, CStateManager& mgr) {
  switch (msg) {
  case kSM_OnFloor:
    if (x258_movementState != NPlayer::kMS_OnGround && x2f8_morphBallState != kMS_Morphed &&
        x300_fallingTime > 0.3f) {
      if (x258_movementState != NPlayer::kMS_Falling) {
        float hardThres = CMath::FastSqrtF(-gpTweakPlayer->mNormalGravAccel * 2.f * 30.f);
        uchar landVol =
            CCast::ToUint8(CMath::Clamp(95.f, -x794_lastVelocity.GetZ() * 1.6f + 95.f, 127.f));
        ushort landSfx;
        if (-x794_lastVelocity.GetZ() < hardThres) {
          landSfx = GetMaterialSoundUnderPlayer(mgr, skPlayerLandSfxSoft,
                                                ARRAY_SIZE(skPlayerLandSfxSoft), 0xffff);
        } else {
          landSfx = GetMaterialSoundUnderPlayer(mgr, skPlayerLandSfxHard,
                                                ARRAY_SIZE(skPlayerLandSfxHard), 0xffff);
          StartSamusVoiceSfx(SFXsam_voxland_02, 127, 5);
          x55c_damageAmt = 0.f;
          x560_prevDamageAmt = 10.f;
          x564_damageLocation = GetTranslation();
          x558_wasDamaged = true;
          mgr.CameraManager()->AddCameraShaker(CCameraShakeData::HardBothAxesShake(0.3f, 1.25f),
                                               false);
          StartLandingControlFreeze();
        }
        DoSfxEffects(CSfxManager::SfxStart(landSfx, landVol, 64, true));

        float rumbleMag = -x794_lastVelocity.GetZ() * (1.f / 110.f);
        if (rumbleMag > 0.f) {
          mgr.GetRumbleManager()->Rumble(mgr, kRFX_PlayerLand, CMath::Limit(rumbleMag, 0.8f),
                                         kRP_One);
        }

        x2a0_ = 0.f;
      }
    } else if (x258_movementState != NPlayer::kMS_OnGround && x2f8_morphBallState == kMS_Morphed) {
      if (GetVelocityWR().GetZ() < -40.f && !x768_morphball->GetIsInHalfPipeMode() &&
          x258_movementState == NPlayer::kMS_ApplyJump && x300_fallingTime > 0.75f) {
        SetCoefficientOfRestitutionModifier(0.2f);
      }
      x768_morphball->Land();
      if (GetVelocityWR().GetZ() < -5.f) {
        mgr.GetRumbleManager()->Rumble(
            mgr, kRFX_PlayerLand,
            CMath::Limit(-GetVelocityWR().GetZ() * (1.f / 110.f) * 0.5f, 0.8f), kRP_One);
        x2a0_ = 0.f;
      }
      if (GetVelocityWR().GetZ() < -30.f) {
        mgr.GetRumbleManager()->Rumble(mgr, kRFX_PlayerLand,
                                       CMath::Limit(-GetVelocityWR().GetZ() * (1.f / 110.f), 0.8f),
                                       kRP_One);
        x2a0_ = 0.f;
      }
    }
    x300_fallingTime = 0.f;
    SetMoveState(NPlayer::kMS_OnGround, mgr);
    break;
  case kSM_Falling:
    if (x2f8_morphBallState == kMS_Morphed) {
      if (x768_morphball->GetSpiderBallState() == CMorphBall::kSBS_Active) {
        break;
      }
    }
    if (x2f8_morphBallState != kMS_Morphed) {
      SetMoveState(NPlayer::kMS_Falling, mgr);
    } else if (x258_movementState == NPlayer::kMS_OnGround) {
      SetMoveState(NPlayer::kMS_FallingMorphed, mgr);
    }
    break;
  case kSM_LandOnNotFloor:
    if (x2f8_morphBallState == kMS_Morphed &&
        x768_morphball->GetSpiderBallState() == CMorphBall::kSBS_Active &&
        x258_movementState != NPlayer::kMS_ApplyJump) {
      SetMoveState(NPlayer::kMS_ApplyJump, mgr);
    }
    break;
  case kSM_OnIceSurface:
    x2ac_surfaceRestraint = kSR_Ice;
    break;
  case kSM_OnMudSlowSurface:
    x2ac_surfaceRestraint = kSR_Organic;
    break;
  case kSM_OnNormalSurface:
    x2ac_surfaceRestraint = kSR_Normal;
    break;
  case kSM_InSnakeWeed:
    x2ac_surfaceRestraint = kSR_Shrubbery;
    break;
  case kSM_AddSplashInhabitant: {
    SetInFluid(true, sender);
    UpdateSubmerged(mgr);
    float length = 0.5f * GetEyeHeight();
    CMaterialFilter filter = CMaterialFilter::MakeInclude(CMaterialList(kMT_Solid));
    CVector3f dir(0.f, 0.f, -1.f);
    const CRayCastResult result = mgr.RayStaticIntersection(GetTranslation(), dir, length, filter);
    if (result.IsInvalid()) {
      SetVelocityWR(GetVelocityWR() * 0.095f);
      SetConstantForceWR(GetConstantForceWR() * 0.095f);
    }
    break;
  }
  case kSM_UpdateSplashInhabitant:
    UpdateSubmerged(mgr);
    if (CheckSubmerged() && !mgr.GetPlayerState()->HasPowerUp(CPlayerState::kIT_GravitySuit)) {
      if (const CScriptWater* water =
              TCastToConstPtr< CScriptWater >(mgr.ObjectById(InFluidId()))) {
        switch (water->GetFluidPlane().GetFluidType()) {
        case CFluidPlane::kFT_NormalWater:
          x2b0_outOfWaterTicks = 0;
          break;
        case CFluidPlane::kFT_Lava:
        case CFluidPlane::kFT_ThickLava:
          x2ac_surfaceRestraint = kSR_Lava;
          break;
        case CFluidPlane::kFT_PoisonWater:
          x2b0_outOfWaterTicks = 0;
          break;
        case CFluidPlane::kFT_PhazonFluid:
          x2ac_surfaceRestraint = kSR_Phazon;
          break;
        default:
          break;
        }
      }
    }
    x9c5_25_splashUpdated = true;
    break;
  case kSM_RemoveSplashInhabitant:
    SetInFluid(false, kInvalidUniqueId);
    UpdateSubmerged(mgr);
    break;
  case kSM_ProjectileCollide:
    x378_orbitPreventionTimer = gpTweakPlayer->mOrbitPreventionTime;
    BreakOrbit(kOB_ProjectileCollide, mgr);
    break;
  case kSM_AddPlatformRider:
    x82e_ridingPlatform = sender;
    break;
  case kSM_Damage:
    if (const CEnergyProjectile* energ =
            TCastToConstPtr< CEnergyProjectile >(mgr.ObjectById(sender))) {
      if ((energ->GetAttribField() & CWeapon::kPA_StaticInterference) ==
          CWeapon::kPA_StaticInterference) {
        mgr.PlayerState()->StaticInterference().AddSource(GetUniqueId(), 0.3f,
                                                          energ->GetInterferenceDuration());
      }
    }
    break;
  case kSM_Deleted:
    mgr.PlayerState()->ResetVisor();
    x730_transitionModels.clear();
    break;
  default:
    break;
  }

  x490_gun->AcceptScriptMsg(msg, sender, mgr);
  x768_morphball->AcceptScriptMsg(msg, sender, mgr);
  CActor::AcceptScriptMsg(msg, sender, mgr);
}

void CPlayer::PreThink(float dt, CStateManager& mgr) {
  x558_wasDamaged = false;
  x55c_damageAmt = 0.f;
  x560_prevDamageAmt = 0.f;
  x564_damageLocation = CVector3f::Zero();
  xa04_preThinkDt = dt;
}

void CPlayer::AdjustEyeOffset(CStateManager& mgr) {
  const CScriptWater* water = TCastToConstPtr< CScriptWater >(mgr.GetObjectById(InFluidId()));
  if (!water) {
    return;
  }

  CVector3f bounds = water->GetTriggerBoundsWR().GetMaxPoint();
  CVector3f eyePos = GetEyePosition();
  eyePos[kDZ] -= GetEyeOffset();
  float waterToDeltaDelta = eyePos.GetZ() - bounds.GetZ();

  if (eyePos.GetZ() >= bounds.GetZ() && waterToDeltaDelta <= 0.25f) {
    SetEyeOffset(GetEyeOffset() + bounds.GetZ() + 0.25f - eyePos.GetZ());
  } else if (eyePos.GetZ() < bounds.GetZ() && waterToDeltaDelta >= -0.2f) {
    SetEyeOffset(GetEyeOffset() + bounds.GetZ() - 0.2f - eyePos.GetZ());
  }
}

void CPlayer::Think(float dt, CStateManager& mgr) {
  UpdateStepUpSmoothing(dt);
  AdjustEyeOffset(mgr);
  UpdateEnvironmentDamageCameraShake(dt, mgr);
  UpdatePhazonDamage(dt, mgr);
  UpdateFreeLook(dt);
  UpdatePlayerHints(mgr);

  if (x2b0_outOfWaterTicks < 2) {
    x2b0_outOfWaterTicks += 1;
  }

  x9c5_24_ = x9c4_31_inWaterMovement;
  x9c4_31_inWaterMovement = x9c5_25_splashUpdated;
  x9c5_25_splashUpdated = false;
  UpdateBombJumpStuff();

  if (0.f < x288_startingJumpTimeout) {
    x288_startingJumpTimeout -= dt;
    if (0.f >= x288_startingJumpTimeout) {
      SetMoveState(NPlayer::kMS_ApplyJump, mgr);
    }
  }

  if (x2a0_ > 0.f) {
    x2a0_ += dt;
  }
  if (x774_samusVoiceTimeout > 0.f) {
    x774_samusVoiceTimeout -= dt;
  }
  if (0.f < x28c_sjTimer) {
    x28c_sjTimer -= dt;
  }

  x300_fallingTime += dt;
  if (x258_movementState == NPlayer::kMS_FallingMorphed && x300_fallingTime > 0.4f) {
    SetMoveState(NPlayer::kMS_ApplyJump, mgr);
  }

  if (x570_immuneTimer > 0.f) {
    x570_immuneTimer -= dt;
  }

  Update(dt, mgr);
  UpdateTransitionFilter(dt, mgr);
  CalculatePlayerMovementDirection(dt);
  UpdatePlayerControlDirection(dt, mgr);

  if (gUseSurfaceHack) {
    if (gSR_Hack == kSR_Water) {
      x2b0_outOfWaterTicks = 0;
    } else {
      x2ac_surfaceRestraint = gSR_Hack;
    }
  }

  if (x2f8_morphBallState == kMS_Unmorphed && x9c5_27_camSubmerged &&
      mgr.GetCameraManager()->GetFluidCounter() == 0) {
    if (const CScriptWater* water = GetVisorRunoffEffect(mgr)) {
      if (water->GetVisorRunoffEffect()) {
        mgr.AddObject(rs_new CHUDBillboardEffect(
            rstl::optional_object< TToken< CGenDescription > >(*water->GetVisorRunoffEffect()),
            rstl::optional_object_null(), mgr.AllocateUniqueId(), true,
            rstl::string_l("WaterSheets"), CHUDBillboardEffect::GetNearClipDistance(mgr),
            CHUDBillboardEffect::GetScaleForPOV(mgr), CColor(1.f, 1.f, 1.f, 1.f),
            CVector3f(1.f, 1.f, 1.f), CVector3f(0.f, 0.f, 0.f)));
      }
      DoSfxEffects(CSfxManager::SfxStart(water->GetVisorRunoffSfx()));
    }
  }
  x9c5_27_camSubmerged = mgr.GetCameraManager()->GetFluidCounter() != 0;

  if (x2f8_morphBallState != kMS_Morphed) {
    if (fabsf(GetTransform().GetColumn(kDX).GetZ()) > FLT_EPSILON ||
        fabsf(GetTransform().GetColumn(kDY).GetZ()) > FLT_EPSILON) {
      CVector3f backupTranslation = GetTranslation();
      CVector3f lookDirFlat = GetTransform().GetColumn(kDY);
      lookDirFlat.SetZ(0.f);
      if (lookDirFlat.CanBeNormalized()) {
        SetTransform(CTransform4f::LookAt(CUnitVector3f(CVector3f::Zero()),
                                          CUnitVector3f(lookDirFlat.AsNormalized())));
      } else {
        SetTransform(CTransform4f::Identity());
      }
      SetTranslation(backupTranslation);
    }
  }

  x794_lastVelocity = GetVelocityWR();
}

void CPlayer::SetFrozenState(CStateManager& stateMgr, CAssetId steamTxtr, ushort sfx,
                             CAssetId iceTxtr) {
  if (!stateMgr.GetCameraManager()->IsInCinematicCamera() && !GetFrozenState()) {
    bool showMsg;
    CSystemState& systemState = gpGameState->SystemState();
    if (x2f8_morphBallState == kMS_Unmorphed) {
      showMsg = systemState.AreFreezeInstructionsStillEnabledFirstPerson();
    } else {
      showMsg = systemState.AreFreezeInstructionsStillEnabledMorphBall();
    }

    if (showMsg) {
      int msgIdx = x2f8_morphBallState != kMS_Morphed ? 19 : 20;
      CSamusHud::DisplayHudMemo(rstl::wstring_l(gpStringTable->GetString(msgIdx)),
                                CHUDMemoParms(5.f, true, false, false));
    }

    x750_frozenTimeout = x758_frozenTimeoutBias + gpTweakPlayer->mFrozenTimeout;
    x754_iceBreakJumps = -x75c_additionalIceBreakJumps;

    CPhysicsActor::Stop();
    ClearForcesAndTorques();
    if (x3b8_grappleState != kGS_None) {
      BreakGrapple(kOB_Freeze, stateMgr);
    } else {
      BreakOrbit(kOB_Freeze, stateMgr);
    }

    AddMaterial(kMT_Immovable, stateMgr);
    IsMorphBallTransitioning();
    xa08_steamTextureId = steamTxtr;
    xa0c_iceTextureId = iceTxtr;
    DoSfxEffects(CSfxManager::SfxStart(sfx));
    EndLandingControlFreeze();
  }
}

bool CPlayer::GetFrozenState() const { return x750_frozenTimeout > 0.f; }

void CPlayer::BreakFrozenState(CStateManager& stateMgr) {
  if (!GetFrozenState()) {
    return;
  }

  x750_frozenTimeout = 0.f;
  x754_iceBreakJumps = 0;
  CPhysicsActor::Stop();
  ClearForcesAndTorques();
  RemoveMaterial(kMT_Immovable, stateMgr);
  if (!stateMgr.GetCameraManager()->IsInCinematicCamera() && xa0c_iceTextureId != kInvalidAssetId) {
    stateMgr.AddObject(rs_new CHUDBillboardEffect(
        rstl::optional_object< TToken< CGenDescription > >(
            gpSimplePool->GetObj(SObjectTag('PART', xa0c_iceTextureId))),
        rstl::optional_object_null(), stateMgr.AllocateUniqueId(), true,
        rstl::string_l("FrostExplosion"), CHUDBillboardEffect::GetNearClipDistance(stateMgr),
        CHUDBillboardEffect::GetScaleForPOV(stateMgr), CColor(1.f, 1.f, 1.f, 1.f),
        CVector3f(1.f, 1.f, 1.f), CVector3f(0.f, 0.f, 0.f)));
    DoSfxEffects(CSfxManager::SfxStart(SFXcrk_break_final));
  }

  x768_morphball->ResetMorphBallIceBreak();
  SetVisorSteam(0.f, 6.f / 14.f, 1.f / 14.f, xa08_steamTextureId, false);
}

void CPlayer::UpdateFrozenState(const CFinalInput& input, CStateManager& mgr) {
  x750_frozenTimeout -= input.Time();
  if (x750_frozenTimeout > 0.f) {
    SetVisorSteam(0.7f, 6.f / 14.f, 1.f / 14.f, xa08_steamTextureId, false);
  } else {
    BreakFrozenState(mgr);
    return;
  }
  if (x258_movementState == NPlayer::kMS_OnGround ||
      x258_movementState == NPlayer::kMS_FallingMorphed) {
    Stop();
    ClearForcesAndTorques();
  }
  x7a0_visorSteam.Update(input.Time());

  switch (x2f8_morphBallState) {
  case kMS_Morphed:
    x490_gun->ProcessInput(input, mgr);
    break;
  case kMS_Unmorphed:
  case kMS_Morphing:
  case kMS_Unmorphing:
    if (ControlMapper::GetPressInput(ControlMapper::kC_JumpOrBoost, input)) {
      if (x754_iceBreakJumps != 0) {
        /* Subsequent Breaks */
        DoSfxEffects(CSfxManager::SfxStart(SFXcrk_break_subsequent));
      } else {
        /* Initial Break */
        DoSfxEffects(CSfxManager::SfxStart(SFXcrk_break_initial));
      }
      int max = gpTweakPlayer->mIceBreakJumpCount;
      if (++x754_iceBreakJumps > max) {
        gpGameState->SystemState().IncNumFreezeInstructionsPrintedFirstPerson();
        CSamusHud::ClearHudMemo();
        BreakFrozenState(mgr);
      }
    }
    break;
  }
}

void CPlayer::EndLandingControlFreeze() {
  x760_controlsFrozen = false;
  x764_controlsFrozenTimeout = 0.f;
}

void CPlayer::UpdateControlLostState(float dt, CStateManager& mgr) {
  x764_controlsFrozenTimeout -= dt;
  if (x764_controlsFrozenTimeout <= 0.f) {
    EndLandingControlFreeze();
  } else {
    const CFinalInput dummy;
    if (x2f8_morphBallState == kMS_Morphed) {
      x768_morphball->ComputeBallMovement(dummy, mgr, dt);
      x768_morphball->UpdateBallDynamics(mgr, dt);
    } else {
      ComputeMovement(dummy, mgr, dt);
    }
  }
}

void CPlayer::StartLandingControlFreeze() {
  x760_controlsFrozen = true;
  x764_controlsFrozenTimeout = 0.75f;
}
