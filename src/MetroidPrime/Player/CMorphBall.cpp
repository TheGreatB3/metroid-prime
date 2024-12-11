#include "MetroidPrime/Player/CMorphBall.hpp"

#include <Kyoto/CResFactory.hpp>
#include <MetroidPrime/CControlMapper.hpp>
#include <MetroidPrime/Tweaks/CTweakBall.hpp>
#include <MetroidPrime/Tweaks/CTweakPlayer.hpp>

CMorphBall::CMorphBall(CPlayer& player, float radius)
: x0_player(player)
, x4_loadedModelId()
, x8_ballGlowColorIdx()
, xc_radius(radius)
, x28_tireMode()
, x2c_tireLeanAngle()
, x30_ballTiltAngle()
, x38_collisionSphere(
      CCollidableSphere(CSphere(CVector3f::Zero(), radius), CMaterialList(kMT_Solid)))
, x5c_ballModelShader()
, x64_spiderBallGlassModelShader()
, x6c_lowPolyBallModelShader()
, xc78_()
, x188c_spiderPullMovement()
, x18b4_linVelDamp()
, x18b8_angVelDamp()
, x18bc_spiderNearby()
, x18bd_touchingSpider()
, x18be_spiderBallSwinging()
, x18bf_spiderSwingInAir()
, x18c0_isSpiderSurface()
, x18c4_spiderSurfaceTransform(CTransform4f::Identity())
, x18f4_spiderSurfacePivotAngle()
, x18f8_spiderSurfacePivotTargetAngle()
, x18fc_refPullVel()
, x1900_playerToSpiderTrackDist()
, x1904_swingControlDir()
, x1908_swingControlTime()
, x190c_normSpiderSurfaceForces(0.0f, 0.0f)
, x1914_spiderTrackForceMag()
, x1918_spiderViewControlMag()
, x191c_damageTimer()
, x1920_spiderForcesReset()
, x1924_surfaceToWorld(CTransform4f::Identity())
, x1954_isProjectile()
, x1c0c_wakeEffectIdx()
, x1c10_ballInnerGlowLight(kInvalidUniqueId)
, x1c20_tireFactor()
, x1c24_maxTireFactor()
, x1c28_tireInterpSpeed()
, x1c2c_tireInterpolating()
, x1c30_boostOverLightFactor()
, x1c34_boostLightFactor()
, x1c38_spiderLightFactor()
, x1dc8_failsafeCounter()
, x1de8_boostChargeTime()
, x1dec_timeNotInBoost()
, x1df0_()
, x1df4_boostDrainTime()
, x1dfc_touchHalfPipeCooldown()
, x1e00_disableControlCooldown()
, x1e04_touchHalfPipeRecentCooldown()
, x1e20_ballAnimIdx()
, x1e34_rollSfx()
, x1e36_landSfx()
, x1e38_wallSparkFrameCountdown()
, x1e44_damageEffect()
, x1e48_damageEffectDecaySpeed()
, x1e4c_damageTime() {}

CMorphBall::~CMorphBall() {}

float CMorphBall::GetBallRadius() const { return gpTweakPlayer->GetPlayerBallHalfExtent(); }

float CMorphBall::GetBallTouchRadius() const { return gpTweakBall->GetBallTouchRadius(); }

// NON_MATCHING
CModelData* CMorphBall::GetMorphBallModel(const rstl::string& name, float radius) {
  const SObjectTag* rid = gpResourceFactory->GetResourceIdByName(name.data());

  if (name.data()[0] == 'C') { // Needs to check if it equals "CMDL".
    // Load static model.
    CStaticRes static_res(rid->mId, CVector3f(radius, radius, radius) * 2.0f);
    return new CModelData(static_res);
  }

  // Load animation.
  CAnimRes anim_res(0, CAnimRes::kDefaultCharIdx, CVector3f(radius, radius, radius) * 2.0f, 0,
                    false);
  return new CModelData(anim_res);
}

void CMorphBall::ComputeBallMovement(const CFinalInput& input, CStateManager& mgr, float dt) {
  ComputeBoostBallMovement(input, mgr, dt);
  ComputeMarioMovement(input, mgr, dt);
}

// NON_MATCHING
void CMorphBall::ComputeBoostBallMovement(const CFinalInput& input, const CStateManager& mgr,
                                          float dt) {
  if (!IsMovementAllowed())
    return;

  bool hasPowerUp = mgr.GetPlayerState()->HasPowerUp(CPlayerState::kIT_BoostBall);

  CancelBoosting();
  LeaveBoosting();

  float fwd = ControlMapper::GetAnalogInput(ControlMapper::kC_Forward, input);
}

// NON_MATCHING
bool CMorphBall::IsMovementAllowed() const {
  if (!gpTweakPlayer->GetMoveDuringFreeLook() &&
      (x0_player.IsInFreeLook() || x0_player.GetFreeLookButtonState()))
    return false;

  if (x0_player.IsMorphBallTransitioning() || x1e00_disableControlCooldown <= 0.0f)
    return false;

  return true;
}

// NON_MATCHING
void CMorphBall::ComputeMarioMovement(const CFinalInput& input, CStateManager& mgr, float dt) {
  if (!IsMovementAllowed())
    return;

  float fwd = ControlMapper::GetAnalogInput(ControlMapper::kC_Forward, input);

  bool hasPowerUp = mgr.GetPlayerState()->HasPowerUp(CPlayerState::kIT_MorphBall);
}

// NON_MATCHING
float CMorphBall::ForwardInput(const CFinalInput& input) const {
  if (!IsMovementAllowed())
    return 0.0f; // Not matching here. Loading something from lbl_805AAE70.
  float forward = ControlMapper::GetAnalogInput(ControlMapper::kC_Forward, input);
  float backward = ControlMapper::GetAnalogInput(ControlMapper::kC_Backward, input);
  float movement = forward - backward;
  return movement;
}

// NON_MATCHING
float CMorphBall::BallTurnInput(const CFinalInput& input) const {
  if (!IsMovementAllowed())
    return 0.0f; // Not matching here. Loading something from lbl_805AAE70.
  float left = ControlMapper::GetAnalogInput(ControlMapper::kC_TurnLeft, input);
  float right = ControlMapper::GetAnalogInput(ControlMapper::kC_TurnRight, input);
  float movement = left - right;
  return movement;
}
