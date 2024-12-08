#include "MetroidPrime/Player/CMorphBall.hpp"

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

void CMorphBall::ComputeBallMovement(const CFinalInput& input, CStateManager& mgr, float dt) {
  ComputeBoostBallMovement(input, mgr, dt);
  ComputeMarioMovement(input, mgr, dt);
}
