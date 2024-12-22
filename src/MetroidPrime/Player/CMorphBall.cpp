#include "MetroidPrime/Player/CMorphBall.hpp"

#include <Kyoto/Audio/CSfxManager.hpp>
#include <Kyoto/CResFactory.hpp>
#include <Kyoto/Particles/CElementGen.hpp>
#include <Kyoto/Particles/CParticleSwoosh.hpp>
#include <MetroidPrime/CActorLights.hpp>
#include <MetroidPrime/CAnimData.hpp>
#include <MetroidPrime/CControlMapper.hpp>
#include <MetroidPrime/CGameLight.hpp>
#include <MetroidPrime/CRainSplashGenerator.hpp>
#include <MetroidPrime/CWorldShadow.hpp>
#include <MetroidPrime/Tweaks/CTweakBall.hpp>
#include <MetroidPrime/Tweaks/CTweakPlayer.hpp>

// NON_MATCHING
CMorphBall::CMorphBall(CPlayer& player, float radius)
: x0_player(player)
, x4_loadedModelId(-1)
, x8_ballGlowColorIdx()
, xc_radius(radius)
, x10_boostControlForce(CVector3f::Zero())
, x1c_controlForce(CVector3f::Zero())
, x28_tireMode()
, x2c_tireLeanAngle()
, x30_ballTiltAngle()
, x38_collisionSphere(
      CCollidableSphere(CSphere(CVector3f::Zero(), radius), CMaterialList(kMT_Solid)))
, x5c_ballModelShader()
, x64_spiderBallGlassModelShader()
, x6c_lowPolyBallModelShader()
, xc78_()
, x187c_spiderBallState(kSBS_Inactive)
, x1880_playerToSpiderNormal(CVector3f::Zero())
, x188c_spiderPullMovement(1.0)
, x1890_spiderTrackPoint(CVector3f::Zero())
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
, x1c14_worldShadow(new CWorldShadow(16, 16, false))
, x1c18_actorLights(new CActorLights(8, CVector3f::Zero(), 4, 4))
, x1c1c_rainSplashGen(new CRainSplashGenerator(x58_ballModel->GetScale(), 40, 2, 0.15, 0.5))
, x1c20_tireFactor(0.0f)
, x1c24_maxTireFactor(0.5f)
, x1c28_tireInterpSpeed(1.0f)
, x1c2c_tireInterpolating(false)
, x1c30_boostOverLightFactor()
, x1c34_boostLightFactor()
, x1c38_spiderLightFactor()
, x1dc8_failsafeCounter()
, x1dcc_(CVector3f::Zero())
, x1dd8_(CVector3f::Zero())
, x1de8_boostChargeTime()
, x1dec_timeNotInBoost()
, x1df0_()
, x1df4_boostDrainTime()
, x1dfc_touchHalfPipeCooldown()
, x1e00_disableControlCooldown()
, x1e04_touchHalfPipeRecentCooldown()
, x1e08_prevHalfPipeNormal(CVector3f::Zero())
, x1e14_halfPipeNormal(CVector3f::Zero())
, x1e20_ballAnimIdx()
, x1e34_rollSfx()
, x1e36_landSfx()
, x1e38_wallSparkFrameCountdown()
, x1e44_damageEffect()
, x1e48_damageEffectDecaySpeed()
, x1e4c_damageTime() {
  x19d4_spiderBallMagnetEffectGen->SetParticleEmission(false);
  x19d4_spiderBallMagnetEffectGen->Update(1.0f / 60.0f);

  kSpiderBallCollisionRadius = GetBallRadius() + 0.2f;

  for (int i = 0; i < 32; i++) {
    CToken token(x19a0_spiderElectric);
    rstl::single_ptr< CParticleSwoosh > swoosh(new CParticleSwoosh(token));
    x19e4_spiderElectricGens.push_back(
        rstl::pair< rstl::single_ptr< CParticleSwoosh >, bool >(swoosh, false));
  }

  // LoadAnimationTokens();
  // InitializeWakeEffects();
}

CMorphBall::~CMorphBall() {}

void CMorphBall::Update(float dt, CStateManager& mgr) {
  if (GetSpiderBallState() == kSBS_Active) {
    CVector3f position = GetBallToWorld().GetTranslation();
    CreateSpiderBallParticles(position, x1890_spiderTrackPoint);
  }

  if (x0_player.GetDeathTime() <= 0.0f) {
    UpdateEffects(dt, mgr);
  }

  if (x1e44_damageEffect > 0.0f) {
    x1e44_damageEffect -= x1e48_damageEffectDecaySpeed * dt;

    if (x1e44_damageEffect <= 0.0f) {
      x1e44_damageEffect = 0.0f;
      x1e48_damageEffectDecaySpeed = 0.0f;
      x1e4c_damageTime = 0.0f;
    } else {
      x1e4c_damageTime += dt;
    }
  }

  if (!x58_ballModel.null()) {
    x58_ballModel->AdvanceAnimation(dt, mgr, kInvalidAreaId, true);
  }

  if (x1c2c_tireInterpolating) {
    x1c20_tireFactor += x1c28_tireInterpSpeed * dt;
    if (x1c20_tireFactor < 0.0f) {
      x1c2c_tireInterpolating = false;
      x1c20_tireFactor = 0.0f;
    } else {
      if (x1c20_tireFactor > x1c24_maxTireFactor) {
        x1c2c_tireInterpolating = false;
        x1c20_tireFactor = x1c24_maxTireFactor;
      }
    }
  }

  if (!x1c1c_rainSplashGen.null()) {
    x1c1c_rainSplashGen->Update(dt, mgr);
  }

  UpdateMorphBallSound(dt);
}

CTransform4f CMorphBall::GetBallToWorld() const {
  return CTransform4f(
      CTransform4f::Translate(x0_player.GetTranslation() + CVector3f(0.0f, 0.0f, GetBallRadius())) *
      x0_player.GetTransform().GetRotation());
}

float CMorphBall::GetBallRadius() const { return gpTweakPlayer->GetPlayerBallHalfExtent(); }

void CMorphBall::SetAsProjectile() { x1954_isProjectile = true; }

// NON_MATCHING
bool CMorphBall::IsInFrustum(const CFrustumPlanes& frustum) const {
  if (x58_ballModel->IsNull())
    return false;

  rstl::optional_object< CAABox > bounds = x19b8_slowBlueTailSwooshGen->GetBounds();

  if (x58_ballModel->IsInFrustum(GetBallToWorld(), frustum))
    return true;

  if (x19b8_slowBlueTailSwooshGen->GetModulationColor().GetAlpha() != 0.0f && bounds.valid() &&
      frustum.BoxFrustumPlanesCheck(bounds.data()) != 0)
    return true;

  return false;
}

float CMorphBall::GetBallTouchRadius() const { return gpTweakBall->GetBallTouchRadius(); }

// NON_MATCHING
void CMorphBall::AcceptScriptMsg(EScriptObjectMessage msg, TUniqueId id, CStateManager& mgr) {
  switch (msg) {
  case kSM_Registered: {
    if (!x19d0_ballInnerGlowGen.null() && x19d0_ballInnerGlowGen->SystemHasLight()) {
      x1c10_ballInnerGlowLight = mgr.AllocateUniqueId();
      const uint inner_glow_id = x1988_ballInnerGlow.GetTag().GetId();
      mgr.AddObject(rs_new CGameLight(x1c10_ballInnerGlowLight, kInvalidAreaId, false,
                                      rstl::string_l("BallLight"), GetBallToWorld(),
                                      x0_player.GetUniqueId(), x19d0_ballInnerGlowGen->GetLight(),
                                      inner_glow_id, 0, 0.0f));
    }
  } break;

  case kSM_Deleted:
    DeleteLight(mgr);
    break;

  default:
    break;
  }
}

void CMorphBall::DeleteLight(CStateManager& mgr) {
  if (x1c10_ballInnerGlowLight != kInvalidUniqueId) {
    mgr.DeleteObjectRequest(x1c10_ballInnerGlowLight);
    x1c10_ballInnerGlowLight = kInvalidUniqueId;
  }
}

void CMorphBall::EnterMorphBallState(CStateManager& mgr) {
  x1c20_tireFactor = 0.0f;
  UpdateEffects(0.0f, mgr);
  x187c_spiderBallState = kSBS_Inactive;
  x58_ballModel->AnimationData()->SetAnimation(CAnimPlaybackParms(0, -1, 1.0f, true), false);
  x1e20_ballAnimIdx = 0;
  StopParticleWakes();
  x1c30_boostOverLightFactor = 0.0f;
  x1c34_boostLightFactor = 0.0f;
  x1c38_spiderLightFactor = 0.0f;
  DisableHalfPipeStatus();
  x30_ballTiltAngle = 0.0f;
  x2c_tireLeanAngle = 0.0f;
}

void CMorphBall::LeaveMorphBallState(CStateManager& mgr) {
  LeaveBoosting();
  CancelBoosting();
  CSfxManager::SfxStop(x1e24_boostSfxHandle);
  StopParticleWakes();
}

bool CMorphBall::GetIsInHalfPipeMode() const { return x1df8_24_inHalfPipeMode; }

void CMorphBall::SetIsInHalfPipeMode(bool val) { x1df8_24_inHalfPipeMode = val; }

bool CMorphBall::GetIsInHalfPipeModeInAir() const { return x1df8_25_inHalfPipeModeInAir; }

void CMorphBall::SetIsInHalfPipeModeInAir(bool val) { x1df8_25_inHalfPipeModeInAir = val; }

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
  if (!IsMovementAllowed() || mgr.GetPlayerState()->HasPowerUp(CPlayerState::kIT_BoostBall))
    return;

  if (!IsBoosting()) {
    CancelBoosting();
    LeaveBoosting();
  } else if (!IsBoosting() && !x1de4_25_boostEnabled) {
    if (gpTweakBall->GetBoostBallDrainTime() < x1df4_boostDrainTime) {
      LeaveBoosting();
    }

    if (!GetIsInHalfPipeMode() && !GetIsInHalfPipeModeInAir()) {
      if (x1df4_boostDrainTime / gpTweakBall->GetBoostBallDrainTime() < 0.3) {
        DampLinearAndAngularVelocities(0.5, 0.01);
      }
      LeaveBoosting();
    }
  } else {
    x1dec_timeNotInBoost += dt;
    bool jump_or_boost = ControlMapper::GetDigitalInput(ControlMapper::kC_JumpOrBoost, input);
    if (!jump_or_boost || GetSpiderBallState() == kSBS_Active) {
      if (x1e20_ballAnimIdx == 1) {
        CAnimPlaybackParms playback_parms(0, -1, 1.0, true);
        x58_ballModel->AnimationData()->SetAnimation(playback_parms, false);
        x1e20_ballAnimIdx = 0;
        CSfxManager::RemoveEmitter(x1e24_boostSfxHandle);

        if (gpTweakBall->GetBoostBallMaxChargeTime() <= x1de8_boostChargeTime) {
          CSfxHandle sfx_handle;
          // CSfxManager::AddEmitter(sfx_handle.GetIndex());
        }
      }

      if (x1de8_boostChargeTime < gpTweakBall->GetBoostBallMaxChargeTime()) {
        if (x1de8_boostChargeTime > 0.0f) {
          CancelBoosting();
        }
      } else {
        EBallBoostState ball_boost_state = GetBallBoostState();
        if (ball_boost_state == kBBS_BoostAvailable) {
          if (GetIsInHalfPipeMode() || GetIsInHalfPipeModeInAir()) {
            EnterBoosting(dt);
          } else {
            CAxisAngle angular_impulse;
            angular_impulse.FromVector(-x1924_surfaceToWorld.GetUp() * 10000.0);
            x0_player.ApplyImpulseWR(CVector3f::Zero(), angular_impulse);
            CancelBoosting();
          }
        } else if (ball_boost_state == kBBS_BoostDisabled) {
          CTransform4f ball_to_world = GetBallToWorld();
        }
      }
    } else {
    }
  }

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
